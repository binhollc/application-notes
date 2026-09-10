#!/usr/bin/env python3
"""Replay a logic-analyzer I2C capture onto a live bus from a Binho adapter.

A capture is a record of what some other controller did, against a target that
was present and answering at that moment. Replaying it puts those bytes on a
bus that may have moved on, so the interesting part of this tool is not sending
the bytes: it is what happens when the target disagrees. A target that refuses a
byte ends the transaction there, and the adapter reports which kind of refusal
it was without reporting where in the payload it landed.

    python i2c_replay.py selfcheck
    python i2c_replay.py parse    capture.csv
    python i2c_replay.py replay   capture.csv --serial <pulsar>
    python i2c_replay.py replay   capture.csv --serial <pulsar> --pace
    python i2c_replay.py replay   capture.csv --serial <pulsar> --stop-on-nak
    python i2c_replay.py arm-target     --serial <supernova>
    python i2c_replay.py release-target --serial <supernova>

The input is the CSV that Saleae's I2C analyzer writes: one row per byte, with
the columns

    Time [s],Packet ID,Address,Data,Read/Write,ACK/NAK

Numbers are read in whichever base the export used, so a capture exported as
hex, decimal or binary all parse. Rows sharing a Packet ID form one transaction;
where the analyzer emitted no Packet ID, contiguous rows with the same address
and direction are grouped instead.

Reads are re-issued, not asserted. The bytes in a captured read came from the
original target and belong to that moment; this tool asks the current target
the same question and reports whether the answer still matches.

Requires pycosmicsdk. The selfcheck and parse commands need neither the SDK
nor an adapter, so a capture can be inspected anywhere.
"""

import argparse
import csv
import io
import sys
import time

TOOL_VERSION = "1.0"

# The two ways a target refuses, as pycosmicsdk's StatusCode spells them.
# definitions.h calls them 0x0203 and 0x0204.
NACK_ADDRESS = "FW_I2C_NACK_ADDRESS"
NACK_BYTE = "FW_I2C_NACK_BYTE"

# An empty bus. Firmware answers a scan with this instead of an empty list.
NO_TARGETS = "FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED"

EXPECTED_COLUMNS = ["Time [s]", "Packet ID", "Address", "Data", "Read/Write", "ACK/NAK"]


class ReplayError(RuntimeError):
    """Anything that should stop the command with a readable message."""


# --------------------------------------------------------------------------
# The capture
# --------------------------------------------------------------------------


class Transaction:
    """One START-to-STOP transaction recovered from the capture."""

    def __init__(self, address, read, time_s, packet_id=None):
        self.address = address
        self.read = read
        self.time_s = time_s
        self.packet_id = packet_id
        self.data = bytearray()
        # Index of the first byte the captured target refused, or None if it
        # acknowledged every byte it was offered.
        self.nak_at = None
        self.address_nak = False
        self.missing_ack = False

    @property
    def direction(self):
        return "read" if self.read else "write"

    @property
    def terminating_nak(self):
        """Is this transaction's NAK just the controller ending a read?

        A controller acknowledges every byte of a read except the last, and
        NAKs that one to tell the target to stop driving. It appears in the
        export as a NAK on the final byte and it is ordinary I2C, so counting
        it as a refusal would report a healthy capture as full of failures.
        """
        return (self.read and not self.address_nak
                and self.nak_at is not None
                and self.nak_at == len(self.data) - 1)

    @property
    def refused(self):
        """Did the captured target actually refuse something?"""
        if self.address_nak:
            return True
        if self.nak_at is None or self.terminating_nak:
            return False
        return True

    def __repr__(self):
        return (f"<{self.direction} 0x{self.address:02X} "
                f"{len(self.data)}B nak_at={self.nak_at}>")

    def describe(self):
        body = " ".join(f"{b:02X}" for b in self.data) or "-"
        line = f"{self.time_s:>12.6f}s  {self.direction:<5} 0x{self.address:02X}  {body}"
        if self.address_nak:
            line += "   [captured: address NAK]"
        elif self.refused:
            line += f"   [captured: NAK at byte {self.nak_at}]"
        if self.missing_ack:
            line += "   [captured: missing ACK/NAK]"
        return line


def _number(text):
    """Read a Saleae number in whichever base the export used."""
    text = text.strip()
    if not text:
        raise ValueError("empty number")
    # int(x, 0) covers 0x48, 0b1001000 and 72; it rejects a bare leading zero,
    # which Saleae does not emit.
    return int(text, 0)


def parse_capture(handle):
    """Turn a Saleae I2C analyzer CSV export into a list of Transactions."""
    reader = csv.DictReader(handle)
    if reader.fieldnames is None:
        raise ReplayError("the capture is empty")
    missing = [c for c in EXPECTED_COLUMNS if c not in reader.fieldnames]
    if missing:
        raise ReplayError(
            "this does not look like a Saleae I2C analyzer export -- missing "
            f"column(s) {', '.join(missing)}. Found: {', '.join(reader.fieldnames)}"
        )

    transactions = []
    current = None
    current_key = None

    for line_no, row in enumerate(reader, start=2):
        packet_id = (row["Packet ID"] or "").strip()
        raw_data = (row["Data"] or "").strip()
        ack = (row["ACK/NAK"] or "").strip()
        read = (row["Read/Write"] or "").strip().lower().startswith("r")
        try:
            address = _number(row["Address"])
            time_s = float((row["Time [s]"] or "0").strip())
        except ValueError as exc:
            raise ReplayError(f"line {line_no}: {exc}") from None

        # An address-NAK row carries no data byte: the analyzer writes the
        # address line out on its own precisely because nothing followed it.
        if not raw_data:
            txn = Transaction(address, read, time_s, packet_id or None)
            txn.address_nak = True
            txn.missing_ack = ack == "Missing ACK/NAK"
            transactions.append(txn)
            current, current_key = None, None
            continue

        # Group by Packet ID where the analyzer supplied one. Where it did not,
        # a change of address or direction starts a new transaction.
        key = ("pid", packet_id) if packet_id else ("run", address, read)
        if current is None or key != current_key:
            current = Transaction(address, read, time_s, packet_id or None)
            current_key = key
            transactions.append(current)

        try:
            current.data.append(_number(raw_data))
        except ValueError as exc:
            raise ReplayError(f"line {line_no}: {exc}") from None

        if ack == "Missing ACK/NAK":
            current.missing_ack = True
        elif ack != "ACK" and current.nak_at is None:
            current.nak_at = len(current.data) - 1

    return transactions


# --------------------------------------------------------------------------
# The adapter
# --------------------------------------------------------------------------


def _sdk():
    """Import pycosmicsdk, with a readable message when it is missing."""
    try:
        import pycosmicsdk
    except ImportError:
        raise ReplayError(
            "pycosmicsdk is not installed. parse and selfcheck do not need it; "
            "replay and the target commands do."
        ) from None
    return pycosmicsdk


def nack_kind(exc):
    """Which refusal an I2cError carries, or None if it is some other failure.

    Two firmware codes, and the difference is the whole point: 0x0203 says the
    target never acknowledged its address, 0x0204 says it took the address and
    then refused a data byte. Neither says how far into the payload the refusal
    happened.
    """
    code = getattr(exc, "status_code", None)
    name = getattr(code, "name", None)
    if name in (NACK_ADDRESS, NACK_BYTE):
        return name
    return None


class Bus:
    """One I2C controller on one bus of one adapter."""

    def __init__(self, serial=None, bus="A", frequency_hz=100_000,
                 pull_up="KOHM_2_2", voltage_mv=3300, verbose=False):
        self.p = _sdk()
        self.serial = serial
        self.bus_name = bus.upper()
        self.frequency_hz = frequency_hz
        self.voltage_mv = voltage_mv
        self.verbose = verbose
        try:
            self.pull_up = getattr(self.p.I2cPullUp, pull_up)
        except AttributeError:
            offered = [n for n in dir(self.p.I2cPullUp) if not n.startswith("_")]
            raise ReplayError(
                f"unknown pull-up '{pull_up}'. This device offers: {', '.join(offered)}"
            ) from None
        try:
            self._bus = getattr(self.p.I2cBus, self.bus_name)
        except AttributeError:
            raise ReplayError(f"unknown I2C bus '{bus}', use A or B") from None
        self._device = None
        self.i2c = None
        self.info = None

    def __enter__(self):
        self._device = self.p.Device.open(serial=self.serial)
        self.info = self._device.info
        self.i2c = self._device.i2c(bus=self._bus)
        self.i2c.set_voltage(voltage_mv=self.voltage_mv)
        # bring_up, not initialize: initialize is not idempotent, and an
        # interface that is already up keeps its OLD pull-up without saying so.
        result = self.i2c.bring_up(frequency_hz=self.frequency_hz, pull_up=self.pull_up)
        if self.verbose:
            print(f"    bring_up -> {result}")
        if getattr(result, "refusal", None) is not None:
            print(f"warning: the device refused the pull-up setting "
                  f"({result.refusal}); the bus keeps whatever it had. A "
                  f"pull-up stronger than about 470 ohms can make an armed "
                  f"target invisible.", file=sys.stderr)
        return self

    def __exit__(self, *exc):
        if self._device is not None:
            self._device.close()
        return False

    def write(self, address, data):
        self.i2c.write(address=address, data=bytes(data))

    def read(self, address, length):
        return self.i2c.read(address=address, length=length)

    def scan(self):
        """Addresses answering on the bus, empty when none do.

        Firmware reports an empty bus as FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED
        rather than an empty list. That is an answer, not a failure, so it is
        translated here -- and it is worth knowing that a pull-up too strong for
        the target to overcome produces the same status as an unplugged cable.
        """
        try:
            return [int(a) for a in self.i2c.scan().addresses_7bit]
        except Exception as exc:
            code = getattr(exc, "status_code", None)
            if getattr(code, "name", None) == NO_TARGETS:
                return []
            raise


# --------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------


class Outcome:
    """What happened when one captured transaction was put back on the bus."""

    def __init__(self, txn):
        self.txn = txn
        self.status = "SUCCESS"
        self.returned = None
        self.detail = ""

    @property
    def aborted(self):
        return self.status in (NACK_ADDRESS, NACK_BYTE)

    def describe(self):
        t = self.txn
        head = f"{t.direction:<5} 0x{t.address:02X}  {len(t.data)}B"
        if self.status == NACK_ADDRESS:
            return (f"  {head}  ABORTED  address NAK -- nothing acknowledged "
                    f"0x{t.address:02X}, so no data byte was sent")
        if self.status == NACK_BYTE:
            return (f"  {head}  ABORTED  data NAK -- the target took the address, "
                    "then refused a byte; the rest was not sent")
        if self.status != "SUCCESS":
            return f"  {head}  FAILED   {self.status}"
        if t.read:
            return f"  {head}  ok       {self.detail}"
        return f"  {head}  ok"


def replay(adapter, transactions, pace=False, stop_on_nak=False, skip_captured_naks=True):
    """Put each captured transaction back on the bus, in order.

    A refusal ends its own transaction: firmware stops the transfer at the byte
    that was not acknowledged rather than clocking out the rest. This loop moves
    on to the next transaction unless stop_on_nak is set.

    The payload is replayed exactly as captured, register-pointer byte included,
    because the export does not distinguish a sub-address from the data that
    follows it and neither should the wire.
    """
    outcomes = []
    previous_time = None

    for txn in transactions:
        if txn.address_nak and skip_captured_naks:
            # The capture recorded an address that acknowledged nothing.
            # Re-issuing it tests the current bus rather than the capture, so it
            # is skipped by default.
            continue

        if pace and previous_time is not None:
            gap = txn.time_s - previous_time
            if gap > 0:
                time.sleep(gap)
        previous_time = txn.time_s

        outcome = Outcome(txn)
        try:
            if txn.read:
                returned = bytes(adapter.read(txn.address, len(txn.data)))
                outcome.returned = returned
                if returned == bytes(txn.data):
                    outcome.detail = "same bytes as the capture"
                else:
                    outcome.detail = (
                        "different bytes: capture "
                        f"{' '.join(f'{b:02X}' for b in txn.data) or '-'} -> now "
                        f"{' '.join(f'{b:02X}' for b in returned) or '-'}")
            else:
                adapter.write(txn.address, bytes(txn.data))
        except ReplayError:
            raise
        except Exception as exc:                      # an SDK CosmicError
            kind = nack_kind(exc)
            if kind is None:
                code = getattr(exc, "status_code", None)
                outcome.status = getattr(code, "name", None) or str(exc)
            else:
                outcome.status = kind

        outcomes.append(outcome)
        if outcome.aborted and stop_on_nak:
            break

    return outcomes


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def _bus_from_args(args):
    return Bus(serial=args.serial, bus=args.bus, frequency_hz=args.frequency,
               pull_up=args.pullup, verbose=args.verbose)


def cmd_parse(args):
    with open(args.capture, newline="") as handle:
        transactions = parse_capture(handle)
    writes = sum(1 for t in transactions if not t.read)
    reads = len(transactions) - writes
    naks = sum(1 for t in transactions if t.refused)
    payload = sum(len(t.data) for t in transactions)

    print(f"{args.capture}: {len(transactions)} transactions "
          f"({writes} write, {reads} read), {payload} payload bytes, "
          f"{naks} refused in the capture")
    if transactions:
        span = transactions[-1].time_s - transactions[0].time_s
        print(f"capture spans {span:.6f} s")
    if not args.quiet:
        print()
        for txn in transactions:
            print(txn.describe())
    return 0


def cmd_replay(args):
    with open(args.capture, newline="") as handle:
        transactions = parse_capture(handle)
    if not transactions:
        raise ReplayError("nothing to replay")

    print(f"{args.capture}: {len(transactions)} transactions parsed")
    with _bus_from_args(args) as adapter:
        print(f"{adapter.info.model.name} {adapter.info.serial_number} "
              f"fw {adapter.info.fw_version}, bus {adapter.bus_name} at "
              f"{args.frequency} Hz, pull-up {args.pullup}\n")
        outcomes = replay(adapter, transactions, pace=args.pace,
                          stop_on_nak=args.stop_on_nak,
                          skip_captured_naks=not args.include_captured_naks)

    for outcome in outcomes:
        print(outcome.describe())

    aborted = [o for o in outcomes if o.aborted]
    failed = [o for o in outcomes if o.status != "SUCCESS" and not o.aborted]
    changed = [o for o in outcomes if o.txn.read and o.returned is not None
               and o.returned != bytes(o.txn.data)]
    print(f"\n{len(outcomes)} replayed, {len(aborted)} aborted on a NAK, "
          f"{len(failed)} failed otherwise, "
          f"{len(changed)} read(s) answered differently than the capture")
    if aborted:
        print("A NAK ends its own transaction at the byte the target refused. "
              "The adapter reports which of the two refusals it was, not how "
              "far into the payload it happened.")
    return 1 if (aborted or failed) else 0


# --- the bench fixture -------------------------------------------------------
#
# A refusal is worth demonstrating, which means provoking one on purpose. An
# adapter can be armed as an I2C target running the EEPROM emulation model with
# busy_behavior=NACK, and it then NACKs its own address for the whole write
# cycle -- the same acknowledge-polling a 24Cxx datasheet describes. That is a
# reproducible address-phase refusal with no extra part on the bench.
#
# Two rules this obeys, both learned the hard way and both in the SDK's own
# target_eeprom_driven.py example:
#
#   * Exactly one controller. This never initialises the target adapter's
#     controller -- two controllers on one bus both drive SDA and SCL and both
#     energise the shared VTARG rail.
#   * Probe first, switch second. The target role is taken from the device
#     until it is released, so everything checkable is checked beforehand.


def _target_from_args(args):
    p = _sdk()
    device = p.Device.open(serial=args.serial)
    return p, device, device.i2c_target(bus=getattr(p.I2cBus, args.bus.upper()))


def cmd_arm_target(args):
    p, device, target = _target_from_args(args)
    try:
        target.bring_up(address=args.address,
                        address_mode=p.I2cTargetAddressMode.BIT_7,
                        address_width=p.I2cTargetMemoryLayout.BIT_8,
                        model=p.I2cTargetModel.EEPROM)
        target.configure_eeprom(
            total_size=args.total_size,
            page_size=args.page_size,
            write_cycle_us=args.write_cycle_us,
            erase_cycle_us=0,
            busy_behavior=p.I2cTargetEepromBusyBehavior.NACK,
            erased_value=0xFF,
            write_protect=None)
        print(f"armed as an EEPROM target at 0x{args.address:02X}: "
              f"{args.total_size} B, page {args.page_size} B, "
              f"write cycle {args.write_cycle_us} us, busy behaviour NACK")
        print("It now NACKs its own address for the write cycle after every "
              "write, so a replay that writes faster than that is refused at "
              "the address phase.")
    finally:
        device.close()
    return 0


def cmd_release_target(args):
    _p, device, target = _target_from_args(args)
    try:
        target.release()
        print("target released; the bus role is handed back")
    finally:
        device.close()
    return 0


def cmd_scan(args):
    with _bus_from_args(args) as adapter:
        found = adapter.scan()
    print(f"scan: {[hex(a) for a in found] or 'nothing answered'}")
    return 0


# --------------------------------------------------------------------------
# Offline self-check
# --------------------------------------------------------------------------

_HEADER = "Time [s],Packet ID,Address,Data,Read/Write,ACK/NAK"

# A two-byte write that the target acknowledged in full.
_CLEAN_WRITE = f"""{_HEADER}
0.000000000,0,0x50,0x00,Write,ACK
0.000018000,0,0x50,0xAB,Write,ACK
"""

# Nothing at the address: the analyzer writes the address row on its own, with
# no Packet ID and no data byte.
_ADDRESS_NAK = f"""{_HEADER}
0.000000000,,0x62,,Write,NAK
"""

# The target took the address and the first byte, then refused the second.
_DATA_NAK = f"""{_HEADER}
0.001000000,4,0x50,0x10,Write,ACK
0.001018000,4,0x50,0x20,Write,NAK
"""

# A read: three bytes came back from the original target.
_READ = f"""{_HEADER}
0.002000000,7,0x50,0x53,Read,ACK
0.002018000,7,0x50,0xFF,Read,ACK
0.002036000,7,0x50,0x00,Read,NAK
"""

# Exported in decimal rather than hex, and with no Packet ID at all, so
# grouping has to fall back to runs of the same address and direction.
_DECIMAL_NO_PID = f"""{_HEADER}
0.003000000,,80,1,Write,ACK
0.003018000,,80,2,Write,ACK
0.003036000,,72,9,Write,ACK
0.003054000,,72,9,Read,ACK
"""

_MISSING_ACK = f"""{_HEADER}
0.004000000,9,0x50,0x77,Write,Missing ACK/NAK
"""

_WRONG_FILE = """name,type,start_time,duration
0,frame,0.1,0.2
"""


def _parse(text):
    return parse_capture(io.StringIO(text))


def cmd_selfcheck(args):
    checks = 0

    txns = _parse(_CLEAN_WRITE)
    assert len(txns) == 1, txns
    assert txns[0].address == 0x50 and not txns[0].read
    assert bytes(txns[0].data) == b"\x00\xAB", txns[0].data
    assert txns[0].nak_at is None and not txns[0].address_nak
    checks += 1

    txns = _parse(_ADDRESS_NAK)
    assert len(txns) == 1 and txns[0].address == 0x62
    assert txns[0].address_nak and len(txns[0].data) == 0
    checks += 1

    txns = _parse(_DATA_NAK)
    assert len(txns) == 1, txns
    assert bytes(txns[0].data) == b"\x10\x20"
    # The refusal is on the second byte, so index 1 -- not the address, and not
    # the whole transaction.
    assert txns[0].nak_at == 1, txns[0].nak_at
    assert not txns[0].address_nak
    checks += 1

    txns = _parse(_READ)
    assert len(txns) == 1 and txns[0].read
    assert bytes(txns[0].data) == b"\x53\xFF\x00"
    # A controller NAKs the last byte of a read to end it; that is normal and
    # must not be read as the target refusing anything.
    assert txns[0].nak_at == 2, txns[0].nak_at
    assert txns[0].terminating_nak and not txns[0].refused
    checks += 1

    # A NAK anywhere else inside a read IS a refusal: the target stopped
    # driving early.
    early = _parse(_READ.replace("0.002018000,7,0x50,0xFF,Read,ACK",
                                 "0.002018000,7,0x50,0xFF,Read,NAK"))
    assert early[0].nak_at == 1 and early[0].refused
    assert not early[0].terminating_nak
    checks += 1

    # A write's final-byte NAK is never a terminator -- only reads have one.
    assert _parse(_DATA_NAK)[0].refused
    assert not _parse(_DATA_NAK)[0].terminating_nak
    checks += 1

    txns = _parse(_DECIMAL_NO_PID)
    assert len(txns) == 3, txns
    assert txns[0].address == 80 and bytes(txns[0].data) == b"\x01\x02"
    assert txns[1].address == 72 and not txns[1].read
    assert txns[2].address == 72 and txns[2].read
    checks += 1

    # "Missing ACK/NAK" is the analyzer saying it could not see the ninth bit --
    # a capture-quality problem, usually a threshold or sample-rate one. It is
    # flagged, but it is not a refusal and must not be counted as one.
    txns = _parse(_MISSING_ACK)
    assert txns[0].missing_ack and txns[0].nak_at is None
    checks += 1

    try:
        _parse(_WRONG_FILE)
    except ReplayError as exc:
        assert "Saleae" in str(exc), exc
        checks += 1
    else:
        raise AssertionError("a non-I2C export should be refused, not parsed")

    assert _number("0x50") == 0x50
    assert _number("80") == 80
    assert _number("0b1010000") == 0x50
    checks += 1

    # Replay against stand-ins that refuse, to prove the loop reports one abort
    # per transaction, keeps going by default, and stops when told to.
    class Refusing:
        def __init__(self, status_name):
            self.status_name = status_name

        def _raise(self):
            code = type("StatusCode", (), {"name": self.status_name})()
            exc = RuntimeError(f"i2c_write: {self.status_name}")
            exc.status_code = code
            raise exc

        def write(self, address, data):
            self._raise()

        def read(self, address, length):
            self._raise()

    both = _parse(_CLEAN_WRITE) + _parse(_DATA_NAK)
    outcomes = replay(Refusing(NACK_ADDRESS), both)
    assert len(outcomes) == 2, outcomes
    assert all(o.status == NACK_ADDRESS and o.aborted for o in outcomes)
    outcomes = replay(Refusing(NACK_BYTE), both, stop_on_nak=True)
    assert len(outcomes) == 1 and outcomes[0].status == NACK_BYTE
    checks += 1

    # A failure that is not a NAK is reported as itself, not as an abort.
    outcomes = replay(Refusing("FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED"),
                      _parse(_CLEAN_WRITE))
    assert outcomes[0].status == "FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED"
    assert not outcomes[0].aborted
    checks += 1

    # A captured address-NAK is skipped by default and included on request.
    assert replay(Refusing(NACK_ADDRESS), _parse(_ADDRESS_NAK)) == []
    assert len(replay(Refusing(NACK_ADDRESS), _parse(_ADDRESS_NAK),
                      skip_captured_naks=False)) == 1
    checks += 1

    # A read whose answer changed is reported as changed, not as a failure.
    class Answering:
        def write(self, address, data):
            return None

        def read(self, address, length):
            return b"\x00" * length

    outcomes = replay(Answering(), _parse(_READ))
    assert outcomes[0].status == "SUCCESS"
    assert outcomes[0].returned == b"\x00\x00\x00"
    assert "different bytes" in outcomes[0].detail
    checks += 1

    # An empty bus is a scan result, not a crash; anything else still raises.
    class Scanner:
        def __init__(self, status_name):
            self.status_name = status_name

        def scan(self, **kwargs):
            exc = RuntimeError(self.status_name)
            exc.status_code = type("StatusCode", (), {"name": self.status_name})()
            raise exc

    empty = Bus.__new__(Bus)
    empty.i2c = Scanner(NO_TARGETS)
    assert empty.scan() == []
    other = Bus.__new__(Bus)
    other.i2c = Scanner("FW_I2C_ARBITRATION_LOST")
    try:
        other.scan()
    except RuntimeError as exc:
        assert "ARBITRATION" in str(exc)
        checks += 1
    else:
        raise AssertionError("a non-empty-bus scan failure must not be swallowed")

    print(f"i2c_replay.py {TOOL_VERSION} self-check: {checks}/{checks} OK")
    return 0


# --------------------------------------------------------------------------


def main(argv=None):
    parent = argparse.ArgumentParser(add_help=False)
    parent.add_argument("--serial", help="adapter serial number; required when "
                                         "more than one adapter is plugged in")
    parent.add_argument("--verbose", action="store_true", help="print SDK detail")
    parent.add_argument("--bus", default="A", help="I2C bus, A or B (default A)")
    parent.add_argument("--frequency", type=int, default=100_000,
                        help="I2C clock in Hz (default 100000)")
    parent.add_argument("--pullup", default="KOHM_2_2",
                        help="I2cPullUp member name (default KOHM_2_2). A pull-up "
                             "stronger than about 470 ohms can make an armed "
                             "target invisible -- see the note.")

    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--version", action="version",
                        version=f"i2c_replay.py {TOOL_VERSION}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    sub = subparsers.add_parser("selfcheck", parents=[parent],
                                help="check the parser offline, no adapter needed")
    sub.set_defaults(func=cmd_selfcheck)

    sub = subparsers.add_parser("parse", parents=[parent],
                                help="parse a capture and print the transactions")
    sub.add_argument("capture", help="Saleae I2C analyzer CSV export")
    sub.add_argument("--quiet", action="store_true", help="totals only")
    sub.set_defaults(func=cmd_parse)

    sub = subparsers.add_parser("replay", parents=[parent],
                                help="replay a capture onto the bus")
    sub.add_argument("capture", help="Saleae I2C analyzer CSV export")
    sub.add_argument("--pace", action="store_true",
                     help="wait out the capture's own inter-transaction gaps")
    sub.add_argument("--stop-on-nak", action="store_true",
                     help="stop the whole replay at the first refused transaction")
    sub.add_argument("--include-captured-naks", action="store_true",
                     help="also re-issue transactions that were refused in the capture")
    sub.set_defaults(func=cmd_replay)

    sub = subparsers.add_parser("scan", parents=[parent],
                                help="list the addresses answering on the bus")
    sub.set_defaults(func=cmd_scan)

    sub = subparsers.add_parser("arm-target", parents=[parent],
                                help="arm an adapter as an EEPROM target that NACKs while busy")
    sub.add_argument("--address", type=lambda s: int(s, 0), default=0x50)
    sub.add_argument("--total-size", type=int, default=256)
    sub.add_argument("--page-size", type=int, default=8)
    sub.add_argument("--write-cycle-us", type=int, default=5000,
                     help="how long the target stays busy after a write (default 5000)")
    sub.set_defaults(func=cmd_arm_target)

    sub = subparsers.add_parser("release-target", parents=[parent],
                                help="release the target role")
    sub.set_defaults(func=cmd_release_target)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except ReplayError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    except Exception as exc:
        # An SDK CosmicError carries a status_code; anything that does is a
        # device or bus condition and belongs on one line, not in a traceback.
        code = getattr(exc, "status_code", None)
        if code is None:
            raise
        name = getattr(code, "name", None) or f"0x{code:04X}"
        print(f"error: {name}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
