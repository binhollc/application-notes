#!/usr/bin/env python3
"""Exercise a Binho Pulsar's CAN-FD (SLCAN) bridge from the command line.

    canfd_slcan.py mode                      # what CDC mode the Pulsar is in
    canfd_slcan.py probe --port COM12        # SLCAN identity and command checks
    canfd_slcan.py listen --port COM12       # decode frames off the bus
    canfd_slcan.py send --port COM12 --id 123 --data DEADBEEF --fd --brs
    canfd_slcan.py roundtrip --port COM12    # request/response, classic and FD
    canfd_slcan.py rates                     # the bitrate tables, and a trap

Two layers are deliberately kept apart here.

`probe`, `listen` and `send` speak SLCAN over the serial port directly, with no
CAN library involved. That is the layer where a problem is diagnosable: you can
see the exact ASCII the firmware received and the exact `\\r` or `\\a` it
answered with.

`roundtrip` goes through `python-can`'s stock `slcan` interface instead, which
is how a reader will actually use the adapter. Running both against the same
hardware is the point: if the raw layer works and the library layer does not,
the fault is in how the library is being driven rather than in the bridge.
"""

from __future__ import annotations

import argparse
import sys
import time

# SLCAN nominal bitrate codes, from the firmware's own table.
NOMINAL_CODES = {
    10_000: "S0", 20_000: "S1", 50_000: "S2", 100_000: "S3", 125_000: "S4",
    250_000: "S5", 500_000: "S6", 800_000: "S7", 1_000_000: "S8",
}

# CAN-FD data-phase codes. Only rates that are exact at the 50 MHz CAN core
# clock are offered by the firmware; anything else answers with a bell.
DATA_CODES = {1_000_000: "Y1", 2_000_000: "Y2", 5_000_000: "Y5"}

# DLC digit to payload length. 0-8 are literal; 9-F are the FD steps.
DLC_TO_LEN = {**{i: i for i in range(9)},
              9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}
LEN_TO_DLC = {v: k for k, v in sorted(DLC_TO_LEN.items())}

OK, BELL = b"\r", b"\a"


# --------------------------------------------------------------------------
# Raw SLCAN
# --------------------------------------------------------------------------


class Slcan:
    """The SLCAN line protocol over a serial port, with nothing in between.

    The port's baud rate is irrelevant: this is a USB CDC endpoint, not a
    UART, so the number is discarded. The CAN bitrate comes from the `S` and
    `Y` commands.
    """

    def __init__(self, port: str, timeout: float = 0.4):
        import serial
        self.serial = serial.Serial(port, 115200, timeout=timeout)
        time.sleep(0.3)
        self.serial.reset_input_buffer()

    def close(self) -> None:
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        try:
            self.command("C")       # never leave the channel open
        except Exception:
            pass
        self.close()

    def command(self, text: str, wait: float = 0.3) -> bytes:
        self.serial.write(text.encode() + b"\r")
        self.serial.flush()
        time.sleep(wait)
        return self.serial.read(4096)

    def open_channel(self, nominal: int, data: int | None,
                     listen_only: bool = False) -> None:
        """Set the bitrates, then open. That order is not a style choice.

        The firmware captures S and Y when it processes the open command, so a
        bitrate set afterwards is accepted, acknowledged, and ignored until the
        channel is closed and reopened.
        """
        self.command("C")
        if nominal not in NOMINAL_CODES:
            raise SystemExit(f"nominal {nominal} is not one of "
                             f"{sorted(NOMINAL_CODES)}")
        if self.command(NOMINAL_CODES[nominal]) != OK:
            raise SystemExit(f"{NOMINAL_CODES[nominal]} was refused")
        if data is not None:
            if data not in DATA_CODES:
                raise SystemExit(f"data {data} is not one of {sorted(DATA_CODES)}")
            if self.command(DATA_CODES[data]) != OK:
                raise SystemExit(f"{DATA_CODES[data]} was refused")
        self.serial.reset_input_buffer()
        self.command("L" if listen_only else "O")

    def read_frames(self, seconds: float):
        """Collect whole SLCAN frame strings for a while.

        Frames are `\\r`-terminated and arrive unsolicited once the channel is
        open, so this keeps a partial tail rather than splitting a frame that
        was still in flight when the window closed.
        """
        end = time.time() + seconds
        buffer = b""
        while time.time() < end:
            buffer += self.serial.read(8192)
        parts = buffer.split(b"\r")
        return [p.decode("ascii", "replace") for p in parts[:-1] if p]


def decode(frame: str) -> dict | None:
    """Turn one SLCAN frame string into its fields.

    The leading letter carries three separate facts at once: whether the frame
    is classic or FD, whether the data phase switched rate, and whether the ID
    is standard or extended. Lowercase is standard, uppercase extended.
    """
    if not frame:
        return None
    kind = frame[0]
    shapes = {"t": ("classic", False), "T": ("classic", True),
              "d": ("fd", False), "D": ("fd", True),
              "b": ("fd+brs", False), "B": ("fd+brs", True)}
    if kind not in shapes:
        return None
    form, extended = shapes[kind]
    id_digits = 8 if extended else 3
    body = frame[1:]
    if len(body) < id_digits + 1:
        return None
    # Everything here is parsed defensively and returns None on anything
    # malformed. A read window can open mid-frame, so the first string out of
    # the buffer is routinely a tail with no leading letter or an odd number of
    # hex digits. That is normal, not an error, and it must not stop a monitor.
    try:
        identifier = int(body[:id_digits], 16)
        dlc = int(body[id_digits], 16)
        digits = body[id_digits + 1:]
        payload = bytes.fromhex(digits) if digits else b""
    except ValueError:
        return None
    return {"kind": form, "ext": extended, "id": identifier, "dlc": dlc,
            "expected_len": DLC_TO_LEN[dlc], "data": payload}


def encode(identifier: int, payload: bytes, fd: bool, brs: bool,
           extended: bool) -> str:
    """Build a transmit frame string, padding to a legal FD length.

    Classic CAN takes any length to 8. FD only has the DLC steps, so a
    13-byte payload has to become 16 with the remainder zeroed; sending 13 is
    not representable and the firmware would reject the DLC.
    """
    length = len(payload)
    if fd:
        for candidate in sorted(LEN_TO_DLC):
            if candidate >= length:
                payload = payload + bytes(candidate - length)
                length = candidate
                break
        else:
            raise SystemExit(f"payload of {length} bytes exceeds the 64-byte maximum")
    elif length > 8:
        raise SystemExit(f"classic CAN carries at most 8 bytes, got {length}")

    if fd:
        letter = "b" if brs else "d"
    else:
        letter = "t"
    if extended:
        letter = letter.upper()
    width = 8 if extended else 3
    return (f"{letter}{identifier:0{width}X}{LEN_TO_DLC[length]:X}"
            f"{payload.hex().upper()}")


def show(entry: dict, prefix: str = "   ") -> str:
    body = entry["data"].hex().upper()
    if len(body) > 32:
        body = body[:32] + "..."
    flag = "" if len(entry["data"]) == entry["expected_len"] else "  LENGTH MISMATCH"
    return (f"{prefix}id=0x{entry['id']:03X} {'ext' if entry['ext'] else 'std'} "
            f"{entry['kind']:<7} dlc={entry['dlc']:X} len={len(entry['data'])} "
            f"{body}{flag}")


# --------------------------------------------------------------------------
# The demo node's message catalogue
#
# Split the way the node is: the diagnostic messages exist to prove the bus
# works, and the device messages exist to give a host something worth
# decoding. Everything is little-endian, which CAN does not dictate, so it has
# to be agreed in writing or the two ends quietly disagree.
# --------------------------------------------------------------------------

HEARTBEAT_ID = 0x100
ENVIRONMENT_ID = 0x110
FD_RAMP_ID = 0x200
TELEMETRY_ID = 0x210
ECHO_RSP_ID = 0x321
COMMAND_REQ_ID = 0x600
COMMAND_RSP_ID = 0x601

COMMANDS = {"info": 0x01, "counters": 0x02, "set-rate": 0x03}


def _u16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little")


def _i16(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 2], "little", signed=True)


def _u32(data: bytes, offset: int) -> int:
    return int.from_bytes(data[offset:offset + 4], "little")


def explain(entry: dict) -> str | None:
    """Decode a catalogue message into readable fields, or None if unknown.

    Returning None rather than guessing matters on a shared bus: an unknown ID
    is somebody else's message, and inventing an interpretation for it is worse
    than saying nothing.
    """
    data, identifier = entry["data"], entry["id"]

    if identifier == HEARTBEAT_ID and len(data) >= 8:
        return (f"heartbeat seq={_u32(data, 0)} rx={_u16(data, 4)} "
                f"echoes={_u16(data, 6)}")

    if identifier == ENVIRONMENT_ID and len(data) >= 8:
        return (f"environment {_i16(data, 0) / 10:.1f} C  "
                f"{_u16(data, 2) / 10:.1f} %RH  {_u16(data, 4) / 10:.1f} hPa  "
                f"flags=0x{data[6]:02X} seq={data[7]}")

    if identifier == FD_RAMP_ID:
        ramp = all((data[i] - data[0]) % 256 == i for i in range(len(data)))
        return (f"fd ramp {len(data)} bytes, first=0x{data[0]:02X}"
                f"{'' if ramp else '  NOT A CLEAN RAMP'}")

    if identifier == TELEMETRY_ID and len(data) >= 12:
        count = _u16(data, 8)
        available = (len(data) - 12) // 2
        samples = [_i16(data, 12 + 2 * i) for i in range(min(count, available))]
        peak = max((abs(s) for s in samples), default=0)
        head = ", ".join(str(s) for s in samples[:6])
        return (f"trace seq={_u32(data, 0)} uptime={_u32(data, 4)} ms  "
                f"{count} samples peak={peak}  [{head}, ...]")

    if identifier == COMMAND_RSP_ID and data:
        code = data[0]
        if code == COMMANDS["info"] and len(data) >= 8:
            tag = bytes(data[1:4]).decode("ascii", "replace")
            return (f"cmd INFO  id={tag} fw={data[4]}.{data[5]} "
                    f"features=0x{data[6]:02X} trace_samples={data[7]}")
        if code == COMMANDS["counters"] and len(data) >= 8:
            return (f"cmd COUNTERS tx={_u16(data, 1)} rx={_u16(data, 3)} "
                    f"echoes={_u16(data, 5)} commands={data[7]}")
        if code == COMMANDS["set-rate"] and len(data) >= 4:
            verdict = "accepted" if data[3] == 0 else f"status 0x{data[3]:02X}"
            return f"cmd SET_RATE period={_u16(data, 1)} ms, {verdict}"
        if code == 0xFF and len(data) >= 2:
            return f"cmd ERROR: node did not understand 0x{data[1]:02X}"
        return f"cmd 0x{code:02X}, {len(data)} bytes"

    if identifier == ECHO_RSP_ID:
        return f"echo reply, {len(data)} bytes"

    return None


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------


def cmd_mode(args) -> int:
    """Report the Pulsar's CDC mode without changing it."""
    try:
        import hid  # noqa: F401
    except ImportError:
        print("  hidapi is not installed; run: pip install hidapi")
        return 1
    print("  CAN-FD mode is a CDC functional mode, set over HID and stored in")
    print("  flash. Use the firmware's own tool, which resets the device so the")
    print("  setting persists:")
    print()
    print("    python device_settings_cli.py --device pulsar show")
    print("    python device_settings_cli.py --device pulsar set --cdc-mode canfd")
    print("    python device_settings_cli.py --device pulsar set --cdc-mode uart_passthrough")
    print()
    print("  While in CAN-FD mode the I2C/SPI/UART/GPIO managers do not run:")
    print("  the device is a dedicated SLCAN bridge until the mode is changed.")
    return 0


def cmd_probe(args) -> int:
    """Check the bridge answers SLCAN, before blaming the bus for anything."""
    with Slcan(args.port) as link:
        link.command("C")
        checks = [
            ("V", b"V0101\r", "version"),
            ("N", b"NFFFF\r", "serial number"),
            ("F", b"F00\r", "status flags"),
            (NOMINAL_CODES[args.bitrate], OK, f"nominal {args.bitrate}"),
            ("Q", BELL, "an unsupported command should bell"),
        ]
        if args.data_bitrate:
            checks.insert(4, (DATA_CODES[args.data_bitrate], OK,
                              f"data phase {args.data_bitrate}"))
        worst = 0
        for text, expected, label in checks:
            got = link.command(text)
            good = got == expected
            worst |= 0 if good else 1
            print(f"  {text:<3} -> {got!r:<12} {'ok' if good else 'UNEXPECTED'}"
                  f"   {label}")
        print()
        print("  V, N and F return fixed constants in this firmware; they prove"
              " the\n  bridge is listening, not what device it is.")
        return worst


def cmd_listen(args) -> int:
    with Slcan(args.port) as link:
        link.open_channel(args.bitrate, args.data_bitrate, args.listen_only)
        mode = "listen-only" if args.listen_only else "normal"
        print(f"  channel open ({mode}), {args.bitrate} nominal"
              + (f", {args.data_bitrate} data" if args.data_bitrate else "")
              + f"; collecting for {args.seconds:g} s")
        frames = link.read_frames(args.seconds)
        counts: dict[tuple, int] = {}
        for raw in frames:
            entry = decode(raw)
            if entry is None:
                continue
            key = (entry["id"], entry["kind"], len(entry["data"]))
            counts[key] = counts.get(key, 0) + 1
            if args.all:
                print(show(entry))
        if not args.all:
            for (identifier, kind, length), n in sorted(counts.items()):
                print(f"   id=0x{identifier:03X} {kind:<7} len={length:<3} "
                      f"x{n}")
        print(f"  {len(frames)} frame string(s), "
              f"{sum(counts.values())} decoded, {len(counts)} distinct")
        return 0 if frames else 1


def cmd_send(args) -> int:
    payload = bytes.fromhex(args.data) if args.data else b""
    with Slcan(args.port) as link:
        link.open_channel(args.bitrate, args.data_bitrate)
        text = encode(int(args.id, 16), payload, args.fd, args.brs, args.ext)
        print(f"  TX {text[:48]}{'...' if len(text) > 48 else ''}")
        reply = link.command(text, wait=0.4)
        # The bridge answers the command itself with CR or BELL, and any bus
        # frames that arrived meanwhile follow in the same read.
        head = reply[:1]
        print(f"  bridge said {head!r}"
              f" {'(accepted)' if head == OK else '(rejected)' if head == BELL else ''}")
        for raw in reply.split(b"\r"):
            entry = decode(raw.decode("ascii", "replace"))
            if entry:
                print(show(entry, "   RX "))
        return 0 if head == OK else 1


def cmd_roundtrip(args) -> int:
    """Request/response through python-can, which is how readers will use it."""
    try:
        import can
    except ImportError:
        print("  python-can is not installed; run: pip install python-can")
        return 1

    def drain(bus, seconds: float = 1.5) -> int:
        """Empty the queue and say how much was waiting.

        This matters more than it looks. The bridge buffers bus traffic while
        the host is opening the port and cycling the channel to set bitrates,
        so by the time a request goes out there is a backlog ahead of its
        reply. Reading "the next few frames" after sending reads history, and
        a working node looks like it never answered.
        """
        seen = 0
        end = time.time() + seconds
        while time.time() < end:
            if bus.recv(timeout=0.05) is None:
                break
            seen += 1
        return seen

    def await_reply(bus, identifier: int, seconds: float = 2.0):
        end = time.time() + seconds
        while time.time() < end:
            message = bus.recv(timeout=0.2)
            if message is not None and message.arbitration_id == identifier:
                return message
        return None

    bus = can.Bus(interface="slcan", channel=args.port, bitrate=args.bitrate)
    failures = 0
    try:
        if args.data_bitrate:
            # set_bitrate closes the channel, writes S and Y, and reopens it,
            # which is exactly the order the firmware needs.
            bus.set_bitrate(args.bitrate, args.data_bitrate)
        print(f"  python-can slcan on {args.port}: {args.bitrate} nominal"
              + (f", {args.data_bitrate} data" if args.data_bitrate else ""))

        for label, kwargs, payload in (
            ("classic", {}, bytes([0xDE, 0xAD, 0xBE, 0xEF, 0, 1, 2, 3])),
            ("CAN-FD, 64 bytes, BRS",
             {"is_fd": True, "bitrate_switch": True}, bytes(range(64))),
        ):
            print(f"\n  --- {label} ---")
            print(f"    drained {drain(bus)} buffered frame(s) first")
            bus.send(can.Message(arbitration_id=int(args.request, 16),
                                 is_extended_id=False, data=payload, **kwargs))
            reply = await_reply(bus, int(args.response, 16))
            if reply is None:
                print(f"    no reply on 0x{int(args.response, 16):03X}")
                failures += 1
                continue
            kind = "classic"
            if reply.is_fd:
                kind = "fd+brs" if reply.bitrate_switch else "fd"
            print(f"    reply id=0x{reply.arbitration_id:03X} {kind} "
                  f"dlc={reply.dlc} len={len(reply.data)}")
            same = bytes(reply.data) == payload
            print(f"    payload identical: {same}")
            if not same:
                failures += 1
    finally:
        bus.shutdown()
    return 1 if failures else 0


def cmd_monitor(args) -> int:
    """Decode the demo node's catalogue rather than dumping hex.

    Opens in normal mode by default, which on a two-node bus is the only
    choice that works. See the warning under --listen-only.
    """
    with Slcan(args.port) as link:
        link.open_channel(args.bitrate, args.data_bitrate, args.listen_only)
        mode = "listen-only" if args.listen_only else "normal"
        print(f"  {mode}, {args.bitrate} nominal"
              + (f", {args.data_bitrate} data" if args.data_bitrate else "")
              + f"; {args.seconds:g} s\n")
        if args.listen_only:
            print("  WARNING: a listen-only adapter does not acknowledge. If it")
            print("  is the only other node, every frame the peer sends goes")
            print("  unacknowledged, is retried indefinitely, and the peer never")
            print("  advances past its first frame. Expect thousands of copies")
            print("  of one message. Only use this where something else ACKs.\n")
        latest: dict[int, str] = {}
        unknown: dict[int, int] = {}
        decoded = 0
        for raw in link.read_frames(args.seconds):
            entry = decode(raw)
            if entry is None:
                continue
            text = explain(entry)
            if text is None:
                unknown[entry["id"]] = unknown.get(entry["id"], 0) + 1
                continue
            decoded += 1
            latest[entry["id"]] = text
            if args.all:
                print(f"   0x{entry['id']:03X}  {text}")
        if not args.all:
            for identifier in sorted(latest):
                print(f"   0x{identifier:03X}  {latest[identifier]}")
        print(f"\n  {decoded} catalogue frame(s) decoded")
        if unknown:
            others = ", ".join(f"0x{i:03X} x{n}" for i, n in sorted(unknown.items()))
            print(f"  not in the catalogue: {others}")
        return 0 if decoded else 1


def cmd_command(args) -> int:
    """Send a command to the node and decode its reply."""
    payload = bytes([COMMANDS[args.name]])
    if args.name == "set-rate":
        payload += int(args.period).to_bytes(2, "little")

    with Slcan(args.port) as link:
        link.open_channel(args.bitrate, args.data_bitrate)
        # Drain first. The node transmits continuously, so a reply sits behind
        # whatever the bridge buffered while the channel was being set up.
        link.serial.reset_input_buffer()
        text = encode(COMMAND_REQ_ID, payload, fd=False, brs=False, extended=False)
        print(f"  TX {text}   ({args.name})")
        deadline = time.time() + 2.0
        link.serial.write(text.encode() + b"\r")
        link.serial.flush()
        while time.time() < deadline:
            for raw in link.read_frames(0.3):
                entry = decode(raw)
                if entry and entry["id"] == COMMAND_RSP_ID:
                    print(f"  RX 0x{COMMAND_RSP_ID:03X}  {explain(entry)}")
                    return 0
        print(f"  no reply on 0x{COMMAND_RSP_ID:03X}")
        return 1


def cmd_rates(args) -> int:
    print("  Nominal (arbitration) bitrates, SLCAN 'S':")
    for rate, code in sorted(NOMINAL_CODES.items()):
        print(f"    {code}  {rate:>9,} bit/s")
    print("\n  CAN-FD data-phase bitrates, SLCAN 'Y':")
    for rate, code in sorted(DATA_CODES.items()):
        print(f"    {code}  {rate:>9,} bit/s")
    print("\n  DLC to payload length:")
    print("    " + "  ".join(f"{d:X}={DLC_TO_LEN[d]}" for d in range(16)))
    print("\n  One trap worth knowing. python-can's slcan backend maps")
    print("  750000 -> S7, and this firmware's S7 is 800 kbit/s. Asking for")
    print("  750 kbit/s therefore puts the bus at 800 kbit/s with no warning.")
    print("  Its data-phase table offers only 2 Mbit/s and 5 Mbit/s, so Y1 is")
    print("  reachable from the raw protocol but not through that backend.")
    return 0


# --------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="canfd_slcan.py",
        description="Drive a Binho Pulsar's CAN-FD (SLCAN) bridge.")
    subparsers = parser.add_subparsers(dest="command", metavar="command")

    def add(name, function, help_text, wants_port=True):
        sub = subparsers.add_parser(name, help=help_text)
        sub.set_defaults(function=function)
        if wants_port:
            sub.add_argument("--port", required=True,
                             help="serial port of the bridge, e.g. COM12")
            sub.add_argument("--bitrate", type=int, default=500_000,
                             choices=sorted(NOMINAL_CODES),
                             help="nominal bitrate (default 500000)")
            sub.add_argument("--data-bitrate", type=int, default=None,
                             choices=sorted(DATA_CODES),
                             help="CAN-FD data-phase bitrate")
        return sub

    add("mode", cmd_mode, "how to switch the Pulsar into CAN-FD mode",
        wants_port=False)
    add("probe", cmd_probe, "check the bridge answers SLCAN")

    sub = add("listen", cmd_listen, "decode frames off the bus")
    sub.add_argument("--seconds", type=float, default=4.0)
    sub.add_argument("--all", action="store_true",
                     help="print every frame instead of a summary")
    sub.add_argument("--listen-only", action="store_true",
                     help="open with 'L', so the adapter never transmits")

    sub = add("send", cmd_send, "transmit one frame")
    sub.add_argument("--id", required=True, help="hex identifier, e.g. 123")
    sub.add_argument("--data", default="", help="hex payload, e.g. DEADBEEF")
    sub.add_argument("--fd", action="store_true", help="send as CAN-FD")
    sub.add_argument("--brs", action="store_true",
                     help="switch to the data-phase rate (implies --fd)")
    sub.add_argument("--ext", action="store_true", help="29-bit identifier")

    sub = add("monitor", cmd_monitor, "decode the demo node's message catalogue")
    sub.add_argument("--seconds", type=float, default=4.0)
    sub.add_argument("--all", action="store_true",
                     help="print every frame instead of the latest per id")
    sub.add_argument("--listen-only", action="store_true",
                     help="open with 'L'. Only valid where another node "
                          "acknowledges; on a two-node bus it stalls the peer")

    sub = add("command", cmd_command, "send a command to the node and decode the reply")
    sub.add_argument("name", choices=sorted(COMMANDS),
                     help="which command to send")
    sub.add_argument("--period", type=int, default=100,
                     help="period in ms for set-rate (clamped 10..10000)")

    sub = add("roundtrip", cmd_roundtrip,
              "request/response via python-can, classic and FD")
    sub.add_argument("--request", default="123", help="request id (hex)")
    sub.add_argument("--response", default="321", help="expected reply id (hex)")

    add("rates", cmd_rates, "bitrate tables and the DLC mapping",
        wants_port=False)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if not getattr(args, "command", None):
        build_parser().print_help()
        return 2
    if getattr(args, "brs", False):
        args.fd = True
    try:
        return args.function(args)
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
