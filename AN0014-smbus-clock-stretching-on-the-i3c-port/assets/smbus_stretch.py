#!/usr/bin/env python3
"""Talk to an I2C or SMBus target that stretches SCL, from the Supernova's I3C port.

The Supernova's I3C engine keeps clocking while a target holds SCL low, so a
target that stretches the clock loses bits. pycosmicsdk 1.6.0 adds a mode that
clocks legacy I2C transfers on the I3C port in software and waits for the
target, up to the SMBus 35 ms timeout. This tool turns that mode on, does the
transfer and turns it off again.

    python smbus_stretch.py read  0x2A 16
    python smbus_stretch.py read  0x0B 2 --register 0x09
    python smbus_stretch.py write 0x2A 0x01 0x02 0x03
    python smbus_stretch.py sweep --address 0x2A
    python smbus_stretch.py handoff --address 0x2A

`handoff` runs the SMBus to I3C Basic detection flow of the PCI-SIG ECN
"Chapter 12. Architectural Out-of-Band Management" as far as the adapter can:
DISEC(DISHJ) to 7Eh, SMBus discovery (static addresses and ARP Get UDID with
PEC), then, for a device that answered 7Eh, ENEC(ENHJ) and the Hot-Join.
`--t2wrst-us` resets the SMBus interface by holding SCL low (firmware 4.6.0),
and `--sda-pull-up` sets the adapter's SDA pull-up for the SMBus phase. SCL has
no pull-up on the adapter, so the external SCL pull-up is still switched by hand.

`read` and `write` work with any target. `sweep` needs the reference target
supplied with AN0014 (stretch_target_pic18q20.c): it tells the target how long
to stretch, reads from it, and reports each result.

Requires pycosmicsdk 1.6.0 or later and Supernova firmware 4.6.0 or later.
Older firmware refuses the mode with FW_UNSUPPORTED_COMMAND.
"""

import argparse
import sys
import time

import pycosmicsdk as p

TOOL_VERSION = "1.2"

ARP_ADDRESS = 0x61      # SMBus Device Default Address
ARP_GET_UDID = 0x03     # general Get UDID: count 0x11, 16 UDID bytes, address, PEC

# The reference target's protocol: [0xF0, lo, hi] sets the stretch in
# microseconds, applied after every later address match; a read returns 0xA5.
CFG_MARKER = 0xF0
READ_FILL = 0xA5

# Stretch durations the sweep visits, in microseconds. All are under the SMBus
# 35 ms timeout; section 5.3 of AN0014 covers what happens past it.
SWEEP_US = (0, 50, 500, 2000, 10000, 30000)


def number(text):
    return int(text, 0)


def pec(data):
    """SMBus PEC: CRC-8, polynomial x^8 + x^2 + x + 1, initial value 0."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else crc << 1
    return crc


assert pec(b"123456789") == 0xF4   # CRC-8/SMBUS check value


def open_port(args):
    """Open the Supernova, power the I3C port and bring the bus up."""
    dev = p.Device.open(serial=args.serial, model=p.DeviceModel.SUPERNOVA)
    i3c = dev.i3c()
    i3c.set_voltage(voltage_mv=args.voltage_mv)
    # The rates apply to I3C traffic; the stretching mode sets its own SCL rate.
    i3c.bring_up(
        push_pull_rate=p.I3cPushPullRate.PUSH_PULL_1_MHZ_DC_40,
        open_drain_rate=p.I3cOpenDrainRate.OPEN_DRAIN_100_KHZ,
        i2c_open_drain_rate=p.I2cOpenDrainRate.STANDARD_MODE,
    )
    state = i3c.enable_i2c_clock_stretching(args.khz * 1000)
    print(f"{dev.info.fw_version}  I3C port {args.voltage_mv} mV  "
          f"stretching on, SCL {state.frequency_hz} Hz")
    return dev, i3c


def close_port(dev, i3c):
    # SDR, HDR and CCC calls are refused while the mode is on; leave the port usable.
    i3c.disable_i2c_clock_stretching()
    dev.close()


def cmd_read(args):
    dev, i3c = open_port(args)
    try:
        sub = bytes([args.register]) if args.register is not None else b""
        data = i3c.legacy_i2c_read(args.address, args.length, subaddress=sub)
        print(f"read  0x{args.address:02X}: {data.hex(' ')}")
    finally:
        close_port(dev, i3c)


def cmd_write(args):
    dev, i3c = open_port(args)
    try:
        i3c.legacy_i2c_write(args.address, bytes(args.data))
        print(f"write 0x{args.address:02X}: {bytes(args.data).hex(' ')}")
    finally:
        close_port(dev, i3c)


def set_target_stretch(i3c, address, us):
    i3c.legacy_i2c_write(address, bytes([CFG_MARKER, us & 0xFF, us >> 8]))
    # The reference target restarts its I2C peripheral at the STOP to apply the
    # new stretch; an address sent before that completes is not acknowledged.
    time.sleep(0.01)


def cmd_sweep(args):
    dev, i3c = open_port(args)
    failures = 0
    try:
        print(f"{'stretch':>9}  {'reads':>5}  {'ok':>3}  {'median ms':>9}  result")
        for us in SWEEP_US:
            set_target_stretch(i3c, args.address, us)
            ok, times, errors = 0, [], {}
            for _ in range(args.count):
                t = time.perf_counter()
                try:
                    data = i3c.legacy_i2c_read(args.address, args.length)
                    ok += data == bytes([READ_FILL]) * args.length
                except p.CosmicError as e:
                    errors[e.status_code.name] = errors.get(e.status_code.name, 0) + 1
                times.append(time.perf_counter() - t)
            times.sort()
            result = "all bytes correct" if ok == args.count else f"errors {errors or 'bad data'}"
            failures += args.count - ok
            print(f"{us:>7} us  {args.count:>5}  {ok:>3}  "
                  f"{times[len(times) // 2] * 1e3:>9.2f}  {result}")
    finally:
        try:
            set_target_stretch(i3c, args.address, 0)   # never leave the target stretching
        finally:
            close_port(dev, i3c)
    return 1 if failures else 0


def smbus_discovery(i3c, addresses):
    """Probe each static address with a one-byte read, then ARP Get UDID. Needs stretching on."""
    found = []
    for a in addresses:
        try:
            i3c.legacy_i2c_read(a, 1)
            found.append(a)
            print(f"  static 0x{a:02X}: ACK")
        except p.I3cError as e:
            print(f"  static 0x{a:02X}: {e.status_code.name}")
    try:
        r = i3c.legacy_i2c_read(ARP_ADDRESS, 19, subaddress=bytes([ARP_GET_UDID]))
        frame = bytes([ARP_ADDRESS << 1, ARP_GET_UDID, (ARP_ADDRESS << 1) | 1]) + r[:-1]
        ok = r[0] == 0x11 and pec(frame) == r[-1]
        print(f"  ARP 0x61 Get UDID: {r[1:17].hex()} address 0x{r[17] >> 1:02X} "
              f"PEC {'ok' if ok else 'BAD'}")
        found.append(("arp", r[17] >> 1))
    except p.I3cError as e:
        print(f"  ARP 0x61 Get UDID: {e.status_code.name} (no ARP-capable device)")
    return found


def smbus_reset(i3c, t2wrst_us):
    """Reset the SMBus interface: SCL low for T2wrst. Returns False when not asked for."""
    if not t2wrst_us:
        print("   To reset the SMBus interface, pass --t2wrst-us (SCL low for T2wrst) or pulse SMRST#.")
        return False
    held = i3c.hold_scl_low(t2wrst_us)
    print(f"   SMBus reset: SCL held low {held} us (asked {t2wrst_us} us)")
    return True


def cmd_handoff(args):
    dev = p.Device.open(serial=args.serial, model=p.DeviceModel.SUPERNOVA)
    i3c = dev.i3c()
    try:
        i3c.set_voltage(voltage_mv=args.voltage_mv)
        i3c.bring_up(
            push_pull_rate=p.I3cPushPullRate.PUSH_PULL_1_MHZ_DC_40,
            open_drain_rate=p.I3cOpenDrainRate.OPEN_DRAIN_100_KHZ,
            i2c_open_drain_rate=p.I2cOpenDrainRate.STANDARD_MODE,
        )
        print(f"{dev.info.fw_version}  I3C port {args.voltage_mv} mV (SMBus voltage)")

        def disec():
            try:
                i3c.ccc.disec_broadcast(p.I3cEvent.HJ)
                return True
            except p.I3cError as e:
                if e.status_code.name != "FW_I3C_NACK_ADDRESS":
                    raise
                return False

        if args.sda_pull_up:
            i3c.set_sda_pull_up(p.I3cSdaPullUp[args.sda_pull_up])
        acked = disec()
        print(f"1. DISEC(DISHJ) to 7Eh: {'ACK, an I3C Basic device is present' if acked else 'NACK, SMBus only'}")
        state = i3c.enable_i2c_clock_stretching(args.khz * 1000)   # CCCs are refused from here on
        print(f"2. SMBus discovery, stretching on, SCL {state.frequency_hz} Hz")
        found = smbus_discovery(i3c, args.address)
        i3c.disable_i2c_clock_stretching()
        if not acked:
            print("   SMBus mode stays.")
            smbus_reset(i3c, args.t2wrst_us)
            return 0 if found else 1
        if found and not args.no_smbus_only:
            print("3. An SMBus-only device answered: reset the SMBus interface and stay in SMBus mode.")
            print("   Pass --no-smbus-only if the devices found are the I3C Basic device itself.")
            smbus_reset(i3c, args.t2wrst_us)
            return 0
        print(f"3. DISEC(DISHJ) to 7Eh again: {'ACK' if disec() else 'NACK'}")
        if not args.yes:
            input("4. Switch the external SMBus pull-up on SCL off, then press Enter "
                  "(the adapter's SDA pull-up follows the I3C engine) ")
        time.sleep(args.t_smb2i3c_ms / 1000)
        i3c.set_voltage(voltage_mv=args.i3c_voltage_mv)
        joined = []
        sub = dev.subscribe(p.I3cHotJoinNotification, joined.append)
        try:
            i3c.ccc.enec_broadcast(p.I3cEvent.HJ)
            print(f"5. ENEC(ENHJ) to 7Eh at {args.i3c_voltage_mv} mV: ACK")
        except p.I3cError as e:
            print(f"5. ENEC(ENHJ) to 7Eh at {args.i3c_voltage_mv} mV: {e.status_code.name}")
        deadline = time.monotonic() + args.hj_timeout_s
        while not joined and time.monotonic() < deadline:
            time.sleep(0.05)
        sub.unsubscribe()
        for n in joined:
            print(f"6. Hot-Join: PID {bytes(n.pid).hex()} BCR 0x{n.bcr:02X} DCR 0x{n.dcr:02X} "
                  f"dynamic address 0x{n.dynamic_address:02X}")
        if not joined:
            print(f"6. No Hot-Join in {args.hj_timeout_s} s: continue detection, or reset and "
                  "return to SMBus mode")
        return 0 if joined else 1
    finally:
        dev.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--version", action="version", version=f"%(prog)s {TOOL_VERSION}")
    ap.add_argument("--serial", help="Supernova serial number, if more than one is connected")
    ap.add_argument("--voltage-mv", type=int, default=3300,
                    help="I3C port voltage; 1200-3300 selects the HV connector (default 3300)")
    ap.add_argument("--khz", type=int, default=100, help="SCL rate, 10-1000 kHz; the adapter tops out near 750 kHz (default 100)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("read", help="read bytes, optionally after a register byte")
    r.add_argument("address", type=number)
    r.add_argument("length", type=int)
    r.add_argument("--register", type=number, help="register byte written before the read")
    r.set_defaults(func=cmd_read)

    w = sub.add_parser("write", help="write bytes")
    w.add_argument("address", type=number)
    w.add_argument("data", type=number, nargs="+")
    w.set_defaults(func=cmd_write)

    s = sub.add_parser("sweep", help="stretch sweep against the AN0014 reference target")
    s.add_argument("--address", type=number, default=0x2A)
    s.add_argument("--count", type=int, default=100, help="reads per stretch duration")
    s.add_argument("--length", type=int, default=16, help="bytes per read")
    s.set_defaults(func=cmd_sweep)

    h = sub.add_parser("handoff", help="SMBus to I3C Basic detection flow (PCI-SIG ECN, Chapter 12)")
    h.add_argument("--address", type=number, nargs="+", default=[0x2A],
                   help="static SMBus addresses to probe (default 0x2A)")
    h.add_argument("--i3c-voltage-mv", type=int, default=1800, help="I3C Basic voltage (default 1800)")
    h.add_argument("--t-smb2i3c-ms", type=float, default=0.0,
                   help="wait after the last 7Eh before I3C signalling (Tsmb2i3c, from the ECN)")
    h.add_argument("--hj-timeout-s", type=float, default=1.0, help="how long to wait for a Hot-Join")
    h.add_argument("--no-smbus-only", action="store_true",
                   help="treat devices found in step 2 as the I3C Basic device, not SMBus-only parts")
    h.add_argument("--yes", action="store_true", help="do not stop for the pull-up switch")
    h.add_argument("--t2wrst-us", type=int, default=0,
                   help="SMBus reset: hold SCL low this long where the flow resets the interface")
    h.add_argument("--sda-pull-up", choices=["AUTO", "OFF", "ON"],
                   help="adapter SDA pull-up during the SMBus phase (default: leave as is)")
    h.set_defaults(func=cmd_handoff)

    args = ap.parse_args(argv)
    try:
        return args.func(args) or 0
    except p.CosmicError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
