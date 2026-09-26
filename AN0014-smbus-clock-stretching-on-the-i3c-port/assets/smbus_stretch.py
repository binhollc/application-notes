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

TOOL_VERSION = "1.0"

# The reference target's protocol: [0xF0, lo, hi] sets the stretch in
# microseconds, applied after every later address match; a read returns 0xA5.
CFG_MARKER = 0xF0
READ_FILL = 0xA5

# Stretch durations the sweep visits, in microseconds. All are under the SMBus
# 35 ms timeout; section 5.3 of AN0014 covers what happens past it.
SWEEP_US = (0, 50, 500, 2000, 10000, 30000)


def number(text):
    return int(text, 0)


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


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--version", action="version", version=f"%(prog)s {TOOL_VERSION}")
    ap.add_argument("--serial", help="Supernova serial number, if more than one is connected")
    ap.add_argument("--voltage-mv", type=int, default=3300,
                    help="I3C port voltage; 1200-3300 selects the HV connector (default 3300)")
    ap.add_argument("--khz", type=int, default=100, help="SCL rate, 10-400 kHz (default 100)")
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

    args = ap.parse_args(argv)
    try:
        return args.func(args) or 0
    except p.CosmicError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
