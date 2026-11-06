#!/usr/bin/env python3
"""Set the I2C SCL duty cycle on a Binho Supernova or Pulsar.

By default the adapter picks the SCL high and low times itself. pycosmicsdk
1.6.0 adds I2cController.set_duty_cycle(high_pct), which sets the share of each
SCL period the clock is held high, 20 to 80 percent in steps of 10. Setting a
ratio also makes the frequency exact: never above the request.

    python i2c_duty_cycle.py set   --khz 100 --high 30
    python i2c_duty_cycle.py set   --khz 400 --high 40 --traffic 10
    python i2c_duty_cycle.py set   --khz 100 --high 0
    python i2c_duty_cycle.py table --khz 100 400 1000

`set` applies one ratio and, with --traffic, keeps SCL toggling for that many
seconds so an oscilloscope or logic analyzer can measure it. `table` asks the
adapter for every ratio at each frequency and prints what it reports, then
restores the default timing.

Requires pycosmicsdk 1.6.0 or later and adapter firmware 4.6.0 or later.
Older firmware refuses the call with FW_UNSUPPORTED_COMMAND.
"""

import argparse
import sys
import time

import pycosmicsdk as p

TOOL_VERSION = "1.0"

RATIOS = (0, 20, 30, 40, 50, 60, 70, 80)   # 0 = the adapter's default timing

MODELS = {"supernova": p.DeviceModel.SUPERNOVA, "pulsar": p.DeviceModel.PULSAR}


def open_bus(args):
    dev = p.Device.open(serial=args.serial, model=MODELS.get(args.model))
    bus = dev.i2c()
    bus.set_voltage(voltage_mv=args.voltage_mv)
    bus.bring_up(frequency_hz=args.khz[0] * 1000, pull_up=p.I2cPullUp[args.pull_up])
    print(f"{dev.model.name} {dev.info.fw_version}  I2C {args.voltage_mv} mV  "
          f"pull-up {args.pull_up}")
    return dev, bus


def show(khz, dc):
    ratio = "default" if dc.high_pct == 0 else f"{dc.high_pct} % high"
    print(f"request {khz:>5} kHz  ->  {dc.frequency_hz:>8} Hz  {ratio}")


def cmd_set(args):
    dev, bus = open_bus(args)
    try:
        dc = bus.set_duty_cycle(high_pct=args.high)
        show(args.khz[0], dc)
        if args.traffic:
            # With no target at the address, each read is an address and a NACK,
            # which still clocks nine SCL periods: enough to measure.
            n, end = 0, time.monotonic() + args.traffic
            while time.monotonic() < end:
                try:
                    bus.read(args.address, 16)
                except p.CosmicError:
                    pass
                n += 1
            print(f"{n} transfers to 0x{args.address:02X} in {args.traffic} s")
    finally:
        dev.close()


def cmd_table(args):
    dev, bus = open_bus(args)
    try:
        for khz in args.khz:
            bus.configure(frequency_hz=khz * 1000, pull_up=p.I2cPullUp[args.pull_up])
            for pct in RATIOS:
                show(khz, bus.set_duty_cycle(high_pct=pct))
    finally:
        bus.set_duty_cycle(high_pct=0)
        dev.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--version", action="version", version=f"%(prog)s {TOOL_VERSION}")
    ap.add_argument("--model", choices=sorted(MODELS), help="adapter model, if both are connected")
    ap.add_argument("--serial", help="adapter serial number")
    ap.add_argument("--voltage-mv", type=int, default=3300, help="bus voltage (default 3300)")
    ap.add_argument("--pull-up", default="KOHM_2_2", choices=[m.name for m in p.I2cPullUp],
                    help="on-board pull-up (default KOHM_2_2)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("set", help="apply one ratio")
    s.add_argument("--khz", type=int, nargs=1, default=[100])
    s.add_argument("--high", type=int, required=True, help="20-80 in steps of 10, or 0")
    s.add_argument("--traffic", type=float, default=0, help="seconds of SCL traffic to generate")
    s.add_argument("--address", type=lambda t: int(t, 0), default=0x50,
                   help="address the traffic reads from (default 0x50)")
    s.set_defaults(func=cmd_set)

    t = sub.add_parser("table", help="print what the adapter reports for every ratio")
    t.add_argument("--khz", type=int, nargs="+", default=[100, 400, 1000])
    t.set_defaults(func=cmd_table)

    args = ap.parse_args(argv)
    try:
        return args.func(args) or 0
    except p.CosmicError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
