#!/usr/bin/env python3
"""SMBus to I3C bring-up on one Supernova I3C port: 0x7E discovery at 3.3 V, then DAA at 1.8 V.

    1. Operate in SMBus mode at 3.3 V.
    2. Broadcast a 0x7E command (DISEC, disable Hot-Join): an ACK means an I3C device is on the bus.
    3. Probe every SMBus address: a device that answers and is not a known I3C device is SMBus-only.
       If one remains, stop here and stay in SMBus mode.
    4. Move the bus and its pull-ups from 3.3 V to 1.8 V.
    5. Assign dynamic addresses: SETDASA (static -> dynamic) or ENTDAA, then check each device's PID.

    python smbus_to_i3c_daa.py --i3c-static 0x3A --daa setdasa
    python smbus_to_i3c_daa.py --i3c-static 0x3A --daa entdaa

--emulate-target SERIAL turns a second Supernova into the I3C device (static address = the first
--i3c-static value), so the flow runs with no DUT. Needs pycosmicsdk and Supernova fw 4.6.0 or later.
"""
import argparse
import sys
import time

import pycosmicsdk as p

EMULATED_PID = bytes.fromhex("0B0000000A01")   # any 48-bit value; the controller reads it back


def number(text):
    return int(text, 0)


def open_supernova(serial):
    devs = [d for d in p.list_devices() if d.model is p.DeviceModel.SUPERNOVA and (not serial or d.serial_number.startswith(serial))]
    if not devs:
        sys.exit(f"no Supernova matches {serial or 'any'}")
    return p.Device.open(serial=devs[0].serial_number)


def smbus_scan(i3c, first, last):
    """Addresses that ACK a one-byte SMBus read."""
    found = []
    for a in range(first, last + 1):
        try:
            i3c.legacy_i2c_read(a, 1)
            found.append(a)
        except p.I3cError:
            pass
    return found


def ack_7e(i3c):
    try:
        i3c.ccc.disec_broadcast(p.I3cEvent.HJ)
        return True
    except p.I3cError as e:
        if e.status_code.name != "FW_I3C_NACK_ADDRESS":
            raise
        return False


def run(args, ctl, tgt_dev):
    i3c = ctl.i3c()
    step = lambda text: print(text, flush=True)

    # 1. SMBus mode at 3.3 V
    i3c.set_voltage(voltage_mv=args.smbus_mv)
    if tgt_dev:
        tgt_dev.i3c().set_voltage(voltage_mv=args.smbus_mv)
        tgt_dev.i3c_target().initialize(memory_layout=p.I3cTargetMemoryLayout.MEMORY_1_BYTE_X_1024_REGS,
                                        pid=EMULATED_PID, bcr=0x06, dcr=0xC5, static_address=args.i3c_static[0])
    i3c.bring_up(push_pull_rate=p.I3cPushPullRate.PUSH_PULL_1_MHZ_DC_40,
                 open_drain_rate=p.I3cOpenDrainRate.OPEN_DRAIN_100_KHZ,
                 i2c_open_drain_rate=p.I2cOpenDrainRate.STANDARD_MODE)
    i3c.ccc.rstdaa()   # start from no dynamic addresses, as after power-up
    step(f"1. SMBus mode, bus at {args.smbus_mv} mV")

    # 2. 0x7E discovery
    if not ack_7e(i3c):
        step("2. DISEC to 7Eh: NACK, no I3C device. Stay in SMBus mode.")
        return 1
    step("2. DISEC to 7Eh: ACK, an I3C device is present")

    # 3. SMBus-only check
    found = smbus_scan(i3c, 0x08, 0x77)
    smbus_only = [a for a in found if a not in args.i3c_static]
    step(f"3. SMBus probe 08h-77h: {' '.join(f'{a:02X}h' for a in found) or 'none'} "
         f"-> SMBus-only: {' '.join(f'{a:02X}h' for a in smbus_only) or 'none'}")
    if smbus_only:
        step("   An SMBus-only device remains. Stay in SMBus mode.")
        return 1

    # 4. 3.3 V -> 1.8 V
    i3c.set_voltage(voltage_mv=args.i3c_mv)
    if tgt_dev:
        tgt_dev.i3c().set_voltage(voltage_mv=args.i3c_mv)
    time.sleep(args.settle_ms / 1000)
    step(f"4. Bus and pull-ups at {args.i3c_mv} mV")

    # 5. DAA
    if args.daa == "setdasa":
        assigned = []
        for n, s in enumerate(args.i3c_static):
            i3c.ccc.setdasa(static=s, dynamic=args.first_dynamic + n)
            assigned.append(args.first_dynamic + n)
    else:
        assigned = [t.dynamic_address for t in i3c.init_bus()]
    ok = True
    for da in assigned:
        try:
            pid = i3c.ccc.getpid(da)
            step(f"5. {args.daa.upper()}: dynamic {da:02X}h, GETPID {pid.hex().upper()}")
        except p.I3cError as e:
            ok = False
            step(f"5. {args.daa.upper()}: dynamic {da:02X}h, GETPID {e.status_code.name}")
    if not assigned:
        step(f"5. {args.daa.upper()}: no device took a dynamic address")
    return 0 if ok and assigned else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", help="controller Supernova serial (prefix)")
    ap.add_argument("--i3c-static", type=number, nargs="+", required=True,
                    help="static addresses of the I3C devices; any other SMBus responder counts as SMBus-only")
    ap.add_argument("--daa", choices=["setdasa", "entdaa"], default="setdasa")
    ap.add_argument("--first-dynamic", type=number, default=0x08, help="first dynamic address for SETDASA")
    ap.add_argument("--smbus-mv", type=int, default=3300)
    ap.add_argument("--i3c-mv", type=int, default=1800)
    ap.add_argument("--settle-ms", type=float, default=10.0, help="wait after the voltage change")
    ap.add_argument("--emulate-target", metavar="SERIAL", help="second Supernova that plays the I3C device")
    ap.add_argument("--rounds", type=int, default=1)
    args = ap.parse_args()

    ctl = open_supernova(args.serial)
    tgt = open_supernova(args.emulate_target) if args.emulate_target else None
    passed = 0
    try:
        print(ctl.fetch_device_info().fw_version, "controller", "| target", tgt.fetch_device_info().fw_version if tgt else "DUT")
        for _ in range(args.rounds):
            try:
                passed += run(args, ctl, tgt) == 0
            except p.CosmicError as e:
                print("error:", e)
    finally:
        ctl.close()
        if tgt:
            tgt.close()
    if args.rounds > 1:
        print(f"{passed}/{args.rounds} rounds passed")
    return 0 if passed == args.rounds else 1


if __name__ == "__main__":
    sys.exit(main())
