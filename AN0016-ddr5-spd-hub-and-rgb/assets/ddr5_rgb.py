"""Drive the Corsair RGB controller behind a DDR5 SPD5118 hub (AN0016).

The controller answers at 0x18 on the DIMM sideband, through the hub, with the
hub in I2C mode. Register map re-implemented from the description in OpenRGB's
Corsair DRAM controller; no OpenRGB code is used.

  info              print VID, PID and protocol from the device-info block
  leave-bootloader  if PID is 0x0B00 (firmware-update mode), write 0x23 <- 0x01
  color R,G,B [...] set one colour on all 6 LEDs, or 6 colours, one per LED
  effect MODE C1 [C2]  hardware effect: static, rainbow, wave, pulse
  demo              colours, per-LED rainbow, chase, then the rainbow wave

Requires pycosmicsdk. See AN0016 for wiring and limits.
"""
import argparse
import sys
import time

from pycosmicsdk import (CosmicError, Device, I2cOpenDrainRate,
                         I3cOpenDrainRate, I3cPushPullRate)

RGB = 0x18
LEDS = 6
PID_BOOTLOADER = 0x0B00
MODES = {"static": 0x10, "rainbow": 0x08, "wave": 0x03, "pulse": 0x01}
RAINBOW = [(255, 0, 0), (255, 80, 0), (255, 220, 0), (0, 255, 0), (0, 0, 255), (160, 0, 255)]


def crc8(data):
    # CRC-8, polynomial 0x07, initial value 0.
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def retry(fn, tries=5, delay=0.02):
    # The controller may NACK the first access after it has been idle.
    for k in range(tries):
        try:
            return fn()
        except CosmicError:
            if k == tries - 1:
                raise
            time.sleep(delay)


def write(i3c, reg, val):
    retry(lambda: i3c.legacy_i2c_write(RGB, bytes([reg, val])))


def read(i3c, reg):
    # Register pointer + read. A bare read is not answered.
    return retry(lambda: i3c.legacy_i2c_read(RGB, 1, subaddress=bytes([reg]))[0])


def info(i3c):
    write(i3c, 0x61, 0x00)
    write(i3c, 0x21, 0x00)
    buf = bytes(read(i3c, 0x40) for _ in range(32))
    if read(i3c, 0x42) != crc8(buf):
        sys.exit("device-info CRC mismatch")
    return {"vid": buf[1] << 8 | buf[0], "pid": buf[3] << 8 | buf[2], "protocol": buf[28]}


def send(i3c, packet, apply_code):
    """Load a packet byte by byte into 0x20, check its CRC, apply it."""
    write(i3c, 0x0B, 0x00)
    write(i3c, 0x21, 0x00)
    for b in packet:
        write(i3c, 0x20, b)
    dev, calc = read(i3c, 0x42), crc8(packet)
    if dev != calc:
        raise RuntimeError(f"CRC mismatch: device 0x{dev:02X}, host 0x{calc:02X}; not applied")
    write(i3c, 0x82, apply_code)
    for _ in range(5):
        if not read(i3c, 0x30) & 0x08:
            return
        time.sleep(0.01)


def colors(i3c, rgb_list):
    """Colour buffer: 6 x (R, G, B, 0xFF), applied with 0x82 <- 0x02."""
    send(i3c, bytes(b for c in rgb_list for b in (*c, 0xFF)), 0x02)


def effect(i3c, mode, c1, c2, speed=0x01, direction=0x00, bright=0xFF):
    """20-byte effect packet, applied with 0x82 <- 0x01."""
    packet = bytes([mode, speed, 0x01, direction, *c1, bright, *c2, bright] + [0] * 8)
    send(i3c, packet, 0x01)


def leave_bootloader(i3c):
    d = info(i3c)
    print(f"before: PID 0x{d['pid']:04X} protocol {d['protocol']}")
    if d["pid"] != PID_BOOTLOADER:
        print("not in firmware-update mode; nothing written")
        return
    write(i3c, 0x23, 0x01)  # Never write 0x00 to 0x23; see AN0016.
    time.sleep(0.5)
    d = info(i3c)
    print(f"after:  PID 0x{d['pid']:04X} protocol {d['protocol']}")


def demo(i3c):
    for c in ((255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 255)):
        colors(i3c, [c] * LEDS)
        time.sleep(2)
    colors(i3c, RAINBOW)
    time.sleep(3)
    for k in range(18):
        colors(i3c, RAINBOW[k % LEDS:] + RAINBOW[:k % LEDS])
        time.sleep(0.25)
    effect(i3c, MODES["wave"], (255, 0, 0), (0, 0, 255), speed=0x02)


def rgb(text):
    c = tuple(int(x) for x in text.split(","))
    if len(c) != 3 or not all(0 <= v <= 255 for v in c):
        raise argparse.ArgumentTypeError("expected R,G,B with values 0-255")
    return c


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", help="Supernova serial number (default: first found)")
    ap.add_argument("--voltage-mv", type=int, default=1100,
                    help="bus voltage; 800-1199 selects the LV connector (default 1100)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("info")
    sub.add_parser("leave-bootloader")
    p = sub.add_parser("color")
    p.add_argument("rgb", type=rgb, nargs="+", help="R,G,B once, or 6 times (one per LED)")
    p = sub.add_parser("effect")
    p.add_argument("mode", choices=MODES)
    p.add_argument("c1", type=rgb)
    p.add_argument("c2", type=rgb, nargs="?")
    p.add_argument("--speed", type=int, default=1, choices=range(0, 3))
    sub.add_parser("demo")
    args = ap.parse_args()

    with Device.open(serial=args.serial) as dev:
        i3c = dev.i3c()
        i3c.set_voltage(args.voltage_mv)
        i3c.bring_up(push_pull_rate=I3cPushPullRate.PUSH_PULL_1_MHZ_DC_40,
                     open_drain_rate=I3cOpenDrainRate.OPEN_DRAIN_100_KHZ,
                     i2c_open_drain_rate=I2cOpenDrainRate.STANDARD_MODE)
        if args.cmd == "info":
            d = info(i3c)
            print(f"VID 0x{d['vid']:04X} PID 0x{d['pid']:04X} protocol {d['protocol']}")
        elif args.cmd == "leave-bootloader":
            leave_bootloader(i3c)
        elif args.cmd == "color":
            if len(args.rgb) not in (1, LEDS):
                sys.exit(f"give 1 or {LEDS} colours")
            colors(i3c, args.rgb * LEDS if len(args.rgb) == 1 else args.rgb)
        elif args.cmd == "effect":
            effect(i3c, MODES[args.mode], args.c1, args.c2 or args.c1, speed=args.speed)
        else:
            demo(i3c)


if __name__ == "__main__":
    main()
