#!/usr/bin/env python3
"""capture_vd55g1.py — I3C_CONTROLLER_CAPTURE_LINES against an STMicro VD55G1 (Gilisymo LS-CAM-I3C).

The sensor reads out over I3C one line at a time and drops a line it is not read within ~340 us,
so the reads have to happen on the adapter. This script boots the sensor with pycosmicsdk, closes
the SDK (the I3C and GPIO configuration survive in fw), then sends the capture command over raw HID
and collects the frame(s) from vendor bulk IN. One run = power-up + boot + one capture command, so
`--runs N` gives a pass ratio.

Wiring (Supernova rev C): LS-CAM-I3C on the I3C HV port at 3300 mV, VIN from the HV rail, NRST on
GPIO PIN_1, sensor GPIO0 (line ready) on GPIO PIN_3, GPIO1 on PIN_5. Boots with GPIO0 wired (8/8);
a logic-analyzer probe on GPIO0 does block the boot. Loose jumpers drop the sensor (address NACK).

The patch arrays are ST's and are not in this repo: pass the Gilisymo driver,
github.com/gilisymo/vd55g1_stm32h533 Drivers/LS55G1/ls55g1_driver.c.

usage: capture_vd55g1.py SERIAL LS55G1_DRIVER_C [--width 160 --height 120 --frames 1
                         --trigger gpio-timed|gpio|trace --runs 5 --out frames/]
Requires: pip install pycosmicsdk hidapi pyusb pillow
"""
import argparse
import os
import re
import struct
import sys
import time

import pycosmicsdk as S
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from raw_link import I3C_CONTROLLER_CAPTURE_LINES, RawLink  # noqa: E402

NRST, LINE_RDY = S.GpioPinNumber.PIN_1, S.GpioPinNumber.PIN_3
LINE_RDY_FW = 2                       # GpioPins_t: GPIO_1 = 0, so PIN_3 = 2
FSM, CMD_BOOT, CMD_STBY = 0x001C, 0x0200, 0x0201
TRIGGERS = {"gpio": 0, "gpio-falling": 1, "gpio-timed": 2, "trace": 3}
STAGING = 32768                       # bulk staging pool on the Supernova
REQ = "<BB4sHHBBHB4sB8sHI"
RESP = "<HIHHHHI"                     # lines, bytes, driverError, min/max period, maxRead, total


def c_arrays(path):
    """name -> bytes for every `const uint8_t name[...] = {...};` in the C file."""
    src = open(path).read()
    return {m.group(1): bytes(int(v, 16) for v in re.findall(r"0x[0-9a-fA-F]+", m.group(2)))
            for m in re.finditer(r"const uint8_t (\w+)\[[^=]*=\s*\{(.*?)\};", src, re.S)}


class Cam:
    """Bring-up and configuration, ported from ls55g1_init_cut20 + ls55g1_configure (cut 2.0)."""

    def __init__(self, dev):
        self.i3c, self.gpio = dev.i3c(), dev.gpio()

    def rd(self, reg, n=1):
        return self.i3c.sdr_read(self.da, n, subaddress=reg.to_bytes(2, "big"))

    def wr(self, reg, *data):
        self.i3c.sdr_write(self.da, bytes(data), subaddress=reg.to_bytes(2, "big"))

    def wr_block(self, reg, blob, step=30):
        for off in range(0, len(blob), step):
            self.i3c.sdr_write(self.da, blob[off:off + step], subaddress=(reg + off).to_bytes(2, "big"))

    def poll(self, reg, want, mask=0xFF, tries=50):
        for _ in range(tries):
            try:
                if self.rd(reg)[0] & mask == want:
                    return True
            except S.I3cError:            # NACKs while it boots
                pass
            time.sleep(0.01)
        return False

    def power_up(self):
        self.i3c.set_voltage(3300)
        self.gpio.set_voltage(3300)
        self.gpio.configure_pin(LINE_RDY, mode=S.GpioFunctionality.DIGITAL_INPUT)
        self.gpio.configure_pin(NRST, mode=S.GpioFunctionality.DIGITAL_OUTPUT, initial=S.GpioLogicLevel.LOW)
        time.sleep(0.1)
        self.gpio.write(NRST, S.GpioLogicLevel.HIGH)
        time.sleep(0.1)
        self.i3c.bring_up(push_pull_rate=S.I3cPushPullRate.PUSH_PULL_5_MHZ_DC_50,
                          open_drain_rate=S.I3cOpenDrainRate.OPEN_DRAIN_1_MHZ,
                          i2c_open_drain_rate=S.I2cOpenDrainRate.FAST_MODE)
        self.da = self.i3c.init_bus()[0].dynamic_address
        model = self.rd(0x0000, 4)[::-1].decode("ascii", "replace")
        assert model == "S5G1" and self.rd(0x0004)[0] == 0x20, f"not a VD55G1 cut 2.0 ({model})"

    def boot(self, patches):
        assert self.poll(FSM, 0x1), "not READY_TO_BOOT"
        self.wr_block(0x2000, patches["patchData1a"] + patches["patchData1b"])
        self.wr(CMD_BOOT, 0x2)                     # PATCH_AND_BOOT
        time.sleep(0.2)
        assert self.poll(CMD_BOOT, 0x0, 0x3), "PATCH_AND_BOOT did not complete"
        time.sleep(0.2)
        assert self.poll(FSM, 0x2), "not SW_STBY after boot (is GPIO0 wired? see header)"
        self.wr(CMD_STBY, 0x6)                     # START_VTRAM_UPDATE
        time.sleep(0.2)
        self.poll(CMD_STBY, 0x0, 0xF)
        self.wr(0xC638, 0x21, 0x0)
        self.wr_block(0xC3C0, patches["patchData2a"] + patches["patchData2b"])
        self.wr_block(0xC640, patches["patchData2c"] + patches["patchData2d"])
        self.wr(CMD_STBY, 0x7)                     # END_VTRAM_UPDATE
        time.sleep(0.2)
        self.poll(CMD_STBY, 0x0, 0xF)
        self.wr(0x0220, *(12_000_000).to_bytes(4, "little"))   # EXT_CLOCK, as ST's demo
        self.wr(0x0224, 0x0, 0x8C, 0x86, 0x47)     # MIPI data rate
        self.wr(0x0300, 0x68, 0x04)                # LINE_LENGTH
        self.wr(0x0500, 0x0)
        self.wr(0x0300, 0xC8, 0xAF)
        self.wr(0x050C, 0x82, 0x06)                # CTX0 FRAME_LENGTH
        for reg in (0x055C, 0x05AC, 0x05AF):       # CTX1-3 FRAME_LENGTH (0x05AF as ST writes it)
            self.wr(reg, 0x82, 0x06, 0x0, 0x0)
        self.wr(0x052E, 0x1)
        self.wr(0x051D, 0x0C)                      # GPIO0 = line ready
        self.wr(0x0500, 0x0)

    def configure(self, w, h, frame_length=700, exposure=500, line_length=0xC738, ready_gpio=0, readout_ctrl=0x03,
                  again=None, dgain=1.1):
        assert self.poll(FSM, 0x2, tries=200), "not SW_STBY"
        self.wr(0x050C, frame_length & 0xFF, frame_length >> 8, 0, 0)
        self.wr(0x0300, line_length & 0xFF, line_length >> 8)
        self.wr(0x030A, 0x08, 0x00)                # FORMAT_CTRL: RAW8
        self.wr(0x0324, readout_ctrl, 0x00)        # I3C_FRAME_READOUT_CTRL: ST writes 0x03
        self.wr(0x0326, 0x00)
        self.wr(0x032A, 0x42)
        self.wr(0x0410, 0x11, 0x11)
        self.wr(0x0488, 0xF0, 0x00)
        self.wr(0x0500, 0x0 if again is None else 0x2)  # auto exposure, or manual (ST: 0x02)
        if again is not None:
            self.wr(0x0501, again)                 # analog gain code, x32/(32-code): 28 = x8
        self.wr(0x0510, 174 & 0xFF, 174 >> 8)      # Y_START
        self.wr(0x0514, 200 & 0xFF, 200 >> 8)      # X_START
        self.wr(0x0512, h & 0xFF, h >> 8)          # Y_HEIGHT
        self.wr(0x0516, w & 0xFF, w >> 8)          # X_WIDTH
        for io in range(4):                        # GPIO_n_CTRL = 0x051D + n; 0x0C = line ready
            self.wr(0x051D + io, 0x0C if io == ready_gpio else 0x00)
        self.wr(0x052E, 0x0)
        self.wr(0x0537, 0x0)
        self.wr(0x03E4, 0x11, 0x11)
        self.wr(0x0504, int((dgain % 1) * 256), int(dgain))  # digital gain, 8.8 fixed point
        self.wr(0x050C, frame_length & 0xFF, frame_length >> 8, 0, 0)
        self.wr(0x0502, exposure & 0xFF, exposure >> 8)

    def readout_rate(self, rate="PUSH_PULL_10_MHZ_DC_40"):
        # Line reads must fit the ~340 us line period: 10 MHz, as ST's 9.5. Only after boot -- right
        # after NRST the sensor NACKs at 10 MHz -- and 12.5 MHz NACKed 20/20 on the jumper wiring.
        self.i3c.configure(push_pull_rate=getattr(S.I3cPushPullRate, rate),
                           open_drain_rate=S.I3cOpenDrainRate.OPEN_DRAIN_1_MHZ,
                           i2c_open_drain_rate=S.I2cOpenDrainRate.FAST_MODE)
        assert self.rd(FSM)[0] == 0x2, "lost the sensor at the readout rate"


def capture(link, da, w, lines, trigger, timeout_ms=4000, read_delay_us=0, period_ns=0):
    """Send I3C_CONTROLLER_CAPTURE_LINES, START_STREAMING folded in; return (result, stats, payload)."""
    params = struct.pack(REQ, da, 2, (0x2000).to_bytes(2, "big").ljust(4, b"\0"), w, lines,
                         TRIGGERS[trigger], LINE_RDY_FW, timeout_ms,
                         2, CMD_STBY.to_bytes(2, "big").ljust(4, b"\0"), 1, b"\x01".ljust(8, b"\0"), read_delay_us, period_ns)
    result, rest = link.request(I3C_CONTROLLER_CAPTURE_LINES, params, timeout_ms=timeout_ms + 3000)
    stats = dict(zip(("lines", "bytes", "driverError", "minPeriodUs", "maxPeriodUs",
                      "maxReadUs", "totalUs"), struct.unpack(RESP, rest[:struct.calcsize(RESP)])))
    return result, stats, link.collect(stats["bytes"]) if stats["bytes"] else b""


def boot_sensor(args, patches):
    """Power up, patch, boot and configure the sensor; return its dynamic address."""
    dev = S.Device.open(serial=args.serial)
    try:
        cam = Cam(dev)
        cam.power_up()
        cam.boot(patches)
        cam.configure(args.width, args.height, ready_gpio=args.line_ready_gpio, readout_ctrl=args.readout_ctrl,
                      exposure=args.exposure, again=args.again, dgain=args.dgain,
                      frame_length=args.frame_length)
        cam.readout_rate(args.readout_rate)
        return cam.da
    finally:
        dev.close()


def trace_once(args, da):
    """Diagnostic: start streaming, log every PIN_3 transition for --trace-ms, no reads."""
    link = RawLink(args.serial)
    try:
        result, stats, data = capture(link, da, 1, 1, "trace", timeout_ms=args.trace_ms)
        _r, fsm = link.i3c_read(da, FSM, 1)
    finally:
        link.close()
    entries = [struct.unpack_from("<I", data, i)[0] for i in range(0, len(data) - 3, 4)]
    events = [(e & 0x7FFFFFFF, e >> 31) for e in entries]
    rises = [t for t, lvl in events[1:] if lvl]
    highs = [t1 - t0 for (t0, l0), (t1, _l1) in zip(events, events[1:]) if l0]
    print(f"trace: result=0x{result:04X} fsm after={fsm.hex()} level@0={events[0][1] if events else '?'} "
          f"transitions={len(events) - 1} rises={len(rises)}")
    print("first 12 events (us, level):", events[1:13])
    if len(rises) > 1:
        gaps = [b - a for a, b in zip(rises, rises[1:])]
        print(f"rise-to-rise us: min {min(gaps)} max {max(gaps)} | high width us: min {min(highs)} max {max(highs)}"
              f" | last event at {events[-1][0]} us")
    return result == 0


def capture_once(args, da, run):
    link = RawLink(args.serial)
    try:
        lines = args.lines or args.height * args.frames
        t0 = time.monotonic()
        result, stats, data = capture(link, da, args.width, lines, args.trigger, read_delay_us=args.post_delay_us,
                                       period_ns=args.line_period_ns)
        wall = time.monotonic() - t0
        _r, fsm = link.i3c_read(da, FSM, 1)
    finally:
        link.close()
    ok = result == 0 and len(data) == args.width * lines
    print(f"run {run}: result=0x{result:04X} got {len(data)}/{args.width * lines} B in {wall * 1000:.0f} ms, "
          f"fsm after={fsm.hex()} {stats}")
    for f in range(len(data) // (args.width * args.height)):
        frame = data[f * args.width * args.height:(f + 1) * args.width * args.height]
        Image.frombytes("L", (args.width, args.height), frame).save(
            os.path.join(args.out, f"run{run:02d}_f{f}.png"))
    return ok


def one_run(args, patches, run):
    return capture_once(args, boot_sensor(args, patches), run)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("serial")
    ap.add_argument("driver_c")
    ap.add_argument("--width", type=int, default=160)
    ap.add_argument("--height", type=int, default=120)
    ap.add_argument("--frames", type=int, default=1, help="consecutive frames in one capture")
    ap.add_argument("--trigger", choices=TRIGGERS, default="gpio-timed")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--out", default="frames")
    ap.add_argument("--phase", choices=("all", "boot", "capture"), default="all",
                    help="boot / capture: one half of a run, for driving the GPIO0 wiring step by hand")
    ap.add_argument("--run", type=int, default=0, help="run number for --phase capture output names")
    ap.add_argument("--da", type=lambda v: int(v, 0), default=0x08, help="sensor address for --phase capture")
    ap.add_argument("--line-ready-gpio", type=int, choices=range(4), default=0,
                    help="sensor GPIO that carries line ready (GPIO0 is a boot strap: wired, it blocks boot)")
    ap.add_argument("--readout-ctrl", type=lambda v: int(v, 0), default=0x03, help="I3C_FRAME_READOUT_CTRL (0x0324)")
    ap.add_argument("--readout-rate", default="PUSH_PULL_10_MHZ_DC_40", help="I3cPushPullRate name for line reads")
    ap.add_argument("--lines", type=int, default=0, help="capture this many lines instead of height x frames")
    ap.add_argument("--post-delay-us", type=int, default=300,
                    help="wait this long after each line's edge/due time before reading (max 1000):"
                         " the sensor raises line ready before the line is written, and an early read resets it")
    ap.add_argument("--again", type=int, choices=range(29), help="manual exposure, analog gain code (28 = x8)")
    ap.add_argument("--dgain", type=float, default=1.1, help="digital gain, 1.0-32.0")
    ap.add_argument("--exposure", type=int, default=500, help="exposure lines, <= frame length 700 (manual mode)")
    ap.add_argument("--frame-length", type=int, default=700, help="sensor frame length in lines")
    ap.add_argument("--line-period-ns", type=int, default=340500,
                    help="--trigger gpio-timed: line N is read at line 0's edge + N x this (clean 5/5 from 337.7 to 345 us at 300 us delay)")
    ap.add_argument("--trace-ms", type=int, default=300, help="--trigger trace: how long to log PIN_3")
    args = ap.parse_args()
    if not 0 <= args.post_delay_us <= 1000:
        sys.exit("--post-delay-us must be 0-1000")
    if args.width * (args.lines or args.height * args.frames) > STAGING:
        sys.exit(f"{args.width}x{args.height}x{args.frames} does not fit the {STAGING} B staging pool")
    patches = c_arrays(args.driver_c)
    if args.phase == "boot":
        print(f"booted, DA=0x{boot_sensor(args, patches):02X}")
        return 0
    os.makedirs(args.out, exist_ok=True)
    if args.phase == "capture":
        once = trace_once(args, args.da) if args.trigger == "trace" else capture_once(args, args.da, args.run)
        return 0 if once else 1
    passed = sum(one_run(args, patches, r) for r in range(args.runs))
    print(f"{passed}/{args.runs} runs captured every line")
    return 0 if passed == args.runs else 1


if __name__ == "__main__":
    sys.exit(main())
