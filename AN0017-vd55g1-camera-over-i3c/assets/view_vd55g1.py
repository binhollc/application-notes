#!/usr/bin/env python3
"""view_vd55g1.py — live view of the VD55G1 (Gilisymo LS-CAM-I3C) through the Supernova.

Boots the sensor once, then loops: capture the next frame (I3C_CONTROLLER_CAPTURE_LINES; its
START_STREAMING write is ACKed while already streaming) and show it. The sensor's frame length
sets the rate: 300 lines x 340 us = 9.8 fps, 60/60 frames; below ~280 lines (Y_START 174 + 120
rows) every capture times out. Same wiring and driver file as capture_vd55g1.py. Close the
window or Ctrl+C to stop.

usage: view_vd55g1.py SERIAL LS55G1_DRIVER_C [--again 0 --exposure 60 --frame-length 300 --scale 4 --record out.mp4]
"""
import argparse
import collections
import os
import signal
import subprocess
import sys
import time
import tkinter as tk

from PIL import Image, ImageTk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture_vd55g1 import boot_sensor, c_arrays, capture  # noqa: E402
from raw_link import RawLink  # noqa: E402

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("serial")
    ap.add_argument("driver_c")
    ap.add_argument("--width", type=int, default=160)
    ap.add_argument("--height", type=int, default=120)
    ap.add_argument("--exposure", type=int, default=60, help="exposure lines, with --again only")
    ap.add_argument("--again", type=int, help="manual exposure, analog gain code 0-28 (default: auto exposure)")
    ap.add_argument("--dgain", type=float, default=1.1)
    ap.add_argument("--frame-length", type=int, default=300, help="sensor frame length in lines (>= 280)")
    ap.add_argument("--record", help="also write the frames to this video file (needs ffmpeg)")
    ap.add_argument("--scale", type=int, default=4)
    ap.add_argument("--line-period-ns", type=int, default=340500)
    ap.add_argument("--post-delay-us", type=int, default=300)
    args = ap.parse_args()
    args.line_ready_gpio, args.readout_ctrl, args.readout_rate = 0, 0x03, "PUSH_PULL_10_MHZ_DC_40"
    w, h = args.width, args.height

    patches = c_arrays(args.driver_c)
    cam = {"da": boot_sensor(args, patches), "link": RawLink(args.serial), "bad_run": 0}
    root = tk.Tk()
    root.title("VD55G1 via Supernova I3C")
    label = tk.Label(root)
    label.pack()
    status = tk.Label(root, font=("monospace", 10))
    status.pack()
    rec = None
    if args.record:
        # ponytail: fixed 10 fps tag; the sensor runs at ~9.8 with --frame-length 300
        rec = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "gray",
                                "-s", f"{w * args.scale}x{h * args.scale}", "-r", "10", "-i", "-",
                                "-pix_fmt", "yuv420p", args.record], stdin=subprocess.PIPE)
    stats = {"ok": 0, "bad": 0, "reboots": 0}
    shown = collections.deque(maxlen=20)                 # times of the last good frames

    def frame():
        try:
            result, _st, data = capture(cam["link"], cam["da"], w, h, "gpio-timed", timeout_ms=1000,
                                        read_delay_us=args.post_delay_us, period_ns=args.line_period_ns)
        except TimeoutError:
            result, data = 0xFFFF, b""
        if result == 0 and len(data) == w * h:
            cam["bad_run"] = 0
            shown.append(time.monotonic())
            img = Image.frombytes("L", (w, h), data).resize((w * args.scale, h * args.scale), Image.NEAREST)
            if rec:
                rec.stdin.write(img.tobytes())
            label.image = ImageTk.PhotoImage(img)
            label.configure(image=label.image)
            stats["ok"] += 1
        else:
            stats["bad"] += 1
            cam["bad_run"] += 1
            if cam["bad_run"] >= 3:                       # sensor lost (NACKs its address): boot it again
                cam["link"].close()
                cam["da"] = boot_sensor(args, patches)
                cam["link"] = RawLink(args.serial)
                cam["bad_run"] = 0
                stats["reboots"] += 1
        fps = (len(shown) - 1) / (shown[-1] - shown[0]) if len(shown) > 1 else 0.0
        status.configure(text=f"{fps:.1f} fps   frames ok {stats['ok']}  bad {stats['bad']}"
                              f"  reboots {stats['reboots']}   last result 0x{result:04X}")
        root.after(1, frame)

    root.after(1, frame)
    for sig in (signal.SIGINT, signal.SIGTERM):          # Ctrl+C / kill close the video cleanly too
        signal.signal(sig, lambda *_: root.quit())
    t0 = time.monotonic()
    try:
        root.mainloop()
    finally:
        cam["link"].close()
        dt = time.monotonic() - t0
        print(f"{stats['ok']} frames ok, {stats['bad']} bad, {stats['reboots']} reboots in {dt:.1f} s "
              f"({stats['ok'] / dt:.1f} fps)")
        if rec:
            rec.stdin.close()
            rec.wait()


if __name__ == "__main__":
    sys.exit(main())
