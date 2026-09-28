#!/usr/bin/env python3
"""Capture the AN0014 `handoff` flow in Saleae Logic 2 and save it decoded.

Logic 2 must be running with its automation server on (Preferences, or `--automation`).
This script starts a capture, runs `smbus_stretch.py handoff` with the arguments after `--`,
stops, adds Logic 2's I2C analyzer and the flow_labels HLA, then saves the .sal and a CSV.

    python capture_flow.py --scl 1 --sda 0 --out flow -- --khz 400 handoff --address 0x2A \\
        --sda-pull-up ON --t2wrst-us 35000 --step-delay-s 0.5 --pulsar-pull-up any

Requires logic2-automation (pip install logic2-automation).
"""

import argparse
import os
import subprocess
import sys
import time

from saleae import automation

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=10430, help="Logic 2 automation port")
    ap.add_argument("--device", help="Logic device id (default: the first real one)")
    ap.add_argument("--simulate", action="store_true", help="use a simulated device (dry run)")
    ap.add_argument("--scl", type=int, default=1, help="SCL channel (default 1)")
    ap.add_argument("--sda", type=int, default=0, help="SDA channel (default 0)")
    ap.add_argument("--rate", type=int, default=50_000_000, help="digital sample rate (default 50 MS/s)")
    ap.add_argument("--threshold", type=float, default=None, help="Logic Pro only: logic threshold volts")
    ap.add_argument("--lead-s", type=float, default=0.5, help="capture before the flow starts")
    ap.add_argument("--out", default="flow", help="output prefix: <out>.sal and <out>-steps.csv")
    ap.add_argument("flow", nargs=argparse.REMAINDER, help="-- then the smbus_stretch.py arguments")
    args = ap.parse_args()
    flow = args.flow[1:] if args.flow[:1] == ["--"] else args.flow

    with automation.Manager.connect(port=args.port) as m:
        devices = m.get_devices(include_simulation_devices=args.simulate)
        dev = args.device or next((d.device_id for d in devices if d.is_simulation == args.simulate), None)
        if dev is None:
            sys.exit("no Logic device found")
        cfg = automation.LogicDeviceConfiguration(
            enabled_digital_channels=[args.sda, args.scl],
            digital_sample_rate=args.rate,
            digital_threshold_volts=args.threshold)
        with m.start_capture(device_id=dev, device_configuration=cfg,
                             capture_configuration=automation.CaptureConfiguration(
                                 capture_mode=automation.ManualCaptureMode())) as cap:
            time.sleep(args.lead_s)
            rc = 0
            if flow:
                rc = subprocess.call([sys.executable, os.path.join(HERE, "smbus_stretch.py"), *flow])
            time.sleep(args.lead_s)
            cap.stop()
            i2c = cap.add_analyzer("I2C", label="SMBus", settings={"SCL": args.scl, "SDA": args.sda})
            hla = cap.add_high_level_analyzer(os.path.join(HERE, "flow_labels"),
                                              "SMBus to I3C Basic flow", input_analyzer=i2c, label="Flow")
            out = os.path.abspath(args.out)
            cap.save_capture(out + ".sal")
            cap.export_data_table(out + "-steps.csv", analyzers=[i2c, hla])
            print(f"saved {out}.sal and {out}-steps.csv (flow exit {rc})")
            return rc


if __name__ == "__main__":
    sys.exit(main())
