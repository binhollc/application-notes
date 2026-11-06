#!/usr/bin/env python3
"""Check a CMIS I3C target from a Binho Supernova with pycosmicsdk: reads, writes, CCCs, DAA.

Every command initialises the bus (RSTDAA, ENTDAA), finds the target whose
Provisioned ID ends in "CMIS", and reports a count such as "reads 60/60".

    python cmis_target_check.py reads  --length 128 --count 60 [--split] [--ibi]
    python cmis_target_check.py mixed  --count 60
    python cmis_target_check.py ccc    [--getstatus 100]
    python cmis_target_check.py daa    --cycles 10
    python cmis_target_check.py i2c

reads   write the byte address, then read --length bytes, at addresses that cover
        Lower Memory, Upper Memory and the B.1.2 wrap. Combined (write, Sr, read)
        unless --split (write, STOP, read). --ibi sends ENEC first, else DISEC.
mixed   per round: write 7Fh=00 and 7Eh=00 (data writes), then combined reads of
        7Fh (1 byte) and 80h (16 bytes), checked against the image.
ccc     GETBCR, GETDCR, GETPID, GETMXDS, GETMRL, GETMWL, then GETSTATUS N times.
daa     RSTDAA, check the old address NACKs, ENTDAA, check a read; N cycles.
i2c     before any DAA: read 00h:0-3 and 00h:46-48 at static address 0x50 over I2C.

The expected bytes come from the CMIS 5.3 Appendix C image in cmis_image.h, the
same image the target firmware is built from (--image points at it).
"""
import argparse, collections, os, re, sys, time

from pycosmicsdk import (CosmicError, Device, DeviceModel, I2cOpenDrainRate, I3cEvent, I3cOpenDrainRate,
                         I3cPushPullRate)

ADDRS = [0x00, 0x10, 0x7C, 0x80, 0xA0, 0xFC]
def load_image(path):
    text = open(path).read()
    def arr(name):
        body = re.search(name + r"\[\d+\]\s*=\s*\{(.*?)\};", text, re.S).group(1)
        return bytes(int(x, 16) for x in re.findall(r"0x([0-9a-fA-F]{2})", body))
    img = arr("cmis_lower") + arr("cmis_page00")
    return img[:128] * 2 + img[128:] * 2      # B.1.2: the address wraps inside its segment


def expect(img, addr, n):
    base = 0 if addr < 0x80 else 256
    return img[base + (addr & 0x7F): base + (addr & 0x7F) + n]


def same(got, want, addr):
    """Compare, skipping 00h:4 (Flags Summary, latched by the IBI) and 00h:46/48 (I3C support
    and mode status, which the target fills in over the Appendix C image)."""
    if len(got) != len(want):
        return False
    for k, (g, w) in enumerate(zip(got, want)):
        a = addr & 0x80 | (addr + k) & 0x7F
        if g != w and a not in (4, 46, 48):
            return False
    return True


def status(e):
    """Short result name of a CosmicError, e.g. I3C_NACK_ADDRESS."""
    code = getattr(e, "status_code", None)
    return getattr(code, "name", None) or (f"0x{code:04X}" if isinstance(code, int) else str(e))


class Bus:
    def __init__(self, args):
        self.dev = Device.open(serial=args.serial, model=None if args.serial else DeviceModel.SUPERNOVA)
        self.i3c = self.dev.i3c()
        self.ccc = self.i3c.ccc
        self.i3c.set_voltage(args.voltage_mv)
        self.i3c.bring_up(push_pull_rate=getattr(I3cPushPullRate, args.push_pull),
                          open_drain_rate=I3cOpenDrainRate.OPEN_DRAIN_1_MHZ,
                          i2c_open_drain_rate=I2cOpenDrainRate.FAST_MODE)
        self.da = None

    def init_bus(self):
        for _ in range(4):        # the first RSTDAA after a target reset is NACKed
            try:
                table = self.i3c.init_bus()
            except CosmicError as e:
                print("init bus:", status(e))
                continue
            for t in table:
                if bytes(t.pid)[2:] == b"CMIS":
                    self.da = t.dynamic_address
                    print(f"target DA 0x{self.da:02X}, PID {bytes(t.pid).hex(' ')}")
                    return
        sys.exit("no CMIS target on the bus")

    def result(self, fn, *args):
        """(result name, value) of a CCC call, without raising."""
        try:
            return "SUCCESS", fn(*args)
        except CosmicError as e:
            return status(e), None

    def read(self, addr, n, split=False):
        if split:
            self.i3c.sdr_write(self.da, bytes([addr]))
            return bytes(self.i3c.sdr_read(self.da, n))
        return bytes(self.i3c.sdr_read(self.da, n, subaddress=bytes([addr])))

    def write(self, addr, data):
        self.i3c.sdr_write(self.da, bytes(data), subaddress=bytes([addr]))


def cmd_reads(bus, img, args):
    bus.init_bus()
    if args.ibi:
        bus.ccc.enec_broadcast(I3cEvent.IBI)
    else:
        bus.ccc.disec_broadcast(I3cEvent.IBI | I3cEvent.CR | I3cEvent.HJ)
    ok, marks = 0, ""
    for i in range(args.count):
        addr = ADDRS[i % len(ADDRS)]
        try:
            good = same(bus.read(addr, args.length, args.split), expect(img, addr, args.length), addr)
            marks += "." if good else "B"
        except CosmicError:
            good, marks = False, marks + "F"
        ok += good
        time.sleep(args.gap)
    print(marks)
    print(f"reads {ok}/{args.count}  ({'split' if args.split else 'combined'}, "
          f"{args.length} B, IBI {'on' if args.ibi else 'off'})")


def cmd_mixed(bus, img, args):
    bus.init_bus()
    bus.ccc.disec_broadcast(I3cEvent.IBI | I3cEvent.CR | I3cEvent.HJ)
    ok = 0
    for i in range(args.count):
        try:
            bus.write(0x7F, [0x00])
            bus.write(0x7E, [0x00])
            page = bus.read(0x7F, 1)
            upper = bus.read(0x80, 16)
            good = page == b"\x00" and same(upper, expect(img, 0x80, 16), 0x80)
            if not good:
                print(i, "BAD", page.hex(), upper.hex(" "))
        except CosmicError as e:
            good = False
            print(i, status(e))
        ok += good
    print(f"rounds {ok}/{args.count}")


def fmt(v):
    if v is None:
        return ""
    if isinstance(v, int):
        return f"{v:02x}"
    if isinstance(v, (bytes, bytearray)):
        return bytes(v).hex(" ")
    return str(v)


def cmd_ccc(bus, img, args):
    bus.init_bus()
    c = bus.ccc
    for name, fn in (("GETBCR", c.getbcr), ("GETDCR", c.getdcr), ("GETPID", c.getpid),
                     ("GETMXDS", c.getmxds), ("GETMRL", c.getmrl), ("GETMWL", c.getmwl)):
        res, value = bus.result(fn, bus.da)
        print(f"{name:8} {res:22} {fmt(value)}")
    count = collections.Counter(bus.result(c.getstatus, bus.da)[0] for _ in range(args.getstatus))
    print(f"GETSTATUS {count.get('SUCCESS', 0)}/{args.getstatus}  {dict(count)}")
    print("00h:46-48", bus.read(46, 3).hex(" "))
    bus.write(46, [0x00, 0x80, 0x00])
    print("00h:46-48 after writing 00 80 00:", bus.read(46, 3).hex(" "))
    bus.write(47, [0x00])


def cmd_daa(bus, img, args):
    bus.init_bus()
    c, ok = bus.ccc, 0
    for i in range(args.cycles):
        rst = bus.result(c.rstdaa)[0]
        old = bus.result(c.getstatus, bus.da)[0]
        bus.result(c.entdaa)    # the adapter may report the closing 7Eh NACK; check with a read
        try:
            back = bus.read(0x00, 4) == expect(img, 0x00, 4)
        except CosmicError:
            back = False
        good = rst == "SUCCESS" and old != "SUCCESS" and back
        ok += good
        print(f"{i}: RSTDAA {rst}, GETSTATUS at old DA {old}, read after ENTDAA {'ok' if back else 'FAIL'}")
    print(f"cycles {ok}/{args.cycles}")


def cmd_i2c(bus, img, args):
    for addr, n in ((0, 4), (46, 3)):
        try:
            data = bus.i3c.legacy_i2c_read(0x50, n, subaddress=bytes([addr]))
            print(f"I2C 0x50 00h:{addr}: {bytes(data).hex(' ')}")
        except CosmicError as e:
            print(f"I2C 0x50 00h:{addr}: {status(e)}")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--serial", default=os.environ.get("PYCOSMICSDK_SUPERNOVA_SERIAL"))
    p.add_argument("--push-pull", default="PUSH_PULL_2_5_MHZ_DC_25")
    p.add_argument("--voltage-mv", type=int, default=3300)
    p.add_argument("--image", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "cmis_image.h"))
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("reads")
    r.add_argument("--length", type=int, default=16)
    r.add_argument("--count", type=int, default=60)
    r.add_argument("--split", action="store_true")
    r.add_argument("--ibi", action="store_true")
    r.add_argument("--gap", type=float, default=0.02)
    sub.add_parser("mixed").add_argument("--count", type=int, default=60)
    sub.add_parser("ccc").add_argument("--getstatus", type=int, default=100)
    sub.add_parser("daa").add_argument("--cycles", type=int, default=10)
    sub.add_parser("i2c")
    args = p.parse_args()
    img = load_image(args.image)
    bus = Bus(args)
    try:
        globals()["cmd_" + args.cmd](bus, img, args)
    finally:
        bus.dev.close()


if __name__ == "__main__":
    main()
