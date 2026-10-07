"""Read a DDR5 SPD5118 hub from the Binho Supernova I3C LV connector (AN0016).

Default: legacy I2C mode, reads only. Prints the device type (MR0-1), revision,
I3C-mode flag (MR18) and temperature (MR49-50).

  --nvm FILE   also read the 1024-byte SPD NVM into FILE. Selects each 128-byte
               page through MR11 (the only register this script writes) and
               restores MR11 afterwards. Checks the SPD CRC over bytes 0-509.
  --i3c        also switch the hub to I3C with SETAASA, read MR0-1, MR18 and the
               temperature in SDR, then return it to I2C with RSTDAA.

Requires pycosmicsdk. See AN0016 for wiring and limits.
"""
import argparse
import sys
import time

from pycosmicsdk import (CosmicError, Device, I2cOpenDrainRate,
                         I3cOpenDrainRate, I3cPushPullRate)

HUB = 0x50
MR11 = 0x0B
NVM_PAGE_BYTES = 128
NVM_PAGES = 8
CHUNK = 32

PUSH_PULL = {
    "1": I3cPushPullRate.PUSH_PULL_1_MHZ_DC_40,
    "3.125": I3cPushPullRate.PUSH_PULL_3_125_MHZ_DC_25,
    "5": I3cPushPullRate.PUSH_PULL_5_MHZ_DC_50,
}


def retry(fn, tries=3, delay=0.02):
    for k in range(tries):
        try:
            return fn()
        except CosmicError:
            if k == tries - 1:
                raise
            time.sleep(delay)


def mr(i3c, reg, n=1):
    return retry(lambda: i3c.legacy_i2c_read(HUB, n, subaddress=bytes([reg])))


def temperature_c(lsb, msb):
    # MR50[4:0]:MR49[7:0], bits 1:0 unused; 11-bit two's complement, 0.25 C/LSB.
    raw = (((msb & 0x1F) << 8) | lsb) >> 2
    if raw & 0x400:
        raw -= 0x800
    return raw * 0.25


def spd_crc16(data):
    # JEDEC SPD CRC: CRC-16, polynomial 0x1021, initial value 0.
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def bring_up(i3c, rate):
    return i3c.bring_up(push_pull_rate=rate,
                        open_drain_rate=I3cOpenDrainRate.OPEN_DRAIN_100_KHZ,
                        i2c_open_drain_rate=I2cOpenDrainRate.STANDARD_MODE)


def report_i2c(i3c):
    mr01 = mr(i3c, 0x00, 2)
    rev = mr(i3c, 0x02)[0]
    mr18 = mr(i3c, 0x12)[0]
    lsb, msb = mr(i3c, 0x31, 2)
    print(f"MR0-1 {mr01.hex()}  (SPD5118 reads 5118)")
    print(f"MR2 revision 0x{rev:02X}  MR18 0x{mr18:02X}  (bit 5 = I3C mode)")
    print(f"MR49-50 {lsb:02X} {msb:02X} = {temperature_c(lsb, msb):.2f} C")
    if mr01 != b"\x51\x18":
        sys.exit("not an SPD5118 at 0x50")


def dump_nvm(i3c, path):
    saved = mr(i3c, MR11)[0]
    if saved & 0x08:
        sys.exit("MR11 bit 3 set (2-byte addressing); this script uses 1-byte addressing")
    data = bytearray()
    try:
        for page in range(NVM_PAGES):
            value = (saved & ~0x07) | page
            retry(lambda: i3c.legacy_i2c_write(HUB, bytes([MR11, value])))
            for off in range(0, NVM_PAGE_BYTES, CHUNK):
                # Address bit 7 set = NVM, bits 6:0 = offset within the page.
                data += retry(lambda: i3c.legacy_i2c_read(
                    HUB, CHUNK, subaddress=bytes([0x80 | off])))
    finally:
        retry(lambda: i3c.legacy_i2c_write(HUB, bytes([MR11, saved])))
    with open(path, "wb") as f:
        f.write(data)
    calc, stored = spd_crc16(data[:510]), data[510] | (data[511] << 8)
    verdict = "OK" if calc == stored else "MISMATCH"
    print(f"NVM: {len(data)} bytes to {path}, MR11 restored to 0x{saved:02X}")
    print(f"SPD byte 2 0x{data[2]:02X} (0x12 = DDR5), byte 3 0x{data[3]:02X}, "
          f"CRC 0-509 calc 0x{calc:04X} stored 0x{stored:04X} {verdict}")


def report_i3c(i3c, rate):
    bring_up(i3c, rate)
    i3c.ccc.setaasa([HUB])
    try:
        rd = lambda reg, n=1: retry(lambda: i3c.sdr_read(HUB, n, subaddress=bytes([reg])))
        mr18 = rd(0x12)[0]
        mr01 = rd(0x00, 2)
        lsb, msb = rd(0x31, 2)
        print(f"I3C SDR: MR18 0x{mr18:02X}  MR0-1 {mr01.hex()}  "
              f"temperature {temperature_c(lsb, msb):.2f} C")
    finally:
        i3c.ccc.rstdaa()
        bring_up(i3c, PUSH_PULL["1"])
    print(f"after RSTDAA (I2C): MR18 0x{mr(i3c, 0x12)[0]:02X}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--serial", help="Supernova serial number (default: first found)")
    ap.add_argument("--voltage-mv", type=int, default=1100,
                    help="bus voltage; 800-1199 selects the LV connector (default 1100)")
    ap.add_argument("--nvm", metavar="FILE", help="read the 1 KB SPD NVM into FILE")
    ap.add_argument("--i3c", action="store_true", help="also read in I3C SDR mode")
    ap.add_argument("--i3c-mhz", choices=sorted(PUSH_PULL), default="1",
                    help="I3C push-pull rate for --i3c (default 1)")
    args = ap.parse_args()

    with Device.open(serial=args.serial) as dev:
        i3c = dev.i3c()
        i3c.set_voltage(args.voltage_mv)
        bring_up(i3c, PUSH_PULL["1"])
        report_i2c(i3c)
        if args.nvm:
            dump_nvm(i3c, args.nvm)
        if args.i3c:
            report_i3c(i3c, PUSH_PULL[args.i3c_mhz])


if __name__ == "__main__":
    main()
