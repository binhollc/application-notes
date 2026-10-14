"""raw_link.py: raw HID command channel plus the vendor bulk IN endpoint, no SDK.

Used for commands no released SDK knows yet (I3C_CONTROLLER_CAPTURE_LINES). Wire format, from
usb_transfer_protocol.h and commands_definitions.h:
  HID report (65 B) = 0x00 report id + u16 LE packet header (sot | eot<<1 | len<<2) + payload,
  payload = u16 id + u16 code + parameters; a reply starts with u16 id, u16 code, u16 result.
Requires: pip install hidapi pyusb.
"""
import struct
import time

import hid
import usb.core
import usb.util

VID, SUPERNOVA_PID = 0x1FC9, 0x82FC
VENDOR_INTERFACE = 1
PACKET = 64


def cmd(group, value, role=1, typ=0):
    return (group << 10) | (role << 8) | (typ << 7) | value


GROUP_I3C = 6
I3C_CONTROLLER_PRIVATE_TRANSFER = cmd(GROUP_I3C, 8)
I3C_CONTROLLER_CAPTURE_LINES = cmd(GROUP_I3C, 14)


class RawLink:
    def __init__(self, serial):
        paths = [d["path"] for d in hid.enumerate(VID, SUPERNOVA_PID)
                 if d["serial_number"] == serial and d["interface_number"] == 0]
        if not paths:
            found = sorted({d["serial_number"] for d in hid.enumerate(VID, SUPERNOVA_PID)})
            raise RuntimeError(f"no Supernova with serial {serial}; connected: {found or 'none'}")
        self.h = hid.device()
        self.h.open_path(paths[0])
        self.usb = usb.core.find(idVendor=VID, idProduct=SUPERNOVA_PID,
                                 custom_match=lambda d: d.serial_number == serial)
        intf = self.usb.get_active_configuration()[(VENDOR_INTERFACE, 0)]
        self.ep_in = next(e.bEndpointAddress for e in intf
                          if usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)
        usb.util.claim_interface(self.usb, VENDOR_INTERFACE)
        self._id = 0
        self.discard()

    def close(self):
        usb.util.release_interface(self.usb, VENDOR_INTERFACE)
        usb.util.dispose_resources(self.usb)
        self.h.close()

    def request(self, code, params, timeout_ms=4000):
        """Send one command (single packet) and return (result, reply parameters)."""
        self._id = (self._id + 1) & 0xFFFF
        payload = struct.pack("<HH", self._id, code) + params
        assert len(payload) <= PACKET - 2, "multi-packet requests not needed here"
        hdr = 0x1 | 0x2 | (len(payload) << 2)
        self.h.write(b"\x00" + struct.pack("<H", hdr) + payload.ljust(PACKET - 2, b"\x00"))
        deadline = time.monotonic() + timeout_ms / 1000
        body = b""
        while time.monotonic() < deadline:
            pkt = bytes(self.h.read(PACKET, 50))
            if not pkt:
                continue
            h = struct.unpack("<H", pkt[:2])[0]
            if h & 0x1:
                body = b""
            body += pkt[2:2 + (h >> 2)]
            if not h & 0x2:
                continue
            rid, rcode, result = struct.unpack("<HHH", body[:6])
            if rid == self._id and rcode == code:
                return result, body[6:]
            body = b""                     # a notification or a late reply: not ours
        raise TimeoutError(f"no reply to 0x{code:04X}")

    def collect(self, count, timeout_ms=3000):
        """Read exactly `count` payload bytes from vendor IN (fewer only on timeout)."""
        body, empty = b"", 0
        while len(body) < count and empty < 10:
            try:
                chunk = bytes(self.usb.read(self.ep_in, count - len(body) + PACKET, timeout=timeout_ms))
            except usb.core.USBError:
                empty += 1
                continue
            body += chunk
        return body

    def discard(self):
        try:
            while self.usb.read(self.ep_in, 16384, timeout=30):
                pass
        except usb.core.USBError:
            pass

    def i3c_read(self, addr, reg, n, reg_len=2):
        """SDR private read with a register address (I3C_CONTROLLER_PRIVATE_TRANSFER)."""
        flags = 1                         # direction READ (bit 0), mode I3C_SDR (=0, bits 1-2)
        params = struct.pack("<BBBB4sH", addr, flags, 0, reg_len,
                             reg.to_bytes(reg_len, "big").ljust(4, b"\x00"), n)
        result, rest = self.request(I3C_CONTROLLER_PRIVATE_TRANSFER, params)
        _addr, _dir, moved = struct.unpack("<BBH", rest[:4])
        return result, rest[4:4 + moved]
