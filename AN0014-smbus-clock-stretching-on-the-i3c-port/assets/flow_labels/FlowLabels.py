"""Logic 2 high-level analyzer: one label per transaction of the AN0014 `handoff` flow.

Input is Logic 2's I2C analyzer. A broadcast CCC to 7Eh decodes cleanly there: the CCC byte and the
defining byte end in the I3C T-bit, which the I2C analyzer reads as the ninth (ACK) bit.
"""
from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame

CCC = {0x00: "ENEC", 0x01: "DISEC"}
EVENTS = {0x01: "INT", 0x02: "CR", 0x08: "HJ"}


def events(b):
    return "+".join(n for bit, n in EVENTS.items() if b & bit) or f"0x{b:02X}"


class FlowLabels(HighLevelAnalyzer):
    result_types = {"step": {"format": "{{data.label}}"}}

    def __init__(self):
        self.t0 = None
        self.addr = None
        self.read = False
        self.ack = False
        self.data = []

    def decode(self, frame: AnalyzerFrame):
        if frame.type == "start":
            out = self.flush(frame.start_time)
            self.t0, self.addr, self.data = frame.start_time, None, []
            return out
        if frame.type == "address":
            self.addr = frame.data["address"][0]
            self.read = frame.data["read"]
            self.ack = frame.data["ack"]
        elif frame.type == "data":
            self.data.append(frame.data["data"][0])
        elif frame.type == "stop":
            return self.flush(frame.end_time)
        return None

    def flush(self, end):
        if self.t0 is None or self.addr is None:
            self.t0 = None
            return None
        a, d = self.addr, self.data
        if a == 0x7E and not self.read:
            if not self.ack:
                label = "7Eh NACK: no I3C Basic device, SMBus only"
            elif d and d[0] in CCC:
                label = f"{CCC[d[0]]}({events(d[1]) if len(d) > 1 else ''}) broadcast, ACK"
            else:
                label = "7Eh broadcast, ACK"
        elif a == 0x61:
            label = "ARP Get UDID" + ("" if self.ack else ": no ARP device (NACK)")
        else:
            kind = "read" if self.read else "write"
            label = f"SMBus {kind} 0x{a:02X}" + (f", {len(d)} B" if self.ack else ": NACK")
        frame = AnalyzerFrame("step", self.t0, end, {"label": label})
        self.t0 = None
        return frame
