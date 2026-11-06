"""Checks flow_labels/FlowLabels.py without Logic 2: python test_flow_labels.py"""
import os
import sys
import types

saleae, analyzers = types.ModuleType("saleae"), types.ModuleType("saleae.analyzers")


class AnalyzerFrame:
    def __init__(self, type, start_time, end_time, data=None):
        self.type, self.start_time, self.end_time, self.data = type, start_time, end_time, data or {}


analyzers.AnalyzerFrame, analyzers.HighLevelAnalyzer = AnalyzerFrame, object
sys.modules["saleae"], sys.modules["saleae.analyzers"] = saleae, analyzers
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "flow_labels"))
from FlowLabels import FlowLabels  # noqa: E402


def txn(t, addr, read, ack, data=(), stop=True):
    frames = [AnalyzerFrame("start", t, t),
              AnalyzerFrame("address", t, t + 1, {"address": bytes([addr]), "read": read, "ack": ack})]
    frames += [AnalyzerFrame("data", t + 2 + i, t + 3 + i, {"data": bytes([b]), "ack": True})
               for i, b in enumerate(data)]
    return frames + ([AnalyzerFrame("stop", t + 9, t + 9)] if stop else [])


seq = (txn(0, 0x7E, False, False) + txn(10, 0x7E, False, True, (0x01, 0x08))
       + txn(20, 0x7E, False, True, (0x00, 0x08)) + txn(30, 0x2A, True, True, (0xA5,))
       + txn(40, 0x2B, True, False) + txn(50, 0x61, False, True, (0x03,), stop=False)
       + txn(60, 0x61, True, True, tuple(range(19))))
hla = FlowLabels()
got = [r.data["label"] for r in (hla.decode(f) for f in seq) if r]
assert got == ["7Eh NACK: no I3C Basic device, SMBus only", "DISEC(HJ) broadcast, ACK",
               "ENEC(HJ) broadcast, ACK", "SMBus read 0x2A, 1 B", "SMBus read 0x2B: NACK",
               "ARP Get UDID", "ARP Get UDID"], got
print(f"flow_labels: {len(got)} labels ok")
