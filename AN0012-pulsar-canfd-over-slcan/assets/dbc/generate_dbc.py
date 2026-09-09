"""Generate the demo node's DBC via cantools' object model.

Built rather than hand-written for two reasons. The 0x210 trace has sixteen
signals whose bit offsets are easy to slip by one, and a DBC with a one-bit
slip decodes to plausible nonsense rather than failing. And going through the
object model means the file cantools writes is by construction a file cantools
can read.
"""
import os
import sys

from cantools.database import Database
from cantools.database.can import Message, Signal
from cantools.database.conversion import BaseConversion


def _signal(name, start, length, signed, scale=1, unit=None, choices=None):
    # cantools 43 carries scaling and value tables in a conversion object
    # rather than as Signal keywords.
    conversion = BaseConversion.factory(scale=scale, offset=0, choices=choices)
    return Signal(name=name, start=start, length=length,
                  byte_order="little_endian", is_signed=signed,
                  conversion=conversion, unit=unit)


def u(name, start, length, scale=1, unit=None, choices=None):
    return _signal(name, start, length, False, scale, unit, choices)


def s(name, start, length, scale=1, unit=None, choices=None):
    return _signal(name, start, length, True, scale, unit, choices)


messages = [
    Message(
        frame_id=0x100, name="Heartbeat", length=8, senders=["Node"],
        comment="Liveness and counters. Classic CAN so any tool can read it.",
        signals=[
            u("Seq", 0, 32),
            u("RxCount", 32, 16),
            u("EchoCount", 48, 16),
        ]),
    Message(
        frame_id=0x110, name="Environment", length=8, senders=["Node"],
        comment="Simulated sensor values. Classic CAN, 100 ms by default; the "
                "rate is changed with SET_RATE on 0x600.",
        signals=[
            s("Temperature", 0, 16, scale=0.1, unit="degC"),
            u("Humidity", 16, 16, scale=0.1, unit="%"),
            u("Pressure", 32, 16, scale=0.1, unit="hPa"),
            u("Flags", 48, 8),
            u("EnvSeq", 56, 8),
        ]),
    Message(
        frame_id=0x200, name="FdRamp", length=64, senders=["Node"], is_fd=True,
        comment="Diagnostic byte ramp. Its only job is to be a 64-byte FD "
                "frame, which a classic-only host cannot receive at all.",
        signals=[u("FirstByte", 0, 8)]),
    Message(
        frame_id=0x210, name="Telemetry", length=48, senders=["Node"], is_fd=True,
        comment="A 16-sample trace in one frame. The same trace over classic "
                "CAN is six frames plus a reassembly scheme.",
        signals=[
            u("TraceSeq", 0, 32),
            u("UptimeMs", 32, 32, unit="ms"),
            u("SampleCount", 64, 16),
            u("Reserved", 80, 16),
            # Header is 12 bytes, then sixteen int16 samples.
            *[s(f"Sample{i:02d}", (12 + 2 * i) * 8, 16) for i in range(16)],
        ]),
    Message(
        frame_id=0x321, name="EchoResponse", length=8, senders=["Node"],
        comment="Echo of a 0x123 request. Length and frame type follow the "
                "request, so the DBC can only describe the first bytes.",
        signals=[u("Byte0", 0, 8)]),
    Message(
        frame_id=0x600, name="CommandRequest", length=8, senders=["Host"],
        comment="1 = INFO, 2 = COUNTERS, 3 = SET_RATE with Arg in ms.",
        signals=[u("Command", 0, 8), u("Arg", 8, 16)]),
    Message(
        frame_id=0x601, name="CommandResponse", length=8, senders=["Node"],
        comment="First byte echoes the command, so a reply matches its "
                "request without a transaction id.",
        signals=[
            u("Command", 0, 8,
              choices={1: "INFO", 2: "COUNTERS", 3: "SET_RATE", 255: "ERROR"}),
            u("Payload0", 8, 8),
            u("Payload1", 16, 8),
        ]),
]

db = Database(messages=messages)
text = db.as_dbc_string()

# cantools writes a valid DBC but does not emit VFrameFormat, so the FD flag is
# lost on the way out and a reader treats the 48- and 64-byte messages as
# malformed classic frames. The attribute is appended here, and the file is
# reloaded below to confirm the flag survived rather than assuming it did.
FD_IDS = [m.frame_id for m in messages if m.is_fd]
extra = [
    'BA_DEF_ BO_  "VFrameFormat" ENUM  "StandardCAN","ExtendedCAN","reserved",'
    '"J1939PG","StandardCAN_FD","ExtendedCAN_FD";',
    'BA_DEF_DEF_  "VFrameFormat" "StandardCAN";',
]
extra += [f'BA_ "VFrameFormat" BO_ {i} 4;' for i in FD_IDS]   # 4 = StandardCAN_FD
text = text.rstrip("\n") + "\n" + "\n".join(extra) + "\n"

# Default to the DBC that ships beside this script, so running it with no
# argument regenerates the shipped file in place and the self-check below runs
# against what the reader actually has.
if len(sys.argv) > 2:
    raise SystemExit("usage: generate_dbc.py [output.dbc]")
path = sys.argv[1] if len(sys.argv) == 2 else os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "binho_canfd_demo.dbc")
with open(path, "w", encoding="utf-8", newline="\n") as handle:
    handle.write(text)
print(f"  wrote {path}")

import cantools  # noqa: E402  (import here so the write happens even if absent)

reloaded = cantools.database.load_file(path)
wrong = [f"0x{m.frame_id:03X}" for m in reloaded.messages
         if m.is_fd != (m.frame_id in FD_IDS)]
if wrong:
    raise SystemExit(f"  FD flag did not survive the round trip for {wrong}")
print(f"  FD flag survives for {[f'0x{i:03X}' for i in FD_IDS]}")
