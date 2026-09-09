// CAN-FD test node for the Adafruit Feather M4 CAN Express (SAME51J19A).
//
// Written to be the far end of a CAN-FD bus whose near end is a Binho Pulsar
// running its SLCAN bridge. It is two things at once, deliberately kept apart.
//
// The DIAGNOSTIC LAYER is the part to trust when something is wrong. It does
// the least it can and says exactly what happened:
//
//   0x100  classic, 8 bytes, 500 ms   heartbeat and counters
//   0x200  FD+BRS, 64 bytes, 1000 ms  a byte ramp
//   0x123 -> 0x321                    echo, preserving frame type and BRS
//
// The ramp on 0x200 is the frame a classic-CAN-only host cannot receive, so a
// tool that shows it is genuinely doing FD. The echo proves the direction a
// transmit test cannot: that this node received what was sent, and that an FD
// request came back as an FD reply rather than being quietly downgraded.
//
// The SIMULATED DEVICE LAYER sits on top and exists so a host has something
// worth decoding. Nothing here is needed to diagnose the bus:
//
//   0x110  classic, 8 bytes, 100 ms   environment: temperature, humidity,
//                                     pressure, flags, sequence
//   0x210  FD+BRS, 48 bytes, 200 ms   a 16-sample waveform trace in one frame
//   0x600 -> 0x601                     command endpoint: INFO, COUNTERS,
//                                     SET_RATE
//
// 0x210 is the message that makes the case for FD rather than just exercising
// it. Sixteen 16-bit samples plus a header is 44 bytes; classic CAN would need
// six frames and a reassembly scheme to carry the same trace, and would
// interleave them with everything else on the bus. Here it is one frame.
//
// Bit timing has to match the Pulsar: 500 kbit/s arbitration with the data
// phase four times faster, which is 2 Mbit/s, and corresponds to `S6` then
// `Y2` on the SLCAN side. Both ends must agree before either opens the
// channel, because the Pulsar captures S/Y when it processes `O`.
//
// CAN1 is the module to use on this board: it is the one wired to the onboard
// transceiver, and the only one whose CANH/CANL reach the header. CAN0 exists
// but its pins are D12/D13 with nothing behind them.
//
// beginFD() also brings the transceiver out of standby (PB12 low) and enables
// its boost converter (PB13 high). Those are the two pins that, left alone,
// produce a node that looks configured and never drives the bus.

#define CAN0_MESSAGE_RAM_SIZE (0)     // CAN0 unused, so it claims no message RAM
#define CAN1_MESSAGE_RAM_SIZE (1728)

#include <ACANFD_FeatherM4CAN.h>
#include <math.h>

static const uint32_t ARBITRATION_BITRATE = 500UL * 1000UL;   // SLCAN 'S6'
static const DataBitRateFactor DATA_FACTOR = DataBitRateFactor::x4;  // 2 Mbit/s, 'Y2'

// --- diagnostic layer
static const uint32_t HEARTBEAT_ID = 0x100;
static const uint32_t FD_RAMP_ID   = 0x200;
static const uint32_t ECHO_REQ_ID  = 0x123;
static const uint32_t ECHO_RSP_ID  = 0x321;

// --- simulated device layer
static const uint32_t ENVIRONMENT_ID = 0x110;
static const uint32_t TELEMETRY_ID   = 0x210;
static const uint32_t COMMAND_REQ_ID = 0x600;
static const uint32_t COMMAND_RSP_ID = 0x601;

static const uint32_t HEARTBEAT_PERIOD_MS  = 500;
static const uint32_t FD_RAMP_PERIOD_MS    = 1000;
static const uint32_t TELEMETRY_PERIOD_MS  = 200;
static const uint16_t ENVIRONMENT_DEFAULT_MS = 100;
static const uint16_t ENVIRONMENT_MIN_MS   = 10;    // keeps a bad SET_RATE from
static const uint16_t ENVIRONMENT_MAX_MS   = 10000; // flooding or stalling the bus

// Command bytes for 0x600.
static const uint8_t CMD_INFO     = 0x01;
static const uint8_t CMD_COUNTERS = 0x02;
static const uint8_t CMD_SET_RATE = 0x03;
static const uint8_t CMD_ERROR    = 0xFF;

static const uint8_t FW_MAJOR = 1;
static const uint8_t FW_MINOR = 0;

static const uint8_t TRACE_SAMPLES = 16;

static uint32_t gHeartbeatDue   = 0;
static uint32_t gFdRampDue      = 0;
static uint32_t gEnvironmentDue = 0;
static uint32_t gTelemetryDue   = 0;
static uint16_t gEnvironmentPeriod = ENVIRONMENT_DEFAULT_MS;

static uint32_t gHeartbeatSeq = 0;
static uint32_t gFdSeq        = 0;
static uint32_t gEnvSeq       = 0;
static uint32_t gTelemetrySeq = 0;
static uint32_t gTxCount      = 0;
static uint32_t gRxCount      = 0;
static uint32_t gEchoCount    = 0;
static uint32_t gCommandCount = 0;
static bool     gBusReady     = false;
static bool     gVerbose      = true;   // console echo of every frame

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

static const char *typeName(const CANFDMessage::Type inType) {
  switch (inType) {
    case CANFDMessage::CAN_REMOTE:                 return "classic-rtr";
    case CANFDMessage::CAN_DATA:                   return "classic";
    case CANFDMessage::CANFD_NO_BIT_RATE_SWITCH:   return "fd";
    case CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH: return "fd+brs";
  }
  return "?";
}

static void printFrame(const char *inPrefix, const CANFDMessage &inFrame) {
  if (!gVerbose) return;
  Serial.print(inPrefix);
  Serial.print(" id=0x");
  Serial.print(inFrame.id, HEX);
  Serial.print(inFrame.ext ? " ext" : " std");
  Serial.print(" ");
  Serial.print(typeName(inFrame.type));
  Serial.print(" len=");
  Serial.print(inFrame.len);
  Serial.print(" data=");
  for (uint8_t i = 0; i < inFrame.len; i++) {
    if (inFrame.data[i] < 0x10) Serial.print("0");
    Serial.print(inFrame.data[i], HEX);
  }
  Serial.println();
}

static bool send(const CANFDMessage &inFrame, const char *inPrefix) {
  const uint32_t status = can1.tryToSendReturnStatusFD(inFrame);
  if (status == 0) {
    gTxCount++;
    printFrame(inPrefix, inFrame);
    return true;
  }
  // Worth reading the code rather than treating any failure as "the bus is
  // broken", because the three you actually see mean different things:
  //
  //   1 kInvalidMessage             the frame is malformed, e.g. a length
  //                                 that is not a legal FD length
  //   2 kTransmitBufferIndexTooLarge  idx is non-zero, so the driver tried to
  //                                 use a dedicated TX buffer that does not
  //                                 exist. Copying a received frame and
  //                                 resending it does this, because idx on a
  //                                 received frame is the driver's own index
  //   3 kTransmitBufferOverflow     the queue is full. Usually not a local
  //                                 fault: if nothing on the bus acknowledges
  //                                 a frame, the controller retries it and the
  //                                 queue backs up. A peer with its channel
  //                                 closed, or a missing terminator, both look
  //                                 exactly like this
  Serial.print("TX-FAIL 0x");
  Serial.print(status, HEX);
  Serial.print(" id=0x");
  Serial.println(inFrame.id, HEX);
  return false;
}

// ---------------------------------------------------------------------------
// Byte packing. Little-endian throughout, which is what the host decoder in
// canfd_slcan.py expects; CAN itself imposes no byte order, so this is a
// choice that has to be written down somewhere or the two ends disagree.
// ---------------------------------------------------------------------------

static void put16(uint8_t *outBuffer, const uint16_t inValue) {
  outBuffer[0] = static_cast<uint8_t>(inValue);
  outBuffer[1] = static_cast<uint8_t>(inValue >> 8);
}

static void put32(uint8_t *outBuffer, const uint32_t inValue) {
  outBuffer[0] = static_cast<uint8_t>(inValue);
  outBuffer[1] = static_cast<uint8_t>(inValue >> 8);
  outBuffer[2] = static_cast<uint8_t>(inValue >> 16);
  outBuffer[3] = static_cast<uint8_t>(inValue >> 24);
}

// ---------------------------------------------------------------------------
// Simulated signals. Smooth and bounded rather than random, so a host plotting
// them sees something that looks like an instrument and a dropped frame shows
// up as a visible discontinuity instead of hiding in noise.
// ---------------------------------------------------------------------------

static float phase(const float inPeriodSeconds) {
  return sinf(2.0f * PI * (millis() / 1000.0f) / inPeriodSeconds);
}

static void sendEnvironment() {
  const int16_t  temperature = static_cast<int16_t>((21.5f + 2.0f * phase(30.0f)) * 10.0f);
  const uint16_t humidity    = static_cast<uint16_t>((45.0f + 5.0f * phase(47.0f)) * 10.0f);
  const uint16_t pressure    = static_cast<uint16_t>((1013.2f + 1.5f * phase(73.0f)) * 10.0f);

  CANFDMessage frame;
  frame.id   = ENVIRONMENT_ID;
  frame.ext  = false;
  frame.type = CANFDMessage::CAN_DATA;    // classic, so any CAN tool reads it
  frame.len  = 8;
  put16(&frame.data[0], static_cast<uint16_t>(temperature));  // 0.1 degC, signed
  put16(&frame.data[2], humidity);                            // 0.1 %RH
  put16(&frame.data[4], pressure);                            // 0.1 hPa
  frame.data[6] = 0x01;                                       // flags: sensor ok
  frame.data[7] = static_cast<uint8_t>(++gEnvSeq);
  send(frame, "TX");
}

static void sendTelemetry() {
  // A 16-sample trace, header included, in one frame. This is the message that
  // makes the case for FD: the same trace over classic CAN is six frames plus
  // a reassembly scheme, interleaved with everything else on the bus.
  CANFDMessage frame;
  frame.id   = TELEMETRY_ID;
  frame.ext  = false;
  frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
  frame.len  = 48;                        // a legal FD length; DLC 0xE
  memset(frame.data, 0, sizeof(frame.data));

  gTelemetrySeq++;
  put32(&frame.data[0], gTelemetrySeq);
  put32(&frame.data[4], millis());
  put16(&frame.data[8], TRACE_SAMPLES);
  put16(&frame.data[10], 0);              // reserved, keeps the trace aligned

  for (uint8_t i = 0; i < TRACE_SAMPLES; i++) {
    // One cycle across the trace, drifting slowly between frames, so
    // consecutive frames differ and the sequence is visible in the data.
    const float angle = 2.0f * PI * i / TRACE_SAMPLES
                        + gTelemetrySeq * 0.05f;
    const int16_t sample = static_cast<int16_t>(sinf(angle) * 8000.0f);
    put16(&frame.data[12 + 2 * i], static_cast<uint16_t>(sample));
  }
  send(frame, "TX");
}

// ---------------------------------------------------------------------------
// Command endpoint. Request on 0x600, reply on 0x601, first byte echoed back
// so a host can match a reply to its request without a transaction id.
// ---------------------------------------------------------------------------

static void handleCommand(const CANFDMessage &inRequest) {
  CANFDMessage reply;
  reply.id   = COMMAND_RSP_ID;
  reply.ext  = false;
  reply.type = CANFDMessage::CAN_DATA;
  reply.idx  = 0;
  memset(reply.data, 0, sizeof(reply.data));

  const uint8_t command = (inRequest.len > 0) ? inRequest.data[0] : 0;
  reply.data[0] = command;

  switch (command) {
    case CMD_INFO:
      reply.len = 8;
      reply.data[1] = 'B';
      reply.data[2] = 'I';
      reply.data[3] = 'N';
      reply.data[4] = FW_MAJOR;
      reply.data[5] = FW_MINOR;
      // Feature bits, so a host can tell what to expect without a version
      // table: bit 0 CAN-FD, bit 1 BRS, bit 2 the 0x210 trace.
      reply.data[6] = 0x07;
      reply.data[7] = TRACE_SAMPLES;
      break;

    case CMD_COUNTERS:
      // Truncated to 16 bits each so three counters fit a classic frame; the
      // host is watching for movement, not absolute totals.
      reply.len = 8;
      put16(&reply.data[1], static_cast<uint16_t>(gTxCount));
      put16(&reply.data[3], static_cast<uint16_t>(gRxCount));
      put16(&reply.data[5], static_cast<uint16_t>(gEchoCount));
      reply.data[7] = static_cast<uint8_t>(gCommandCount);
      break;

    case CMD_SET_RATE: {
      // Clamped rather than rejected, and the accepted value is returned, so
      // the host learns what actually took effect instead of assuming.
      uint16_t requested = ENVIRONMENT_DEFAULT_MS;
      if (inRequest.len >= 3) {
        requested = static_cast<uint16_t>(inRequest.data[1])
                    | static_cast<uint16_t>(inRequest.data[2] << 8);
      }
      if (requested < ENVIRONMENT_MIN_MS) requested = ENVIRONMENT_MIN_MS;
      if (requested > ENVIRONMENT_MAX_MS) requested = ENVIRONMENT_MAX_MS;
      gEnvironmentPeriod = requested;
      gEnvironmentDue = millis() + gEnvironmentPeriod;
      reply.len = 4;
      put16(&reply.data[1], gEnvironmentPeriod);
      reply.data[3] = 0x00;               // 0 = accepted
      break;
    }

    default:
      reply.len = 2;
      reply.data[0] = CMD_ERROR;
      reply.data[1] = command;            // the command we did not understand
      break;
  }

  if (send(reply, "TX-CMD")) {
    gCommandCount++;
  }
}

// ---------------------------------------------------------------------------

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);

  // Wait briefly for a console, but never block forever: this node has to run
  // headless on a bus, not only when a terminal happens to be attached.
  const uint32_t deadline = millis() + 3000;
  while (!Serial && millis() < deadline) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(50);
  }
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println();
  Serial.println("=== Feather M4 CAN :: CAN-FD test node ===");
  Serial.print("arbitration ");
  Serial.print(ARBITRATION_BITRATE);
  Serial.print(" bit/s, data x");
  Serial.print(static_cast<uint32_t>(DATA_FACTOR));
  Serial.print(" = ");
  Serial.print(ARBITRATION_BITRATE * static_cast<uint32_t>(DATA_FACTOR));
  Serial.println(" bit/s   (SLCAN: S6 then Y2)");

  ACANFD_FeatherM4CAN_Settings settings(
      ACANFD_FeatherM4CAN_Settings::CLOCK_48MHz, ARBITRATION_BITRATE, DATA_FACTOR);
  // NORMAL_FD is the default and is what puts the node on a real bus. The
  // loopback modes are useful for proving the controller works with nothing
  // attached, which is worth doing before blaming the wiring.
  settings.mModuleMode = ACANFD_FeatherM4CAN_Settings::NORMAL_FD;

  const uint32_t errorCode = can1.beginFD(settings);
  Serial.print("message RAM required: ");
  Serial.print(can1.messageRamRequiredMinimumSize());
  Serial.print(" words (allocated ");
  Serial.print(CAN1_MESSAGE_RAM_SIZE);
  Serial.println(")");

  if (errorCode == 0) {
    gBusReady = true;
    Serial.println("CAN1 configured, transceiver active");
    Serial.println("diagnostic: 0x100 heartbeat, 0x200 ramp, 0x123->0x321 echo");
    Serial.println("device:     0x110 environment, 0x210 trace, 0x600->0x601 cmd");
  } else {
    // Reported rather than retried: a bad bit-timing request is a
    // configuration mistake, and carrying on would hide it.
    Serial.print("CAN1 configuration FAILED, error 0x");
    Serial.println(errorCode, HEX);
  }

  const uint32_t now = millis();
  gHeartbeatDue   = now + HEARTBEAT_PERIOD_MS;
  gFdRampDue      = now + FD_RAMP_PERIOD_MS;
  gEnvironmentDue = now + gEnvironmentPeriod;
  gTelemetryDue   = now + TELEMETRY_PERIOD_MS;
}

// ---------------------------------------------------------------------------

void loop() {
  if (!gBusReady) {
    // Blink slowly, so a misconfigured node is visibly different from a
    // working one without needing the console.
    digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
    return;
  }

  const uint32_t now = millis();

  // --- diagnostic: classic heartbeat with the counters
  if (now >= gHeartbeatDue) {
    gHeartbeatDue = now + HEARTBEAT_PERIOD_MS;
    gHeartbeatSeq++;

    CANFDMessage frame;
    frame.id   = HEARTBEAT_ID;
    frame.ext  = false;
    frame.type = CANFDMessage::CAN_DATA;
    frame.len  = 8;
    put32(&frame.data[0], gHeartbeatSeq);
    put16(&frame.data[4], static_cast<uint16_t>(gRxCount));
    put16(&frame.data[6], static_cast<uint16_t>(gEchoCount));
    send(frame, "TX");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // --- diagnostic: the 64-byte ramp a classic-only host cannot see
  if (now >= gFdRampDue) {
    gFdRampDue = now + FD_RAMP_PERIOD_MS;
    gFdSeq++;

    CANFDMessage frame;
    frame.id   = FD_RAMP_ID;
    frame.ext  = false;
    frame.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
    frame.len  = 64;                       // a legal FD length; DLC 0xF
    for (uint8_t i = 0; i < 64; i++) {
      frame.data[i] = static_cast<uint8_t>(gFdSeq + i);
    }
    send(frame, "TX");
  }

  // --- device: environment, at whatever rate SET_RATE last accepted
  if (now >= gEnvironmentDue) {
    gEnvironmentDue = now + gEnvironmentPeriod;
    sendEnvironment();
  }

  // --- device: the FD trace
  if (now >= gTelemetryDue) {
    gTelemetryDue = now + TELEMETRY_PERIOD_MS;
    sendTelemetry();
  }

  // --- receive, then echo or answer
  CANFDMessage frame;
  while (can1.receiveFD0(frame)) {
    gRxCount++;
    printFrame("RX", frame);

    if (frame.id == ECHO_REQ_ID) {
      // Echo the payload on a different ID, keeping the frame type and the
      // BRS flag. Preserving the type is the point: it lets the host confirm
      // that an FD request came back as an FD response.
      CANFDMessage reply = frame;
      reply.id  = ECHO_RSP_ID;
      reply.ext = frame.ext;
      // idx must be cleared. It is the driver's own field, and on a received
      // frame it holds the index the frame arrived under; leaving it set makes
      // tryToSendReturnStatusFD treat the reply as targeting a dedicated
      // transmit buffer of that number, which fails with status 2 rather than
      // going out on the bus. Zero means "use the transmit FIFO".
      reply.idx = 0;
      if (send(reply, "TX-ECHO")) {
        gEchoCount++;
      }
    } else if (frame.id == COMMAND_REQ_ID) {
      handleCommand(frame);
    }
  }
}
