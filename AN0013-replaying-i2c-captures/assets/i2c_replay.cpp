/**
 * @file i2c_replay.cpp
 * @brief Replay a logic-analyzer I2C capture onto a live bus from a Binho adapter.
 *
 * The CosmicSDK C++ counterpart of i2c_replay.py, with the same parser and the
 * same replay semantics, so a reader can follow the note in either language.
 *
 *   i2c_replay selfcheck
 *   i2c_replay parse   capture.csv
 *   i2c_replay replay  capture.csv
 *   i2c_replay replay  capture.csv --pace --stop-on-nak
 *   i2c_replay scan
 *
 * The input is the CSV that a Saleae I2C analyzer writes: one row per byte,
 * with the columns
 *
 *   Time [s],Packet ID,Address,Data,Read/Write,ACK/NAK
 *
 * Two properties of that file drive the parser, and neither is guessable from
 * the headings. Packet ID groups nothing in a real export, so it is honored
 * only where it varies. And there is no START or STOP column, so a transaction
 * boundary is a silence measured against the capture's own byte rate.
 *
 * Build (see CMakeLists.txt for the portable version):
 *   g++ -std=c++17 -I <cosmicsdk>/include -I <hidapi>/hidapi i2c_replay.cpp \
 *       libcosmicsdk_static.a libhidapi-libusb.a -lusb-1.0 -lpthread -ludev
 *
 * The selfcheck and parse commands need neither an adapter nor a bus.
 */

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "CosmicSDK.hpp"

namespace {

const char* const TOOL_VERSION = "1.0";

const std::vector<std::string> EXPECTED_COLUMNS = {
    "Time [s]", "Packet ID", "Address", "Data", "Read/Write", "ACK/NAK"};

/** Anything that should stop the command with a readable message. */
struct ReplayError : std::runtime_error {
  explicit ReplayError(const std::string& what) : std::runtime_error(what) {}
};

// --------------------------------------------------------------------------
// The capture
// --------------------------------------------------------------------------

/**
 * One transaction phase recovered from the capture.
 *
 * A phase is a run of bytes in one direction to one address. A capture's
 * sub-addressed read is two phases, the pointer write and the read, joined on
 * the wire by a repeated START rather than separated by a STOP, which is what
 * `held` records.
 */
struct Transaction {
  uint8_t address = 0;
  bool read = false;
  double timeS = 0.0;
  std::string packetId;
  std::vector<uint8_t> data;
  // Index of the first byte the captured target refused, or -1 if it
  // acknowledged every byte it was offered.
  int nakAt = -1;
  bool addressNak = false;
  bool missingAck = false;
  // True when the next phase followed without a STOP.
  bool held = false;

  const char* direction() const { return read ? "read" : "write"; }

  /**
   * Is this transaction's NAK just the controller ending a read?
   *
   * A controller acknowledges every byte of a read except the last, and NAKs
   * that one to tell the target to stop driving. It appears in the export as a
   * NAK on the final byte, and it is ordinary I2C, so counting it as a refusal
   * would report a healthy capture as full of failures.
   */
  bool terminatingNak() const {
    return read && !addressNak && nakAt >= 0 &&
           nakAt == static_cast<int>(data.size()) - 1;
  }

  /** Did the captured target actually refuse something? */
  bool refused() const {
    if (addressNak) return true;
    if (nakAt < 0 || terminatingNak()) return false;
    return true;
  }

  std::string describe() const {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6) << std::setw(12) << timeS << "s  "
        << std::left << std::setw(5) << direction() << std::right << " 0x"
        << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
        << static_cast<int>(address) << std::setfill(' ') << std::dec << "  ";
    if (data.empty()) {
      out << "-";
    } else {
      for (size_t i = 0; i < data.size(); ++i) {
        out << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
            << static_cast<int>(data[i]) << std::setfill(' ') << std::dec;
        if (i + 1 < data.size()) out << " ";
      }
    }
    if (addressNak) {
      out << "   [captured: address NAK]";
    } else if (refused()) {
      out << "   [captured: NAK at byte " << nakAt << "]";
    }
    if (missingAck) out << "   [captured: missing ACK/NAK]";
    if (held) out << "   [repeated START, bus held]";
    return out.str();
  }
};

std::string trim(const std::string& s) {
  const size_t first = s.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) return "";
  const size_t last = s.find_last_not_of(" \t\r\n");
  return s.substr(first, last - first + 1);
}

/** Read a Saleae number in whichever base the export used. */
uint32_t parseNumber(const std::string& raw) {
  const std::string text = trim(raw);
  if (text.empty()) throw ReplayError("empty number");
  const char* start = text.c_str();
  int base = 0;  // strtol base 0 covers "0x48" and "72"
  if (text.size() > 2 && text[0] == '0' && (text[1] == 'b' || text[1] == 'B')) {
    start += 2;  // strtol has no binary prefix of its own
    base = 2;
  }
  char* end = nullptr;
  const long value = std::strtol(start, &end, base);
  if (end == start || *end != '\0' || value < 0) {
    throw ReplayError("not a number: '" + text + "'");
  }
  return static_cast<uint32_t>(value);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
  // The analyzer export quotes nothing; the data table does, and is rejected by
  // header before it reaches here.
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.push_back("");
  return fields;
}

/** One row of the export, parsed but not yet grouped. */
struct Row {
  size_t line = 0;
  double time = 0.0;
  std::string packet;
  uint8_t address = 0;
  int byte = -1;  // -1 when the Data column is empty
  bool read = false;
  std::string ack;
};

std::vector<Row> readRows(std::istream& input) {
  std::string header;
  if (!std::getline(input, header)) throw ReplayError("the capture is empty");
  const std::vector<std::string> columns = splitCsvLine(header);

  std::vector<std::string> missing;
  std::vector<int> index(EXPECTED_COLUMNS.size(), -1);
  for (size_t c = 0; c < EXPECTED_COLUMNS.size(); ++c) {
    for (size_t i = 0; i < columns.size(); ++i) {
      if (trim(columns[i]) == EXPECTED_COLUMNS[c]) {
        index[c] = static_cast<int>(i);
        break;
      }
    }
    if (index[c] < 0) missing.push_back(EXPECTED_COLUMNS[c]);
  }
  if (!missing.empty()) {
    std::ostringstream msg;
    msg << "this does not look like a Saleae I2C analyzer export -- missing "
           "column(s) ";
    for (size_t i = 0; i < missing.size(); ++i) {
      msg << missing[i] << (i + 1 < missing.size() ? ", " : "");
    }
    msg << ". Found: " << trim(header)
        << ". Logic 2 writes two different CSVs: use the analyzer's own export, "
           "not the data table.";
    throw ReplayError(msg.str());
  }

  std::vector<Row> rows;
  std::string line;
  size_t lineNo = 1;
  while (std::getline(input, line)) {
    ++lineNo;
    if (trim(line).empty()) continue;
    const std::vector<std::string> f = splitCsvLine(line);
    Row row;
    row.line = lineNo;
    try {
      const std::string timeText = trim(f.at(index[0]));
      row.time = timeText.empty() ? 0.0 : std::stod(timeText);
      row.packet = trim(f.at(index[1]));
      row.address = static_cast<uint8_t>(parseNumber(f.at(index[2])));
      const std::string dataText = trim(f.at(index[3]));
      row.byte = dataText.empty() ? -1
                                  : static_cast<int>(parseNumber(dataText));
      const std::string rw = trim(f.at(index[4]));
      row.read = !rw.empty() && (rw[0] == 'R' || rw[0] == 'r');
      row.ack = trim(f.at(index[5]));
    } catch (const std::exception& exc) {
      throw ReplayError("line " + std::to_string(lineNo) + ": " + exc.what());
    }
    rows.push_back(row);
  }
  return rows;
}

/**
 * Where to cut between one transaction and the next, in seconds.
 *
 * The export has one row per byte and no STOP column, so the only thing that
 * separates two same-direction writes to one address is the silence between
 * them. That silence is measured rather than assumed: the cut is a multiple of
 * the median inter-byte gap in this capture, so it follows the bus clock.
 *
 * The default factor of 8 comes from three separations measured at 100 kHz:
 * 95 us between bytes of one phase, 209 to 210 us across a repeated START (the
 * START plus the re-sent address cost about two byte times), and 11.6 ms
 * between transactions. The cut has to land above the second and far below the
 * third. A factor of 3 would put it at 287 us, only 1.4x above the
 * repeated-START gap, and a slightly slower bus would read repeated STARTs as
 * STOPs.
 */
double gapThreshold(const std::vector<Row>& rows, double gapFactor,
                    double gapSeconds) {
  if (gapSeconds > 0.0) return gapSeconds;
  std::vector<double> deltas;
  bool havePrevious = false;
  double previous = 0.0;
  for (const Row& row : rows) {
    if (row.byte < 0) {
      havePrevious = false;
      continue;
    }
    if (havePrevious) {
      const double delta = row.time - previous;
      if (delta > 0.0) deltas.push_back(delta);
    }
    previous = row.time;
    havePrevious = true;
  }
  if (deltas.size() < 2) return std::numeric_limits<double>::infinity();
  std::sort(deltas.begin(), deltas.end());
  const size_t mid = deltas.size() / 2;
  const double median = (deltas.size() % 2 == 0)
                            ? (deltas[mid - 1] + deltas[mid]) / 2.0
                            : deltas[mid];
  return median * gapFactor;
}

std::vector<Transaction> parseCapture(std::istream& input, double gapFactor = 8.0,
                                      double gapSeconds = 0.0) {
  const std::vector<Row> rows = readRows(input);
  const double threshold = gapThreshold(rows, gapFactor, gapSeconds);

  // Packet ID groups nothing in a real Logic export: measured on a Logic Pro 8
  // capture, all 395 data rows carried Packet ID 0 while the 30 address-NAK
  // rows carried none. Honor the column only where it actually varies.
  std::vector<std::string> packets;
  for (const Row& row : rows) {
    if (!row.packet.empty() &&
        std::find(packets.begin(), packets.end(), row.packet) == packets.end()) {
      packets.push_back(row.packet);
    }
  }
  const bool usePacket = packets.size() > 1;

  std::vector<Transaction> transactions;
  bool haveCurrent = false;
  bool havePreviousTime = false;
  double previousTime = 0.0;

  for (const Row& row : rows) {
    // An address-NAK row carries no data byte: the analyzer writes the address
    // line out on its own precisely because nothing followed it.
    if (row.byte < 0) {
      Transaction txn;
      txn.address = row.address;
      txn.read = row.read;
      txn.timeS = row.time;
      txn.packetId = row.packet;
      txn.addressNak = true;
      txn.missingAck = (row.ack == "Missing ACK/NAK");
      transactions.push_back(txn);
      haveCurrent = false;
      havePreviousTime = false;
      continue;
    }

    const bool haveGap = havePreviousTime;
    const double gap = haveGap ? row.time - previousTime : 0.0;
    bool split = !haveCurrent;
    if (!split) {
      const Transaction& current = transactions.back();
      split = row.address != current.address || row.read != current.read ||
              (haveGap && gap > threshold) ||
              (usePacket && row.packet != current.packetId);
    }
    if (split) {
      if (haveCurrent && haveGap && gap <= threshold) {
        // Contiguous on the wire but a different phase: the capture shows a
        // repeated START, so the previous phase kept the bus.
        transactions.back().held = true;
      }
      Transaction txn;
      txn.address = row.address;
      txn.read = row.read;
      txn.timeS = row.time;
      txn.packetId = row.packet;
      transactions.push_back(txn);
      haveCurrent = true;
    }

    Transaction& current = transactions.back();
    current.data.push_back(static_cast<uint8_t>(row.byte));
    previousTime = row.time;
    havePreviousTime = true;

    if (row.ack == "Missing ACK/NAK") {
      current.missingAck = true;
    } else if (row.ack != "ACK" && current.nakAt < 0) {
      current.nakAt = static_cast<int>(current.data.size()) - 1;
    }
  }
  return transactions;
}

// --------------------------------------------------------------------------
// The adapter
// --------------------------------------------------------------------------

std::string statusName(uint16_t code) {
  switch (code) {
    case SUCCESS: return "SUCCESS";
    case FW_I2C_NACK_ADDRESS: return "FW_I2C_NACK_ADDRESS";
    case FW_I2C_NACK_BYTE: return "FW_I2C_NACK_BYTE";
    case FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED:
      return "FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED";
    case FW_INTERFACE_ALREADY_INITIALIZED:
      return "FW_INTERFACE_ALREADY_INITIALIZED";
    case FW_UNSUPPORTED_COMMAND: return "FW_UNSUPPORTED_COMMAND";
    case FW_I2C_ROLE_CONFLICT: return "FW_I2C_ROLE_CONFLICT";
    default: {
      std::ostringstream out;
      out << "status 0x" << std::hex << std::uppercase << std::setw(4)
          << std::setfill('0') << code;
      return out.str();
    }
  }
}

/** One I2C controller on one bus of one adapter. */
class Bus {
 public:
  Bus(uint8_t bus, uint32_t frequencyHz, uint8_t pullUp, uint16_t voltageMv,
      std::string serial)
      : bus_(bus), frequencyHz_(frequencyHz), pullUp_(pullUp),
        voltageMv_(voltageMv), serial_(std::move(serial)),
        device_(std::make_unique<CosmicSDK>(2000)) {}

  ~Bus() {
    if (connected_) device_->disconnect();
  }

  void open() {
    // connect() takes the first adapter it enumerates, which on a two-adapter
    // bench is as likely to be the target as the controller. Pin it by serial.
    const bool ok = serial_.empty() ? device_->connect()
                                    : device_->connectWithSerialNumber(serial_);
    if (!ok) {
      throw ReplayError(
          serial_.empty()
              ? "no Binho adapter could be opened. With more than one attached, "
                "pass --serial to choose."
              : "no Binho adapter with serial '" + serial_ + "' could be opened");
    }
    connected_ = true;

    CosmicSDK::DeviceInfo info;
    const uint16_t infoStatus = device_->getDeviceInfo(info);
    if (infoStatus != SUCCESS) {
      throw ReplayError("getDeviceInfo: " + statusName(infoStatus));
    }
    std::cout << info.productName << " " << info.serialNumber << " fw "
              << info.fwVersion << ", bus " << static_cast<int>(bus_) << " at "
              << frequencyHz_ << " Hz\n";

    const uint16_t voltageStatus = device_->i2cSetBusVoltage(voltageMv_, false);
    if (voltageStatus != SUCCESS) {
      throw ReplayError("i2cSetBusVoltage: " + statusName(voltageStatus));
    }
    const uint16_t initStatus =
        device_->i2cControllerInit(bus_, frequencyHz_, pullUp_);
    if (initStatus == FW_I2C_ROLE_CONFLICT) {
      throw ReplayError(
          "i2cControllerInit: FW_I2C_ROLE_CONFLICT. This adapter holds the "
          "target role on one of its buses. CosmicSDK 1.5.0 added i2cDeinit to "
          "release it, but firmware carries it only from 4.5.0; on older "
          "firmware the role survives until the adapter is reset.");
    }
    if (initStatus != SUCCESS && initStatus != FW_INTERFACE_ALREADY_INITIALIZED) {
      throw ReplayError("i2cControllerInit: " + statusName(initStatus));
    }
    // An interface that is already up keeps its previous pull-up without
    // saying so, so the parameters are applied either way.
    const uint16_t paramStatus =
        device_->i2cSetParameters(bus_, frequencyHz_, pullUp_);
    if (paramStatus != SUCCESS) {
      std::cerr << "warning: i2cSetParameters: " << statusName(paramStatus)
                << ". The bus keeps whatever pull-up it had, and one stronger "
                   "than about 470 Ohm can make an armed target invisible.\n";
    }
  }

  uint16_t write(uint8_t address, const std::vector<uint8_t>& data, bool nonStop) {
    return device_->i2cWrite(bus_, address, {}, data, nonStop);
  }

  uint16_t read(uint8_t address, uint16_t length, std::vector<uint8_t>& out,
                bool nonStop) {
    return device_->i2cRead(bus_, address, {}, length, out, nonStop);
  }

  uint16_t scan(std::vector<uint8_t>& addresses) {
    uint8_t count7 = 0;
    uint8_t count10 = 0;
    return device_->i2cScanBus(addresses, &count7, &count10, bus_, false);
  }

 private:
  uint8_t bus_;
  uint32_t frequencyHz_;
  uint8_t pullUp_;
  uint16_t voltageMv_;
  std::string serial_;
  std::unique_ptr<CosmicSDK> device_;
  bool connected_ = false;
};

// --------------------------------------------------------------------------
// Replay
// --------------------------------------------------------------------------

/** What happened when one captured transaction was put back on the bus. */
struct Outcome {
  const Transaction* txn = nullptr;
  uint16_t status = SUCCESS;
  std::vector<uint8_t> returned;
  bool wasRead = false;
  bool changed = false;

  bool aborted() const {
    return status == FW_I2C_NACK_ADDRESS || status == FW_I2C_NACK_BYTE;
  }

  std::string describe() const {
    std::ostringstream out;
    out << "  " << std::left << std::setw(5) << txn->direction() << std::right
        << " 0x" << std::hex << std::uppercase << std::setw(2)
        << std::setfill('0') << static_cast<int>(txn->address)
        << std::setfill(' ') << std::dec << "  " << txn->data.size() << "B  ";
    if (status == FW_I2C_NACK_ADDRESS) {
      out << "ABORTED  address NAK -- nothing acknowledged the address, so no "
             "data byte was sent";
    } else if (status == FW_I2C_NACK_BYTE) {
      out << "ABORTED  data NAK -- the target took the address, then refused a "
             "byte; the rest was not sent";
    } else if (status != SUCCESS) {
      out << "FAILED   " << statusName(status);
    } else if (wasRead) {
      out << (changed ? "ok       different bytes than the capture"
                      : "ok       same bytes as the capture");
    } else {
      out << "ok";
    }
    return out.str();
  }
};

struct ReplayOptions {
  bool pace = false;
  bool stopOnNak = false;
  bool includeCapturedNaks = false;
};

/**
 * Put each captured transaction back on the bus, in order.
 *
 * A refusal ends its own transaction: firmware stops the transfer at the byte
 * that was not acknowledged rather than clocking out the rest. The payload is
 * replayed exactly as captured, register-pointer byte included, because the
 * export does not distinguish a sub-address from the data that follows it and
 * neither does the wire. A phase the capture shows as held is replayed with a
 * repeated START, so the framing survives too.
 */
std::vector<Outcome> replay(Bus& bus, const std::vector<Transaction>& transactions,
                            const ReplayOptions& options) {
  std::vector<Outcome> outcomes;
  bool havePreviousTime = false;
  double previousTime = 0.0;

  for (const Transaction& txn : transactions) {
    if (txn.addressNak && !options.includeCapturedNaks) continue;

    if (options.pace && havePreviousTime) {
      const double gap = txn.timeS - previousTime;
      if (gap > 0.0) {
        std::this_thread::sleep_for(std::chrono::duration<double>(gap));
      }
    }
    previousTime = txn.timeS;
    havePreviousTime = true;

    Outcome outcome;
    outcome.txn = &txn;
    outcome.wasRead = txn.read;
    if (txn.read) {
      outcome.status = bus.read(txn.address,
                                static_cast<uint16_t>(txn.data.size()),
                                outcome.returned, txn.held);
      outcome.changed = outcome.returned != txn.data;
    } else {
      outcome.status = bus.write(txn.address, txn.data, txn.held);
    }
    outcomes.push_back(outcome);
    if (outcome.aborted() && options.stopOnNak) break;
  }
  return outcomes;
}

// --------------------------------------------------------------------------
// Offline self-check
// --------------------------------------------------------------------------

const char* const HEADER = "Time [s],Packet ID,Address,Data,Read/Write,ACK/NAK";

std::vector<Transaction> parseText(const std::string& text, double gapFactor = 8.0,
                                   double gapSeconds = 0.0) {
  std::istringstream stream(text);
  return parseCapture(stream, gapFactor, gapSeconds);
}

#define CHECK(condition, message)                                         \
  do {                                                                    \
    if (!(condition)) {                                                   \
      std::cerr << "self-check FAILED: " << (message) << " (line "        \
                << __LINE__ << ")\n";                                     \
      return 1;                                                           \
    }                                                                     \
  } while (0)

int selfcheck() {
  int checks = 0;
  const std::string h = std::string(HEADER) + "\n";

  // A two-byte write the target acknowledged in full.
  {
    auto t = parseText(h +
                       "0.000000000,0,0x50,0x00,Write,ACK\n"
                       "0.000018000,0,0x50,0xAB,Write,ACK\n");
    CHECK(t.size() == 1, "clean write is one transaction");
    CHECK(t[0].address == 0x50 && !t[0].read, "address and direction");
    CHECK(t[0].data == std::vector<uint8_t>({0x00, 0xAB}), "payload");
    CHECK(t[0].nakAt < 0 && !t[0].addressNak, "nothing refused");
    ++checks;
  }

  // Nothing at the address: the analyzer emits the address row alone, with no
  // Packet ID and no data byte.
  {
    auto t = parseText(h + "0.000000000,,0x62,,Write,NAK\n");
    CHECK(t.size() == 1 && t[0].address == 0x62, "one address-NAK transaction");
    CHECK(t[0].addressNak && t[0].data.empty(), "no payload");
    CHECK(t[0].refused(), "an address NAK is a refusal");
    ++checks;
  }

  // The target took the address and the first byte, then refused the second.
  {
    auto t = parseText(h +
                       "0.001000000,4,0x50,0x10,Write,ACK\n"
                       "0.001018000,4,0x50,0x20,Write,NAK\n");
    CHECK(t.size() == 1, "one transaction");
    CHECK(t[0].nakAt == 1, "refusal is on the second byte, not the address");
    CHECK(t[0].refused() && !t[0].terminatingNak(), "a write has no terminator");
    ++checks;
  }

  // A read: the controller NAKs the last byte to end it, which is not a refusal.
  {
    auto t = parseText(h +
                       "0.002000000,7,0x50,0x53,Read,ACK\n"
                       "0.002018000,7,0x50,0xFF,Read,ACK\n"
                       "0.002036000,7,0x50,0x00,Read,NAK\n");
    CHECK(t.size() == 1 && t[0].read, "one read");
    CHECK(t[0].nakAt == 2, "the NAK is on the final byte");
    CHECK(t[0].terminatingNak() && !t[0].refused(), "terminator is not a refusal");
    ++checks;
  }

  // A NAK anywhere else inside a read is a refusal: the target stopped early.
  {
    auto t = parseText(h +
                       "0.002000000,7,0x50,0x53,Read,ACK\n"
                       "0.002018000,7,0x50,0xFF,Read,NAK\n"
                       "0.002036000,7,0x50,0x00,Read,NAK\n");
    CHECK(t[0].nakAt == 1 && t[0].refused(), "early read NAK is a refusal");
    CHECK(!t[0].terminatingNak(), "and is not the terminator");
    ++checks;
  }

  // Exported in decimal, with no Packet ID at all.
  {
    auto t = parseText(h +
                       "0.003000000,,80,1,Write,ACK\n"
                       "0.003018000,,80,2,Write,ACK\n"
                       "0.003036000,,72,9,Write,ACK\n"
                       "0.003054000,,72,9,Read,ACK\n");
    CHECK(t.size() == 3, "address and direction changes split");
    CHECK(t[0].address == 80 && t[0].data == std::vector<uint8_t>({1, 2}),
          "decimal parsed");
    CHECK(t[1].address == 72 && !t[1].read, "second write");
    CHECK(t[2].address == 72 && t[2].read, "then a read");
    ++checks;
  }

  // "Missing ACK/NAK" is the analyzer failing to see the ninth bit. Flagged,
  // but not a refusal.
  {
    auto t = parseText(h + "0.004000000,9,0x50,0x77,Write,Missing ACK/NAK\n");
    CHECK(t[0].missingAck && t[0].nakAt < 0, "missing ACK is not a refusal");
    ++checks;
  }

  // A non-I2C export is refused by name rather than mis-parsed.
  {
    bool refused = false;
    try {
      parseText("name,type,start_time,duration\n0,frame,0.1,0.2\n");
    } catch (const ReplayError& exc) {
      refused = std::string(exc.what()).find("Saleae") != std::string::npos;
    }
    CHECK(refused, "the data table export is refused by name");
    ++checks;
  }

  CHECK(parseNumber("0x50") == 0x50, "hex");
  CHECK(parseNumber("80") == 80, "decimal");
  CHECK(parseNumber("0b1010000") == 0x50, "binary");
  ++checks;

  // The shape a real Logic 2 export actually has: every data row carries
  // Packet ID 0, only the address-NAK row carries none, and a sub-addressed
  // read is a pointer write and a read sharing that useless packet id.
  const std::string realExport =
      h +
      "0.007708480,0,0x50,0x00,Write,ACK\n"
      "0.007911200,0,0x50,0xDE,Read,ACK\n"
      "0.008009760,0,0x50,0xAD,Read,ACK\n"
      "0.008108480,0,0x50,0xBE,Read,ACK\n"
      "0.008203840,0,0x50,0xEF,Read,NAK\n"
      "0.019485920,,0x62,,Write,NAK\n"
      "0.051083680,0,0x50,0x00,Write,ACK\n"
      "0.051178400,0,0x50,0xDE,Write,ACK\n"
      "0.051274080,0,0x50,0xAD,Write,ACK\n"
      "0.051373120,0,0x50,0xBE,Write,ACK\n"
      "0.051468160,0,0x50,0xEF,Write,ACK\n"
      "0.063084000,0,0x50,0x04,Write,ACK\n"
      "0.063179200,0,0x50,0x11,Write,ACK\n"
      "0.063274400,0,0x50,0x22,Write,ACK\n";
  {
    auto t = parseText(realExport);
    CHECK(t.size() == 5, "five phases, not one twelve-byte write");
    CHECK(!t[0].read && t[0].data == std::vector<uint8_t>({0x00}), "pointer write");
    CHECK(t[0].held, "the pointer write is joined to the read by a repeated START");
    CHECK(t[1].read &&
              t[1].data == std::vector<uint8_t>({0xDE, 0xAD, 0xBE, 0xEF}),
          "the read");
    CHECK(t[1].terminatingNak() && !t[1].refused(), "its NAK is the terminator");
    CHECK(!t[1].held, "and it ends with a STOP");
    CHECK(t[2].addressNak && t[2].address == 0x62, "the address NAK");
    CHECK(t[3].data == std::vector<uint8_t>({0x00, 0xDE, 0xAD, 0xBE, 0xEF}),
          "the four-byte write");
    CHECK(!t[3].held, "11.6 ms of silence is a STOP, not a repeated START");
    CHECK(t[4].data == std::vector<uint8_t>({0x04, 0x11, 0x22}), "the last write");
    int refusals = 0;
    for (const Transaction& txn : t) refusals += txn.refused() ? 1 : 0;
    CHECK(refusals == 1, "only the address NAK counts as refused");
    ++checks;
  }

  // The cut is a knob, and moving it changes the reading in both directions.
  {
    // 150 us sits between the 95 us byte gap and the 210 us repeated START:
    // the phases stay correct but the framing is lost.
    auto tight = parseText(realExport, 8.0, 0.00015);
    CHECK(tight.size() == 5, "phases unchanged at 150 us");
    CHECK(!tight[0].held, "at 150 us the repeated START reads as a STOP");
    // 50 us is below the byte gap, so every byte becomes its own phase.
    auto shredded = parseText(realExport, 8.0, 0.00005);
    CHECK(shredded.size() > 10, "below the byte time every byte splits");
    for (const Transaction& txn : shredded) {
      CHECK(txn.data.size() <= 1, "one byte per phase");
    }
    // Wide open, the two writes merge into one.
    auto wide = parseText(realExport, 8.0, 1.0);
    CHECK(wide.size() == 4, "the two writes merge at 1 s");
    CHECK(wide[3].data == std::vector<uint8_t>(
                              {0x00, 0xDE, 0xAD, 0xBE, 0xEF, 0x04, 0x11, 0x22}),
          "merged payload");
    ++checks;
  }

  std::cout << "i2c_replay " << TOOL_VERSION << " self-check: " << checks << "/"
            << checks << " OK\n";
  return 0;
}

#undef CHECK

// --------------------------------------------------------------------------

struct Options {
  std::string command;
  std::string capture;
  std::string serial;
  uint8_t bus = I2C_BUS_A;
  uint32_t frequency = 100000;
  uint8_t pullUp = I2C_PULLUP_2_2kOhm;
  uint16_t voltageMv = 3300;
  double gapFactor = 8.0;
  double gapSeconds = 0.0;
  bool quiet = false;
  ReplayOptions replayOptions;
};

uint8_t pullUpFromName(const std::string& name) {
  static const std::vector<std::pair<std::string, uint8_t>> table = {
      {"150", I2C_PULLUP_150Ohm}, {"220", I2C_PULLUP_220Ohm},
      {"330", I2C_PULLUP_330Ohm}, {"470", I2C_PULLUP_470Ohm},
      {"680", I2C_PULLUP_680Ohm}, {"1k", I2C_PULLUP_1kOhm},
      {"1.5k", I2C_PULLUP_1_5kOhm}, {"2.2k", I2C_PULLUP_2_2kOhm},
      {"3.3k", I2C_PULLUP_3_3kOhm}, {"4k", I2C_PULLUP_4kOhm},
      {"4.7k", I2C_PULLUP_4_7kOhm}, {"10k", I2C_PULLUP_10kOhm},
      {"off", I2C_PULLUP_DISABLE}};
  for (const auto& entry : table) {
    if (entry.first == name) return entry.second;
  }
  std::string offered;
  for (const auto& entry : table) offered += entry.first + " ";
  throw ReplayError("unknown pull-up '" + name + "'. Offered: " + offered);
}

void usage() {
  std::cout
      << "usage: i2c_replay <command> [options]\n\n"
         "  selfcheck              check the parser offline, no adapter needed\n"
         "  parse <capture.csv>    recover transactions and print them\n"
         "  replay <capture.csv>   put the capture back on the bus\n"
         "  scan                   list the addresses answering on the bus\n\n"
         "  --serial <sn>          pin a specific adapter (required when more\n"
         "                         than one is attached)\n"
         "  --bus A|B              I2C bus (default A)\n"
         "  --frequency <Hz>       I2C clock (default 100000)\n"
         "  --pullup <value>       150 220 330 470 680 1k 1.5k 2.2k 3.3k 4k\n"
         "                         4.7k 10k off (default 2.2k)\n"
         "  --gap-factor <n>       boundary cut, times the median byte gap\n"
         "                         (default 8)\n"
         "  --gap-seconds <s>      absolute boundary cut instead\n"
         "  --pace                 honor the capture's own gaps\n"
         "  --stop-on-nak          stop at the first refused transaction\n"
         "  --include-captured-naks  also re-issue already-refused transactions\n"
         "  --quiet                totals only (parse)\n";
}

int cmdParse(const Options& options) {
  std::ifstream file(options.capture);
  if (!file) throw ReplayError("cannot open " + options.capture);
  const std::vector<Transaction> transactions =
      parseCapture(file, options.gapFactor, options.gapSeconds);

  size_t writes = 0, reads = 0, refusals = 0, payload = 0, held = 0;
  for (const Transaction& txn : transactions) {
    txn.read ? ++reads : ++writes;
    if (txn.refused()) ++refusals;
    if (txn.held) ++held;
    payload += txn.data.size();
  }
  std::cout << options.capture << ": " << transactions.size()
            << " transactions (" << writes << " write, " << reads << " read), "
            << payload << " payload bytes, " << refusals
            << " refused in the capture, " << held
            << " held by a repeated START\n";
  if (!transactions.empty()) {
    std::cout << std::fixed << std::setprecision(6) << "capture spans "
              << transactions.back().timeS - transactions.front().timeS
              << " s\n";
  }
  if (!options.quiet) {
    std::cout << "\n";
    for (const Transaction& txn : transactions) {
      std::cout << txn.describe() << "\n";
    }
  }
  return 0;
}

int cmdReplay(const Options& options) {
  std::ifstream file(options.capture);
  if (!file) throw ReplayError("cannot open " + options.capture);
  const std::vector<Transaction> transactions =
      parseCapture(file, options.gapFactor, options.gapSeconds);
  if (transactions.empty()) throw ReplayError("nothing to replay");
  std::cout << options.capture << ": " << transactions.size()
            << " transactions parsed\n";

  Bus bus(options.bus, options.frequency, options.pullUp, options.voltageMv,
          options.serial);
  bus.open();
  std::cout << "\n";
  const std::vector<Outcome> outcomes =
      replay(bus, transactions, options.replayOptions);

  size_t aborted = 0, failed = 0, changed = 0;
  for (const Outcome& outcome : outcomes) {
    std::cout << outcome.describe() << "\n";
    if (outcome.aborted()) {
      ++aborted;
    } else if (outcome.status != SUCCESS) {
      ++failed;
    }
    if (outcome.wasRead && outcome.changed) ++changed;
  }
  std::cout << "\n" << outcomes.size() << " replayed, " << aborted
            << " aborted on a NAK, " << failed << " failed otherwise, "
            << changed << " read(s) answered differently than the capture\n";
  if (aborted > 0) {
    std::cout << "A NAK ends its own transaction at the byte the target "
                 "refused. The adapter reports which of the two refusals it "
                 "was, not how far into the payload it happened.\n";
  }
  return (aborted > 0 || failed > 0) ? 1 : 0;
}

int cmdScan(const Options& options) {
  Bus bus(options.bus, options.frequency, options.pullUp, options.voltageMv,
          options.serial);
  bus.open();
  std::vector<uint8_t> addresses;
  const uint16_t status = bus.scan(addresses);
  if (status == FW_I2C_BUS_WITH_NO_TARGETS_CONNECTED) {
    // An empty bus is an answer, not a failure. Firmware reports it as a
    // status; a pull-up too strong for the target to overcome looks the same.
    std::cout << "scan: nothing answered\n";
    return 0;
  }
  if (status != SUCCESS) throw ReplayError("i2cScanBus: " + statusName(status));
  std::cout << "scan:";
  for (const uint8_t address : addresses) {
    std::cout << " 0x" << std::hex << std::uppercase << std::setw(2)
              << std::setfill('0') << static_cast<int>(address)
              << std::setfill(' ') << std::dec;
  }
  std::cout << "\n";
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  try {
    if (argc < 2) {
      usage();
      return 2;
    }
    options.command = argv[1];
    for (int i = 2; i < argc; ++i) {
      const std::string arg = argv[i];
      auto next = [&]() -> std::string {
        if (i + 1 >= argc) throw ReplayError(arg + " needs a value");
        return argv[++i];
      };
      if (arg == "--bus") {
        const std::string value = next();
        options.bus = (value == "B" || value == "b") ? I2C_BUS_B : I2C_BUS_A;
      } else if (arg == "--frequency") {
        options.frequency = static_cast<uint32_t>(std::stoul(next()));
      } else if (arg == "--pullup") {
        options.pullUp = pullUpFromName(next());
      } else if (arg == "--gap-factor") {
        options.gapFactor = std::stod(next());
      } else if (arg == "--gap-seconds") {
        options.gapSeconds = std::stod(next());
      } else if (arg == "--pace") {
        options.replayOptions.pace = true;
      } else if (arg == "--stop-on-nak") {
        options.replayOptions.stopOnNak = true;
      } else if (arg == "--include-captured-naks") {
        options.replayOptions.includeCapturedNaks = true;
      } else if (arg == "--serial") {
        options.serial = next();
      } else if (arg == "--quiet") {
        options.quiet = true;
      } else if (arg == "--help" || arg == "-h") {
        usage();
        return 0;
      } else if (!arg.empty() && arg[0] == '-') {
        throw ReplayError("unknown option " + arg);
      } else {
        options.capture = arg;
      }
    }

    if (options.command == "selfcheck") return selfcheck();
    if (options.command == "parse" || options.command == "replay") {
      if (options.capture.empty()) {
        throw ReplayError(options.command + " needs a capture file");
      }
      return options.command == "parse" ? cmdParse(options) : cmdReplay(options);
    }
    if (options.command == "scan") return cmdScan(options);
    usage();
    return 2;
  } catch (const ReplayError& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    return 2;
  } catch (const std::exception& exc) {
    std::cerr << "error: " << exc.what() << "\n";
    return 2;
  }
}
