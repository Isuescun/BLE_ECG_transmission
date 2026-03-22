#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define DATA_CHAR_UUID "abcdefab-1234-5678-1234-abcdefabcdef"  // NOTIFY — data stream
#define CMD_CHAR_UUID  "abcdefab-1234-5678-1234-abcdefabcde0"  // WRITE + NOTIFY — commands

// Must match peripheral
#define NUM_CHANNELS  19
#define BATCH_SIZE    10
#define PACKET_BYTES  (2 + BATCH_SIZE * NUM_CHANNELS * 2)  // 382 bytes

#define SYNC_A  0xAA
#define SYNC_B  0x55

// Fragment reassembly for data stream
uint8_t assemblyBuf[PACKET_BYTES];
int     assemblyLen = 0;

// BLE state
BLERemoteCharacteristic* dataChar = nullptr;
BLERemoteCharacteristic* cmdChar  = nullptr;
BLEClient*               client   = nullptr;
BLEAdvertisedDevice*     myDevice = nullptr;
bool connected = false;
bool doConnect = false;

uint16_t lastPacketId   = 0;
bool     firstPacket    = true;
uint32_t droppedPackets = 0;

// ── Data stream callback ───────────────────────────────────────────────────────
// Reassembles fragmented BLE packets and forwards complete frames to PC
void dataNotifyCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* data,
    size_t   length,
    bool     isNotify)
{
  if (assemblyLen >= PACKET_BYTES) assemblyLen = 0;

  int canCopy = min((int)length, PACKET_BYTES - assemblyLen);
  memcpy(assemblyBuf + assemblyLen, data, canCopy);
  assemblyLen += canCopy;

  if (assemblyLen >= PACKET_BYTES) {
    uint8_t* pkt = assemblyBuf;

    // Drop detection
    uint16_t packetId = (uint16_t)pkt[0] | ((uint16_t)pkt[1] << 8);
    if (!firstPacket) {
      uint16_t expected = lastPacketId + 1;
      if (packetId != expected) {
        uint16_t gap = packetId - expected;
        droppedPackets += gap;
        Serial.print("[WARN] Dropped ");
        Serial.print(gap);
        Serial.print(" batch(es). Total: ");
        Serial.println(droppedPackets);
      }
    }
    lastPacketId = packetId;
    firstPacket  = false;

    // Forward to PC as framed binary
    Serial.write(SYNC_A);
    Serial.write(SYNC_B);
    Serial.write(pkt, PACKET_BYTES);
    Serial.write(0x0A);

    assemblyLen = 0;
  }
}

// ── Command response callback ─────────────────────────────────────────────────
// Peripheral responded to a command (e.g. "BATTERY:87") — forward to PC as text
void cmdNotifyCallback(
    BLERemoteCharacteristic* pChar,
    uint8_t* data,
    size_t   length,
    bool     isNotify)
{
  // Responses are plain text — prefix with CMD: so plot.py can distinguish
  // them from binary data frames
  Serial.print("CMD:");
  Serial.write(data, length);
  Serial.println();
}

// ── BLE scan / connection ─────────────────────────────────────────────────────
class MyCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
      Serial.println("Found target device!");
      BLEDevice::getScan()->stop();
      myDevice  = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient)    { connected = true;  Serial.println("onConnect"); }
  void onDisconnect(BLEClient* pclient) { connected = false; Serial.println("onDisconnect"); }
};

bool connectToServer() {
  Serial.print("Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  client = BLEDevice::createClient();
  client->setClientCallbacks(new MyClientCallback());
  client->connect(myDevice);
  Serial.println(" - Connected");

  BLERemoteService* pService = client->getService(SERVICE_UUID);
  if (!pService) {
    Serial.println("Service not found -- disconnecting");
    client->disconnect();
    return false;
  }

  // ── Data characteristic ───────────────────────────────────────────────────
  dataChar = pService->getCharacteristic(DATA_CHAR_UUID);
  if (!dataChar) {
    Serial.println("Data characteristic not found -- disconnecting");
    client->disconnect();
    return false;
  }
  if (dataChar->canNotify()) {
    dataChar->registerForNotify(dataNotifyCallback);
    Serial.println(" - Data stream subscribed");
  }

  // ── Command characteristic ────────────────────────────────────────────────
  cmdChar = pService->getCharacteristic(CMD_CHAR_UUID);
  if (!cmdChar) {
    Serial.println("Command characteristic not found -- disconnecting");
    client->disconnect();
    return false;
  }
  if (cmdChar->canNotify()) {
    cmdChar->registerForNotify(cmdNotifyCallback);
    Serial.println(" - Command channel subscribed");
  }

  Serial.println("Ready -- data streaming, command channel open");
  Serial.println("Send commands via serial: GET_BATTERY / START / STOP");
  connected = true;
  return true;
}

void setup() {
  Serial.begin(1000000);
  delay(1000);
  Serial.println("Starting BLE Central -- XIAO compatible");

  BLEDevice::init("ESP32_CENTRAL");
  BLEDevice::setMTU(247);

  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyCallbacks());
  scan->setInterval(1349);
  scan->setWindow(449);
  scan->setActiveScan(true);
  scan->start(5, false);
}

void loop() {
  if (doConnect) {
    if (connectToServer()) {
      Serial.println("Connected to BLE server.");
    } else {
      Serial.println("Connection failed.");
    }
    doConnect = false;
  }

  // ── Forward serial commands from PC to peripheral over BLE ────────────────
  // plot.py or any terminal can send: GET_BATTERY\n  START\n  STOP\n
  if (connected && cmdChar != nullptr && Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("Forwarding command: ");
      Serial.println(cmd);
      cmdChar->writeValue((uint8_t*)cmd.c_str(), cmd.length(), true);;
    }
  }

  if (!connected) {
    delay(1000);
    Serial.println("Not connected -- scanning...");
    BLEDevice::getScan()->start(5, false);
  }

  delay(10);  // reduced from 1000ms so serial commands are responsive
}
