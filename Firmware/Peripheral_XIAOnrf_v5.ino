#include <bluefruit.h>
#include <Adafruit_TinyUSB.h>
#include <math.h>

// ── UUIDs (must match central) ────────────────────────────────────────────────
#define SERVICE_UUID  "12345678-1234-1234-1234-1234567890ab"
#define DATA_CHAR_UUID "abcdefab-1234-5678-1234-abcdefabcdef"
#define CMD_CHAR_UUID  "abcdefab-1234-5678-1234-abcdefabcde0"  // new: command channel

// ── Signal parameters ─────────────────────────────────────────────────────────
#define ADC_MID        2048
#define ADC_AMPLITUDE  1000
#define NUM_CHANNELS   19
#define SAMPLE_RATE_HZ 1000
#define BATCH_SIZE     10

// ── Packet layout ─────────────────────────────────────────────────────────────
//  Byte 0-1  : packet_id (uint16_t, little-endian)
//  Byte 2+   : [t0_ch0][t0_ch1]...[t0_ch18][t1_ch0]...[t9_ch18]
//  Total     : 2 + BATCH_SIZE * NUM_CHANNELS * 2 = 382 bytes
#define PACKET_HEADER_BYTES  2
#define PACKET_PAYLOAD_BYTES (BATCH_SIZE * NUM_CHANNELS * 2)
#define PACKET_TOTAL_BYTES   (PACKET_HEADER_BYTES + PACKET_PAYLOAD_BYTES)

uint8_t  packetBuffer[PACKET_TOTAL_BYTES];
uint16_t packetId   = 0;
int      batchIndex = 0;

// ── Per-channel phase accumulators ────────────────────────────────────────────
float phase[NUM_CHANNELS];
float freqHz[NUM_CHANNELS];

// ── Timing ────────────────────────────────────────────────────────────────────
unsigned long previousMicros = 0;
const unsigned long intervalMicros = 1000000 / SAMPLE_RATE_HZ;

// ── Battery (XIAO nRF52840 specific) ─────────────────────────────────────────
#define BAT_READ     14      // P0_14 — pull LOW to enable battery voltage divider path
#define CHARGE_LED   23      // P0_17 — LOW = USB charging, HIGH = on battery
const double vRef        = 3.3;
const unsigned int numReadings = 256;  // 8-bit ADC

uint8_t readBatteryPercent() {
  pinMode(BAT_READ, OUTPUT);
  digitalWrite(BAT_READ, LOW);    // enable divider path
  delay(1);                        // settle
  analogReadResolution(8);
  unsigned int adcCount   = analogRead(PIN_VBAT);
  double adcVoltage = (adcCount * vRef) / numReadings;
  double vBat       = adcVoltage * 3.30014;  // correct for divider ratio
  double level      = (vBat - 3.2) / (4.17 - 3.2) * 100.0;
  level = constrain(level, 0.0, 100.0);
  pinMode(BAT_READ, INPUT);       // release pin to save power
  return (uint8_t)level;
}

// ── BLE ───────────────────────────────────────────────────────────────────────
BLEService        dataService(SERVICE_UUID);
BLECharacteristic dataChar(DATA_CHAR_UUID);   // NOTIFY — data stream
BLECharacteristic cmdChar(CMD_CHAR_UUID);     // WRITE + NOTIFY — command channel

// Command characteristic write callback
void cmdWriteCallback(uint16_t conn_handle, BLECharacteristic* chr,
                      uint8_t* data, uint16_t len) {
  // Null-terminate and parse command string
  char cmd[32] = {0};
  memcpy(cmd, data, min((int)len, 31));

  Serial.print("CMD received: ");
  Serial.println(cmd);

  if (strcmp(cmd, "GET_BATTERY") == 0) {
    uint8_t level = readBatteryPercent();
    char response[20];
    snprintf(response, sizeof(response), "BATTERY:%d", level);
    cmdChar.notify((uint8_t*)response, strlen(response));
    Serial.print("Battery: ");
    Serial.print(level);
    Serial.println("%");
  }
  else if (strcmp(cmd, "STOP") == 0) {
    // placeholder — stop sampling logic goes here
    cmdChar.notify((uint8_t*)"OK:STOP", 7);
  }
  else if (strcmp(cmd, "START") == 0) {
    cmdChar.notify((uint8_t*)"OK:START", 8);
  }
  else {
    cmdChar.notify((uint8_t*)"ERR:UNKNOWN", 11);
  }
}

void connect_callback(uint16_t conn_handle) {
  BLEConnection* conn = Bluefruit.Connection(conn_handle);
  Serial.println("Central connected");

  // Negotiate faster connection for throughput
  conn->requestPHY();
  conn->requestDataLengthUpdate();
  conn->requestMtuExchange(247);
  conn->requestConnectionParameter(6);  // 7.5 ms interval
  delay(1000);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  Serial.print("Disconnected, reason: 0x");
  Serial.println(reason, HEX);
}

void startAdv() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(dataService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void setup() {
  Serial.begin(1000000);
  delay(500);  // do not block on serial -- XIAO hangs forever if no USB monitor
  Serial.println("Starting XIAO Peripheral -- 19ch 1kHz batched");

  // Init phase accumulators
  for (int i = 0; i < NUM_CHANNELS; i++) {
    phase[i]  = 0.0f;
    freqHz[i] = 1.0f + i;   // ch0=1Hz ... ch18=19Hz
  }

  // BLE init
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.autoConnLed(true);  // blue LED blinks while advertising
  Bluefruit.setTxPower(8);
  Bluefruit.setName("XIAO_PERIPHERAL");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
  Bluefruit.Periph.setConnInterval(6, 12);  // 7.5-15 ms

  // ── Data service + characteristic ─────────────────────────────────────────
  dataService.begin();

  dataChar.setProperties(CHR_PROPS_NOTIFY);
  dataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dataChar.setFixedLen(PACKET_TOTAL_BYTES);
  dataChar.begin();

  // ── Command characteristic ─────────────────────────────────────────────────
  cmdChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
  cmdChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  cmdChar.setWriteCallback(cmdWriteCallback);
  cmdChar.begin();

  startAdv();

  Serial.print("Advertising -- packet: ");
  Serial.print(PACKET_TOTAL_BYTES);
  Serial.print(" bytes, notify rate: ");
  Serial.print(SAMPLE_RATE_HZ / BATCH_SIZE);
  Serial.println(" Hz");
}

void loop() {
  if (!Bluefruit.connected()) {
    delay(10);
    return;
  }

  // Acquire one time step at 1kHz
  unsigned long now = micros();
  if (now - previousMicros < intervalMicros) return;
  previousMicros = now;

  int timeStepOffset = PACKET_HEADER_BYTES + batchIndex * NUM_CHANNELS * 2;

  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    phase[ch] += 2.0f * PI * freqHz[ch] / SAMPLE_RATE_HZ;
    if (phase[ch] > 2.0f * PI) phase[ch] -= 2.0f * PI;

    uint16_t sample = ADC_MID + (int16_t)(ADC_AMPLITUDE * sin(phase[ch]));

    int offset = timeStepOffset + ch * 2;
    packetBuffer[offset]     = sample & 0xFF;
    packetBuffer[offset + 1] = (sample >> 8) & 0xFF;
  }

  batchIndex++;

  if (batchIndex >= BATCH_SIZE) {
    packetBuffer[0] = packetId & 0xFF;
    packetBuffer[1] = (packetId >> 8) & 0xFF;

    dataChar.notify(packetBuffer, PACKET_TOTAL_BYTES);

    packetId++;
    batchIndex = 0;
  }
}
