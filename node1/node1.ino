#include <Arduino.h>
#include <AccelStepper.h>

// UART2 設定
#define UART2_TX 17
#define UART2_RX 16
HardwareSerial RS485(2);

// 馬達
#define STEP_PIN 14
#define DIR_PIN  12
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int motorMaxSpeed = 1000;
int motorAcc      = 200;

// 狀態
int32_t currentStep = 0;
int32_t currentTension = 0;
bool motorRunning = false;

// -----------------------------
// CRC8 計算 (MSB-first, poly 0x07)
// -----------------------------
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// -----------------------------
// 封包接收與解析
// -----------------------------
#define PKT_BUF_SIZE 128
uint8_t pktBuf[PKT_BUF_SIZE];
int pktIndex = 0;
int expectedLen = -1;

void processPacket(uint8_t *payload, int len) {
  if (len < 1) return;
  uint8_t cmd = payload[0];

  switch (cmd) {
    case 0x00: // reset highest point
      stepper.setCurrentPosition(0);
      Serial.println("[CMD] Reset highest point");
      break;

    case 0x01: // set speed & acc
      if (len >= 5) {
        int16_t spd = (int16_t)((payload[1] << 8) | payload[2]);
        int16_t acc = (int16_t)((payload[3] << 8) | payload[4]);
        motorMaxSpeed = spd;
        motorAcc = acc;
        stepper.setMaxSpeed(motorMaxSpeed);
        stepper.setAcceleration(motorAcc);
        Serial.printf("[CMD] Set speed=%d acc=%d\n", spd, acc);
      }
      break;

    case 0x02: // set step
      if (len >= 5) {
        int32_t step = ((int32_t)payload[1] << 24) | ((int32_t)payload[2] << 16) | ((int32_t)payload[3] << 8) | (int32_t)payload[4];
        stepper.moveTo(step);
        motorRunning = true;
        Serial.printf("[CMD] Move to step=%ld\n", step);
      }
      break;

    case 0x03: // stop
      stepper.setCurrentPosition(stepper.currentPosition());
      motorRunning = false;
      Serial.println("[CMD] Stop");
      break;

    case 0x04: { // get status
      uint8_t resp[1 + 4 + 4 + 1]; // cmd + step + tension + running
      resp[0] = 0x04;
      int32_t step = stepper.currentPosition();
      resp[1] = (step >> 24) & 0xFF;
      resp[2] = (step >> 16) & 0xFF;
      resp[3] = (step >> 8) & 0xFF;
      resp[4] = step & 0xFF;
      resp[5] = (currentTension >> 24) & 0xFF;
      resp[6] = (currentTension >> 16) & 0xFF;
      resp[7] = (currentTension >> 8) & 0xFF;
      resp[8] = currentTension & 0xFF;
      resp[9] = motorRunning ? 1 : 0;

      // 封包回傳
      // 注意：這邊使用修正過的 sendPacket
      sendPacket(resp, sizeof(resp));
      Serial.println("Reply 0x04 sent");
      break;
    }

    case 0x05: // set tension threshold
      if (len >= 5) {
        int32_t tension = ((int32_t)payload[1] << 24) | ((int32_t)payload[2] << 16) | ((int32_t)payload[3] << 8) | (int32_t)payload[4];
        Serial.printf("[CMD] Set tension threshold=%ld\n", tension);
      }
      break;

    case 0x06: // reset pos = 0
      stepper.setCurrentPosition(0);
      Serial.println("[CMD] Reset pos=0");
      break;

    case 0x07: // get aqua cached data
      Serial.println("[CMD] Get Aqua data");
      break;

    default:
      Serial.printf("[CMD] Unknown cmd=%02X\n", cmd);
      break;
  }
}

// -----------------------------
// 發送封包 (已修正)
// -----------------------------
void sendPacket(uint8_t *payload, int len) {
  if (len < 0 || len > (PKT_BUF_SIZE - 3)) return;
  uint8_t buf[PKT_BUF_SIZE];
  buf[0] = 0xAA;
  buf[1] = (uint8_t)len;
  memcpy(&buf[2], payload, len);
  uint8_t crcVal = crc8(&buf[1], len + 1); // LEN + PAYLOAD
  buf[2 + len] = crcVal;

  // Debug: 列印要送出的 raw bytes
  Serial.print("📤 Send: ");
  for (int i = 0; i < len + 3; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();

  // 正確寫出 header + len + payload + crc (len + 3 bytes)
  RS485.write(buf, len + 3);
  RS485.flush();
}

// -----------------------------
// UART 接收處理 (含 debug)
// -----------------------------
void handleUART() {
  while (RS485.available()) {
    uint8_t b = RS485.read();

    // 如果一開始不是 header，跳過直到遇到 0xAA
    if (pktIndex == 0 && b != 0xAA) {
      continue;
    }

    // 防止溢位
    if (pktIndex >= PKT_BUF_SIZE) {
      pktIndex = 0;
      expectedLen = -1;
      Serial.println("pktBuf overflow, reset");
      continue;
    }

    pktBuf[pktIndex++] = b;

    if (pktIndex == 2) {
      expectedLen = pktBuf[1];
      // sanity check
      if (expectedLen < 0 || expectedLen > (PKT_BUF_SIZE - 3)) {
        Serial.printf("Invalid len %d, reset\n", expectedLen);
        pktIndex = 0;
        expectedLen = -1;
        continue;
      }
    }

    if (expectedLen >= 0 && pktIndex == expectedLen + 3) {
      // calc CRC over LEN + PAYLOAD
      uint8_t calcCrc = crc8(&pktBuf[1], expectedLen + 1);
      uint8_t recvCrc = pktBuf[pktIndex - 1];

      // debug: 列印收到的 raw bytes
      Serial.print("📥 Recv: ");
      for (int i = 0; i < pktIndex; i++) Serial.printf("%02X ", pktBuf[i]);
      Serial.println();
      Serial.printf("calcCrc=%02X recvCrc=%02X\n", calcCrc, recvCrc);

      if (calcCrc == recvCrc) {
        processPacket(&pktBuf[2], expectedLen);
      } else {
        Serial.println("CRC error");
      }
      pktIndex = 0;
      expectedLen = -1;
    }
  }
}

void setup() {
  Serial.begin(115200);
  RS485.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);

  stepper.setMaxSpeed(motorMaxSpeed);
  stepper.setAcceleration(motorAcc);
}

void loop() {
  if(Serial.available()){
    str = Serial.readStringUntil('\n');
    
  }
  handleUART();
  stepper.run();
}
