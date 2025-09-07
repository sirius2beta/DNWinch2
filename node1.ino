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
// CRC8 計算
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
#define PKT_BUF_SIZE 64
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
        int16_t spd = (payload[1] << 8) | payload[2];
        int16_t acc = (payload[3] << 8) | payload[4];
        motorMaxSpeed = spd;
        motorAcc = acc;
        stepper.setMaxSpeed(motorMaxSpeed);
        stepper.setAcceleration(motorAcc);
        Serial.printf("[CMD] Set speed=%d acc=%d\n", spd, acc);
      }
      break;

    case 0x02: // set step
      if (len >= 5) {
        int32_t step = (int32_t) ((payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4]);
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
      sendPacket(resp, sizeof(resp));
      break;
    }

    case 0x05: // set tension threshold
      if (len >= 5) {
        int32_t tension = (int32_t)((payload[1] << 24) | (payload[2] << 16) | (payload[3] << 8) | payload[4]);
        Serial.printf("[CMD] Set tension threshold=%ld\n", tension);
      }
      break;

    case 0x06: // reset pos = 0
      stepper.setCurrentPosition(0);
      Serial.println("[CMD] Reset pos=0");
      break;

    case 0x07: // get aqua cached data
      // TODO: Aqua 緩存讀取 (跟前面寫的 Aqua polling 任務整合)
      Serial.println("[CMD] Get Aqua data");
      break;

    default:
      Serial.printf("[CMD] Unknown cmd=%02X\n", cmd);
      break;
  }
}

// -----------------------------
// 發送封包
// -----------------------------
void sendPacket(uint8_t *payload, int len) {
  uint8_t header = 0xAA;
  uint8_t length = len;
  uint8_t crc = crc8(&length, 1);
  crc = crc8(payload, len ^ crc); // 錯誤？應該逐步算，我可以再改

  // 正確計算應該：
  uint8_t buf[64];
  buf[0] = 0xAA;
  buf[1] = length;
  memcpy(&buf[2], payload, len);
  uint8_t crcVal = crc8(&buf[1], length + 1); // LEN + PAYLOAD
  buf[2 + len] = crcVal;

  RS485.write(buf, 3 + len - 1); // header + len + payload + crc
}

// -----------------------------
// UART 接收處理
// -----------------------------
void handleUART() {
  while (RS485.available()) {
    uint8_t b = RS485.read();

    if (pktIndex == 0 && b != 0xAA) {
      continue; // 等到 header
    }

    pktBuf[pktIndex++] = b;

    if (pktIndex == 2) {
      expectedLen = pktBuf[1];
    }

    if (expectedLen >= 0 && pktIndex == expectedLen + 3) {
      uint8_t calcCrc = crc8(&pktBuf[1], expectedLen + 1);
      uint8_t recvCrc = pktBuf[pktIndex - 1];
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
  handleUART();
  stepper.run();
}
