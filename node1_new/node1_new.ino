#include <Arduino.h>
#include <AccelStepper.h>
#include "HX711.h"

// UART1 連接aqua
#define UART1_TX 1
#define UART1_RX 22
// UART2 連接master
#define UART2_TX 17
#define UART2_RX 16
#define slave_addr 0x12

HardwareSerial Aqua(2);
HardwareSerial MainBus(2)

// mutex
SemaphoreHandle_t node0Mutex;


// 馬達
#define STEP_PIN 14
#define DIR_PIN  12
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int motorMaxSpeed = 1000;
int motorAcc      = 200;


// 秤重感測器
#define HX711_DT  26   // DOUT   
#define HX711_SCK 27 // or other suitable GPIO  
HX711 scale;

// 狀態
int32_t currentStep = 0;
int32_t currentTension = 0;
bool motorRunning = false;

// -----------------------------
// CRC16 計算
// -----------------------------
uint16_t crc16(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc = crc >> 1;
        }
    }
    return crc;
}


// -----------------------------
// 封包接收與解析
// -----------------------------
#define PKT_BUF_SIZE 128
uint8_t buff1[PKT_BUF_SIZE]; //UART1 buffer
int buff1Len = 0;
uint8_t buff2[PKT_BUF_SIZE]; //UART2 buffer
int buff2Len = 0;
int expectedLen = -1;

void processPacket(uint8_t *payload, int len) {
  if (len < 1) return;
  if (payload[0] != slave_addr) return;
  uint8_t cmd = payload[1];
  if (xSemaphoreTake(node0Mutex, portMAX_DELAY)) {
    switch (cmd) {
      case 0x20: // reset highest point
        stepper.setCurrentPosition(0);
        Serial.println("[CMD] Reset highest point");
        break;
  
      case 0x21: // set speed & acc
        if (len >= 6) {
          int16_t spd = (int16_t)((payload[2] << 8) | payload[3]);
          int16_t acc = (int16_t)((payload[4] << 8) | payload[5]);
          motorMaxSpeed = spd;
          motorAcc = acc;
          stepper.setMaxSpeed(motorMaxSpeed);
          stepper.setAcceleration(motorAcc);
          Serial.printf("[CMD] Set speed=%d acc=%d\n", spd, acc);
        }
        break;
  
      case 0x22: // set step
        if (len >= 6) {
          int32_t step = ((int32_t)payload[2] << 24) | ((int32_t)payload[3] << 16) | ((int32_t)payload[4] << 8) | (int32_t)payload[5];
          stepper.moveTo(step);
          motorRunning = true;
          Serial.printf("[CMD] Move to step=%ld\n", step);
        }
        break;
  
      case 0x023: // stop
        stepper.setCurrentPosition(stepper.currentPosition());
        motorRunning = false;
        Serial.println("[CMD] Stop");
        break;
  
      case 0x24: { // get status
        uint8_t resp[1 + 1 + 4 + 4 + 1]; // addr + cmd + step + tension + running
        resp[0] = slave_addr;
        resp[1] = 0x04;
        int32_t step = stepper.currentPosition();
        resp[2] = (step >> 24) & 0xFF;
        resp[3] = (step >> 16) & 0xFF;
        resp[4] = (step >> 8) & 0xFF;
        resp[5] = step & 0xFF;
        resp[6] = (currentTension >> 24) & 0xFF;
        resp[7] = (currentTension >> 16) & 0xFF;
        resp[8] = (currentTension >> 8) & 0xFF;
        resp[9] = currentTension & 0xFF;
        resp[10] = motorRunning ? 1 : 0;
  
        // 封包回傳
        sendPacket(resp, sizeof(resp));
        Serial.println("Reply 0x04 sent");
        break;
      }
  
      case 0x25: // set tension threshold
        if (len >= 6) {
          int32_t tension = ((int32_t)payload[2] << 24) | ((int32_t)payload[3] << 16) | ((int32_t)payload[4] << 8) | (int32_t)payload[5];
          Serial.printf("[CMD] Set tension threshold=%ld\n", tension);
        }
        break;
  
      case 0x26: // reset pos = 0
        stepper.setCurrentPosition(0);
        Serial.println("[CMD] Reset pos=0");
        break;
  
      case 0x27: // get aqua cached data
        Serial.println("[CMD] Get Aqua data");
        //replyAquaSnapshotToNode0();
        Serial.println("Node0: CMD 0x07 sent aqua snapshot");
        break;
  
        
  
      default:
        Serial.printf("[CMD] Unknown cmd=%02X\n", cmd);
        break;
    }
    xSemaphoreGive(node0Mutex);
  }
}

// -----------------------------
// 發送封包
// -----------------------------
void sendPacket(uint8_t *payload, int len) {
  if (len < 0 || len > (PKT_BUF_SIZE - 3)) return;
  uint8_t buf[PKT_BUF_SIZE];
  memcpy(&buf[0], payload, len);
  uint16_t crc_calc = crc16(&buf[0], len); // LEN + PAYLOAD
  buf[len] = crc_calc & 0xFF;        // CRC Lo
  buf[1 + len] = (crc_calc >> 8) & 0xFF; // CRC Hi

  // Debug: 列印要送出的 raw bytes
  Serial.print("📤 Send: ");
  for (int i = 0; i < len + 2; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();

  // 正確寫出 header + len + payload + crc (len + 3 bytes)
  MainBus.write(buf, len + 2);
}

// -----------------------------
// UART 接收處理 (含 debug)
// -----------------------------
void handleUART() {
  while (MainBus.available()) {
    buff2[buff2Len++] = MainBus.read();
  }
  if(buff2Len > 0){
    if(buff2[0] == slave_addr){
      Serial.println("got data");
    }
    //validate CRC16
    if(buff2Len >= 4){ // 
      uint16_t crc_calc = crc16(buff2, buff2Len-2);
      uint16_t crc_recv = buff2[buff2Len-2] | (buff2[buff2Len-1] << 8);

      if(crc_calc != crc_recv){
        Serial.println("X check CRC16 failed");
        Serial.print("Recv: ");
        for(int i=0; i<buff2Len; i++){
          Serial.print(buff2[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      } else{
        processPacket(buff2, buff2Len);
      }
      buff2Len = 0; // 在成功處理後清除
    }
  }
  

}

// -----------------------------
// Node0 UART2 command parsing task
// -----------------------------
void node0Task(void *pv) {
  (void)pv;
  for (;;) {
    handleUART();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void setup() {
  Serial.begin(115200);
  Aqua.begin(19200, SERIAL_8E1, UART1_RX, UART1_TX);
  MainBus.begin(19200, SERIAL_8E1, UART2_RX, UART2_TX);

  stepper.setMaxSpeed(motorMaxSpeed);
  stepper.setAcceleration(motorAcc);

  scale.begin(HX711_DT, HX711_SCK);
  Serial.println("HX711 ready");
  
  node0Mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(node0Task, "node0Task", 4096, NULL, 1, NULL, 1);
}

void loop() {
  if (xSemaphoreTake(node0Mutex, portMAX_DELAY)) {
    if (scale.is_ready()) {
      currentTension = (int32_t)scale.get_units();
    }
    stepper.run();
    xSemaphoreGive(node0Mutex);
  }
}
