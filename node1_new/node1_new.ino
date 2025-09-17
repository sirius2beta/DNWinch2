#include <Arduino.h>
#include <AccelStepper.h>
#include "HX711.h"

// UART1 連接aqua
#define UART1_TX 25
#define UART1_RX 33

// UART2 連接master
#define UART2_TX 17
#define UART2_RX 16
#define slave_addr 0x12

HardwareSerial Aqua(1);
HardwareSerial MainBus(2);

// mutex
SemaphoreHandle_t node0Mutex;

#define TIMEOUT_MS 100

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
// 狀態
int32_t currentTension = 0;
int32_t tensionThreshold = 0;
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
// 處理例外狀況
// -----------------------------
void sendExceptionResponse(uint8_t functionCode, uint8_t exceptionCode) {
    uint8_t response[5];
    response[0] = slave_addr;              // Slave address
    response[1] = functionCode | 0x80;     // Function code + 0x80
    response[2] = exceptionCode;           // Exception code

    uint16_t crc = crc16(response, 3);
    response[3] = crc & 0xFF;
    response[4] = (crc >> 8) & 0xFF;

    MainBus.write(response, 5);
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

      case 0x04: { // read input registers
        if (len < 6) {
          sendExceptionResponse(0x04, 0x03); // Illegal data value
          break;
        }
      
        uint16_t startAddr = (payload[2] << 8) | payload[3];
        uint16_t quantity = (payload[4] << 8) | payload[5];
      
        if (quantity == 0 || quantity > 10) {
          sendExceptionResponse(0x04, 0x03); // Illegal data value
          break;
        }
      
        Serial.printf("[CMD] Read input registers, addr=%d, qty=%d\n", startAddr, quantity);
      
        uint8_t response[256];
        response[0] = slave_addr;
        response[1] = 0x04;
        response[2] = quantity * 2; // data bytes count
      
        for (uint16_t i = 0; i < quantity; i++) {
          uint16_t addr = startAddr + i;
          uint16_t val = 0;
      
          switch (addr) {
            case 40006:
              val = (currentTension >> 16) & 0xFFFF; // high 16 bits
              break;
            case 40007:
              val = currentTension & 0xFFFF;         // low 16 bits
              break;
            case 40008:
              val = (stepper.currentPosition() >> 16) & 0xFFFF;
              break;
            case 40009:
              val =  stepper.currentPosition() & 0xFFFF;         // low 16 bits
              break;
      
            default:
              val = 0xFFFF; // unknown or unsupported register
              break;
          }
      
          response[3 + i * 2] = (val >> 8) & 0xFF;
          response[4 + i * 2] = val & 0xFF;
        }
      
        uint16_t crc = crc16(response, 3 + quantity * 2);
        response[3 + quantity * 2] = crc & 0xFF;
        response[4 + quantity * 2] = (crc >> 8) & 0xFF;
      
        MainBus.write(response, 5 + quantity * 2);
        break;
      }

      case 0x05:{ // write single coil
        uint16_t coil = (payload[2] << 8) | payload[3];
        uint16_t state = (payload[4] << 8) | payload[5];
        if(coil == 0){
          if(state == 0){
            stepper.setCurrentPosition(stepper.currentPosition());
            motorRunning = false;
            Serial.println("[CMD] Stop");
            uint8_t response[8];
            memcpy(response, payload, 6);
            uint16_t crc = crc16(response, 6); // 根據前6個 bytes 計算 CRC
            response[6] = crc & 0xFF;
            response[7] = (crc >> 8) & 0xFF;
            MainBus.write(response, 8);

          }else{
            Serial.printf("[ERROR] invalid state %1d\n", state);
            sendExceptionResponse(0x05, 0x03); // Function 0x05, Exception 0x03: Illegal Data Value
          }
          
        }else if(coil == 1){
          Serial.printf("[ERROR] invalid coil %1d\n", coil);
          sendExceptionResponse(0x05, 0x03); // Function 0x05, Exception 0x03: Illegal Data Value
        }
        break;
      }
      case 0x06:{ // write single register
        uint16_t addr = (payload[2] << 8) | payload[3];
        uint16_t content = (payload[4] << 8) | payload[5];
        if(addr == 40000){
          motorMaxSpeed = content;
          Serial.printf("[CMD] Set speed=%d \n", motorMaxSpeed);
          uint8_t response[8];
          memcpy(response, payload, 6);
        
          uint16_t crc = crc16(response, 6); // 根據前6個 bytes 計算 CRC
          response[6] = crc & 0xFF;
          response[7] = (crc >> 8) & 0xFF;
          MainBus.write(response, 8);
        }else if(addr == 40001){
          motorAcc = content;
          Serial.printf("[CMD] Set acc=%d\n", motorAcc);
          uint8_t response[8];
          memcpy(response, payload, 6);
        
          uint16_t crc = crc16(response, 6); // 根據前6個 bytes 計算 CRC
          response[6] = crc & 0xFF;
          response[7] = (crc >> 8) & 0xFF;
          MainBus.write(response, 8);
        }
        break;
      }
      case 0x10: {
        uint16_t start_addr = (payload[2] << 8) | payload[3];
        uint16_t reg_count = (payload[4] << 8) | payload[5];
        uint8_t byte_count = payload[6];
    
        Serial.printf("Write Multiple Registers:\n");
        Serial.printf("  Slave: %d\n", slave_addr);
        Serial.printf("  Start Address: %d\n", start_addr);
        Serial.printf("  Register Count: %d\n", reg_count);
        Serial.printf("  Byte Count: %d\n", byte_count);
    
        Serial.print("  Values: ");
        for (int i = 0; i < byte_count; i++) {
          Serial.printf("%02X ", payload[7 + i]);
        }
        Serial.println();
        if(start_addr == 40002){
          if(byte_count == 4){
            int32_t tension = ((int32_t)payload[7] << 24) | ((int32_t)payload[8] << 16) | ((int32_t)payload[9] << 8) | (int32_t)payload[10];
            tensionThreshold = tension;
            Serial.printf("[CMD] Set tension threshold =%1d\n", tension);
          }
        }else if(start_addr == 40004){
          if(byte_count == 4){
            int32_t step = ((int32_t)payload[7] << 24) | ((int32_t)payload[8] << 16) | ((int32_t)payload[9] << 8) | (int32_t)payload[10];
            stepper.moveTo(step);
            motorRunning = true;
            Serial.printf("[CMD] Move to step=%ld\n", step);
          }
        }else if(start_addr == 40008){
          if(byte_count == 4){
            int32_t step = ((int32_t)payload[7] << 24) | ((int32_t)payload[8] << 16) | ((int32_t)payload[9] << 8) | (int32_t)payload[10];
            stepper.setCurrentPosition(step);
            Serial.printf("[CMD] Set current step=%1d\n", step);
          }
        }
        
        uint8_t response[8];
        response[0] = slave_addr;
        response[1] = 0x10;
        response[2] = (start_addr >> 8) & 0xFF;
        response[3] = start_addr & 0xFF;
        response[4] = (reg_count >> 8) & 0xFF;
        response[5] = reg_count & 0xFF;
      
        uint16_t crc = crc16(response, 6);
        response[6] = crc & 0xFF;
        response[7] = (crc >> 8) & 0xFF;
      
        // 送出回應
        MainBus.write(response, 8);
    
        // 這邊可以寫入你的 register 處理邏輯...
        break;
      }
  
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
unsigned long buff2lastRecvTime = 0;
// -----------------------------
// UART 接收處理 (含 debug)
// -----------------------------
void handleUART() {
  // 接收資料
  while (MainBus.available()) {
    if (buff2Len >= sizeof(buff2)) {
      Serial.println("Buffer overflow, clearing buffer");
      buff2Len = 0;
      while (MainBus.available()) MainBus.read(); // 清空 UART 緩衝
      return;
    }
    uint8_t b = MainBus.read();
    buff2[buff2Len++] = b;
    buff2lastRecvTime = millis();
  }

  // 如果 buffer 有資料，但超時了，清掉
  if (buff2Len > 0 && (millis() - buff2lastRecvTime) > TIMEOUT_MS) {
    Serial.println("Timeout exceeded, clearing buffer");
    buff2Len = 0;
  }

  // 只有在資料至少 4 bytes 才開始處理
  if (buff2Len >= 4) {
    

    uint8_t function_code = buff2[1];
    int expected_len = 0;

    if (function_code == 0x04) {
      expected_len = 8;
    } else if (function_code == 0x05) {
      expected_len = 8;
    } else if (function_code == 0x06) {
      expected_len = 8;
    } else if (function_code == 0x10) {
      if (buff2Len >= 7) {
        uint8_t byte_count = buff2[6];
        expected_len = 7 + byte_count + 2;
      } else {
        // 等待更多資料
        return;
      }
    } else {
      Serial.printf("Unsupported function code: 0x%02X, clearing buffer\n", function_code);
      // 印出封包內容（hex格式）
      Serial.print("Packet: ");
      for (int i = 0; i < buff2Len; i++) {
        Serial.printf("%02X ", buff2[i]);
      }
      Serial.println();
      buff2Len = 0;
      while (MainBus.available()) MainBus.read();
      return;
    }

    if (buff2Len < expected_len) {
      // 還沒收到完整資料，等待下一輪
      return;
    }

    // CRC 驗證
    uint16_t crc_calc = crc16(buff2, expected_len - 2);
    uint16_t crc_recv = buff2[expected_len - 2] | (buff2[expected_len - 1] << 8);

    if (crc_calc != crc_recv) {
      Serial.println("CRC mismatch, clearing buffer");
      buff2Len = 0;
      while (MainBus.available()) MainBus.read();
      return;
    }
    if (buff2[0] != slave_addr) {
      Serial.printf("Invalid slave addr: %02X, clearing buffer\n", buff2[0]);
      buff2Len = 0;
      while (MainBus.available()) MainBus.read();
      return;
    }

    // 成功接收完整且 CRC 正確的封包
    Serial.println("Received valid packet");
    processPacket(buff2, expected_len);
    buff2Len = 0;
  }
}

// -----------------------------
// Node0 UART2 command parsing task
// -----------------------------
void node0Task(void *pv) {
  (void)pv;
  for (;;) {
    handleUART();
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
