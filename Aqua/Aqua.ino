#include <HardwareSerial.h>

//UART
#define UART1_TX 23
#define UART1_RX 22
HardwareSerial AquaBus(1);

#define AQUA_TIMEOUT 6000
#define AQUA_SENSOR_COUNT 21     

const uint8_t aquaWakeupCmd[4] = {0x01, 0x11, 0xC0, 0x2C};
const uint8_t specialRegisterCmd[8] = {0x01, 0x03, 0x25, 0x23, 0x00, 0x01, 0x7E, 0xCC};
uint8_t aquaSpecialCmd[11] = {0x01, 0x10, 0x25, 0x24, 0x00, 0x01, 0x02, 0x07, 0xD0, 0xD7, 0xDA};

bool aqua_waked = false;

volatile int currentSensor = 0;

// Aqua 各感測器寄存器讀取指令
const uint8_t aquaCmds[AQUA_SENSOR_COUNT][8] = {
    {0x01,0x03,0x15,0x4A,0x00,0x07,0x21,0xD2}, // 0 溫度
    {0x01,0x03,0x15,0x51,0x00,0x07,0x51,0xD5}, // 1 壓力
    {0x01,0x03,0x15,0x58,0x00,0x07,0x81,0xD7}, // 2 深度
    {0x01,0x03,0x15,0x5F,0x00,0x07,0x30,0x16}, // 3 水位
    {0x01,0x03,0x15,0x66,0x00,0x07,0xE0,0x1B}, // 4 表面高程
    {0x01,0x03,0x15,0x82,0x00,0x07,0xA0,0x2C}, // 5 實際導電率
    {0x01,0x03,0x15,0x89,0x00,0x07,0xD1,0xEE}, // 6 特定導電率
    {0x01,0x03,0x15,0x90,0x00,0x07,0x00,0x29}, // 7 電阻率
    {0x01,0x03,0x15,0x97,0x00,0x07,0xB1,0xE8}, // 8 鹽度
    {0x01,0x03,0x15,0x9E,0x00,0x07,0x61,0xEA}, // 9 總溶解固體
    {0x01,0x03,0x15,0xA5,0x00,0x07,0x10,0x27}, // 10 水密度
    {0x01,0x03,0x15,0xB3,0x00,0x07,0xF1,0xE3}, // 11 大氣壓力
    {0x01,0x03,0x15,0xBA,0x00,0x07,0x21,0xE1}, // 12 pH值
    {0x01,0x03,0x15,0xC1,0x00,0x07,0x51,0xF8}, // 13 pH毫伏
    {0x01,0x03,0x15,0xC8,0x00,0x07,0x81,0xFA}, // 14 ORP
    {0x01,0x03,0x15,0xCF,0x00,0x07,0x30,0x3B}, // 15 溶解氧濃度
    {0x01,0x03,0x15,0xD6,0x00,0x07,0xE1,0xFC}, // 16 溶解氧飽和度
    {0x01,0x03,0x15,0xF2,0x00,0x07,0xA1,0xF7}, // 17 濁度
    {0x01,0x03,0x16,0x15,0x00,0x07,0x11,0x84}, // 18 氧分壓
    {0x01,0x03,0x16,0x23,0x00,0x07,0xF1,0x8A}, // 19 外部電壓
    {0x01,0x03,0x16,0x2A,0x00,0x07,0x21,0x88}  // 20 電池容量
};

uint8_t AquaIregOffset = 20;
float AquaInputBuff[AQUA_SENSOR_COUNT];

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

void wakeupAqua() {

  uint8_t buffer[64];
  int rlen = 0;
  unsigned long start = millis();

  Serial.println("🔄 Sending wakeup command repeatedly...");

  // Step 1: Keep sending wakeup command until any response is received
  while (true) {
    AquaBus.write(aquaWakeupCmd, sizeof(aquaWakeupCmd));

    start = millis();
    bool gotResponse = false;

    // Wait up to 1000ms for any response
    while (millis() - start < 1000) {
      if (AquaBus.available() > 0) {
        while (AquaBus.available()){
          if (rlen < sizeof(buffer)) {
            buffer[rlen++] = AquaBus.read();
          }
        }
        gotResponse = true;
        break;
      }
    }

    if (gotResponse) {
      Serial.println("✅ Aqua responded. Proceeding to special command...");
      delay(200); // Optional delay between retries to avoid spamming
      // 跳過 special code!!!
      while (AquaBus.available()) {
        AquaBus.read(); // 丟掉上一次的資料
      }
      aqua_waked = true;
      return;
    }
  }
}





float bytesToFloat(uint8_t* bytes) {
  union {
    uint8_t b[4];
    float f;
  } u;
  u.b[0] = bytes[3];
  u.b[1] = bytes[2];
  u.b[2] = bytes[1];
  u.b[3] = bytes[0];
  return u.f;
}

bool readAquaRegister(size_t index, float &value) {
  if (index >= AQUA_SENSOR_COUNT) return false;

  const uint8_t *cmd = aquaCmds[index];
  AquaBus.write(cmd, 8);
  unsigned long start = millis();
  uint8_t resp[19];
  int rlen = 0;

  while (millis() - start < AQUA_TIMEOUT) {
    if (AquaBus.available()) {
      resp[rlen++] = AquaBus.read();
      if (rlen == 19) break;
    }
  }
  if (rlen != 19) {
    Serial.println("[AquaTask] Aqua timeout / response error");
    return false;
  }
  value = bytesToFloat(&resp[3]);
  AquaInputBuff[currentSensor] = value;
  return true;
}

// ------------------- Task -------------------
void aquaTask(void *pv) {
  for (;;) {
    Serial.println("[AquaTask] Sending wakeup command repeatedly...");
    if(!aqua_waked){
      wakeupAqua();
    }
    unsigned long start = millis();
    for (;;) {
      float value = 0.0;
      if (readAquaRegister(currentSensor, value)) {
        Serial.print("[AquaTask] Sensor Index ");
        Serial.print(currentSensor);
        Serial.print(" → Value: ");
        Serial.println(value, 4); // 4 decimal places
      } else {
        Serial.print("[AquaTask] Sensor Index ");
        Serial.print(currentSensor);
        Serial.println(" → ❌ Failed to read");
      }
      currentSensor = (currentSensor + 1) % AQUA_SENSOR_COUNT;
      //delay(100); // wait before next sensor read
      if(currentSensor == 0){
        unsigned long operation_time = millis()-start;
        start = millis();
        Serial.print("[AquaTask] Loop operation time: ");
        Serial.print(operation_time);
        Serial.println(" ms");
        // print aqua input buffer raw data in hex
        Serial.println("Aqua Input Buffer (Hex):");
        for(int i=0;i<AQUA_SENSOR_COUNT;i++){
          Serial.printf("%f ", AquaInputBuff[i]);
          if((i+1)%8==0) Serial.println();
        }
      }
    }
    
  }
}


void setup() {
  Serial.begin(115200);
  AquaBus.begin(19200, SERIAL_8E1, UART1_RX, UART1_TX);

  while (!Serial);
  xTaskCreatePinnedToCore(aquaTask, "aquaTask", 4096, NULL, 1, NULL, 1);
  Serial.println("Aqua Setup done");
}

void loop() {
  
}
