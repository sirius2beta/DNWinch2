#include <HardwareSerial.h>

#define SerialAqua Serial2


const uint8_t aquaWakeupCmd[4] = {0x01, 0x0D, 0xC1, 0xE5};
const uint8_t specialRegisterCmd[8] = {0x01, 0x03, 0x25, 0x23, 0x00, 0x01, 0x7E, 0xCC};
const uint8_t aquaSpecialCmd[11] = {0x01, 0x10, 0x25, 0x24, 0x00, 0x01, 0x02, 0x07, 0xD0, 0xD7, 0xDA};

bool waked = false;

// Aqua 各感測器寄存器讀取指令
const uint8_t aquaCmds[][8] = {
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
const size_t aquaCmdCount = sizeof(aquaCmds) / sizeof(aquaCmds[0]);


void wakeupAqua() {

  uint8_t buffer[64];
  int rlen = 0;

  Serial.println("🔄 Sending wakeup command repeatedly...");

  // Step 1: Keep sending wakeup command until any response is received
  while (true) {
    SerialAqua.write(aquaWakeupCmd, sizeof(aquaWakeupCmd));

    unsigned long start = millis();
    bool gotResponse = false;

    // Wait up to 1000ms for any response
    while (millis() - start < 1000) {
      if (SerialAqua.available() > 0) {
        while (SerialAqua.available()){
          if (rlen < sizeof(buffer)) {
            buffer[rlen++] = SerialAqua.read();
          }
        }
        gotResponse = true;
        break;
      }
    }

    if (gotResponse) {
      Serial.println("✅ Aqua responded. Proceeding to special command...");
      break;
    }

    delay(200); // Optional delay between retries to avoid spamming
  }
  delay(500); // Optional delay between retries to avoid spamming
  // Step 2: Send special command
  start = millis();
  
  SerialAqua.write(specialRegisterCmd, sizeof(aquaSpecialCmd));
  while (millis() - start < 2000) {
    rlen = 0;
    if (SerialAqua.available()) {
      if (rlen < sizeof(buffer)) {
        buffer[rlen++] = SerialAqua.read();
      }
    }
  }
  if(rlen >0){
    aquaSpecialCmd[7] = buffer[3];
    aquaSpecialCmd[8] = buffer[4];
  }

  
  SerialAqua.write(aquaSpecialCmd, sizeof(aquaSpecialCmd));

  // Step 3: Wait up to 2 seconds and read full response
  start = millis();
  rlen = 0;
  while (millis() - start < 2000) {
    if (SerialAqua.available()) {
      if (rlen < sizeof(buffer)) {
        buffer[rlen++] = SerialAqua.read();
      }
    }
  }
  if(rlen > 0){
    if(buffer[1] == 0x10){
      Serial.println("✅ set special register complete");
    }
  }
  waked = true;

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
  if (index >= aquaCmdCount) return false;

  const uint8_t *cmd = aquaCmds[index];
  SerialAqua.write(cmd, 8);
  unsigned long start = millis();
  uint8_t resp[19];
  int rlen = 0;

  while (millis() - start < 6000) {
    if (SerialAqua.available()) {
      resp[rlen++] = SerialAqua.read();
      if (rlen == 19) break;
    }
  }
  if (rlen != 19) {
    Serial.println("⚠️ Aqua timeout / response error");
    return false;
  }

  value = bytesToFloat(&resp[3]);
  return true;
}

void setup() {
  Serial.begin(115200);
  SerialAqua.begin(19200, SERIAL_8E1,16, 17); // RX = GPIO7, TX = GPIO6 (or D4/D5 if mapped that way)


  while (!Serial);
  while (!SerialAqua);

  Serial.println("🔧 Setup done");
}

void loop() {
  Serial.println("🔄 Sending wakeup command repeatedly...");
  if(! waked){
    wakeupAqua();
  }
  
  unsigned long start = millis();
  for (size_t i = 0; i < aquaCmdCount; ++i) {
    float value = 0.0;
    if (readAquaRegister(i, value)) {
      Serial.print("Sensor Index ");
      Serial.print(i);
      Serial.print(" → Value: ");
      Serial.println(value, 4); // 4 decimal places
    } else {
      Serial.print("Sensor Index ");
      Serial.print(i);
      Serial.println(" → ❌ Failed to read");
    }
    //delay(100); // wait before next sensor read
  }
  unsigned long operation_time = millis()-start;

  Serial.print("⌛ Loop operation time: ");
  Serial.print(operation_time);
  Serial.println(" ms");
}