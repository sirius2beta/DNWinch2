
#define SONAR_SW_PIN 23
#define Rx 16
#define Tx 17
#define slave_addr 0x11

HardwareSerial mySerial(2);

const uint8_t statusCode[2] = {0x11, 0x11};


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

void setup() {
  Serial.begin(115200);
  mySerial.begin(19200, SERIAL_8E1, Rx, Tx);
  pinMode(SONAR_SW_PIN, OUTPUT);
}

#define TIMEOUT_MS 100

void loop() {
  static uint8_t buff[32];
  static int len = 0;
  static unsigned long lastRecvTime = 0;

  // 接收資料
  while (mySerial.available() && len < sizeof(buff)) {
    uint8_t b = mySerial.read();
    buff[len++] = b;
    Serial.printf("Got byte: 0x%02X\n", b);
    lastRecvTime = millis();
  }

  // 如果 buffer 有資料，但超時了，清掉
  if (len > 0 && (millis() - lastRecvTime) > TIMEOUT_MS) {
    Serial.println("Timeout exceeded, clearing buffer");
    len = 0;
  }

  // 只有在資料至少 4 bytes 才開始處理
  if (len >= 4) {
    if (buff[0] != slave_addr) {
      len = 0;
      while (mySerial.available()) mySerial.read();
      return;
    }

    uint8_t function_code = buff[1];
    int expected_len = 0;

    if (function_code == 0x11) {
      expected_len = 4;
    } else if (function_code == 0x05) {
      expected_len = 8;
    } else {
      Serial.printf("Unsupported function code: 0x%02X\n", function_code);
      len = 0;
      while (mySerial.available()) mySerial.read();
      return;
    }

    // 如果資料還沒收到完整
    if (len < expected_len) {
      return;
    }

    // 計算 CRC
    uint16_t crc_calc = crc16(buff, expected_len - 2);
    uint16_t crc_recv = buff[expected_len - 2] | (buff[expected_len - 1] << 8);

    if (crc_calc != crc_recv) {
      Serial.println("X CRC mismatch");
      Serial.print("Packet: ");
      for (int i = 0; i < expected_len; i++) {
        Serial.printf("%02X ", buff[i]);
      }
      Serial.println();
      len = 0;
      while (mySerial.available()) mySerial.read();
      return;
    }

    // 如果走到這，代表封包正確
    Serial.println("got data");

    // 處理指令
    if (function_code == 0x11) {
      uint8_t reply[4];
      reply[0] = slave_addr;
      reply[1] = 0x11;
      uint16_t c = crc16(reply, 2);
      reply[2] = c & 0xFF;
      reply[3] = (c >> 8) & 0xFF;
      mySerial.write(reply, sizeof(reply));
      Serial.print("Reply: ");
      for (int i = 0; i < 4; i++) Serial.printf("%02X ", reply[i]);
      Serial.println();
    } else if (function_code == 0x05) {
      uint16_t coil = (buff[2] << 8) | buff[3];
      uint16_t state = (buff[4] << 8) | buff[5];
      Serial.printf("Coil=0x%04X, state=0x%04X\n", coil, state);
      
      if (coil == 1) { 
        if(state == 0x0000){ 
          Serial.println("Turn sonar off"); 
          digitalWrite(SONAR_SW_PIN, LOW); 
        }else if (state == 0xFF00) { 
          Serial.println("Turn sonar on"); 
          digitalWrite(SONAR_SW_PIN, HIGH); 
        }else{ 
          Serial.printf("Invalid state value: 0x%04X\n", state); 
        }
      }
      mySerial.write(buff, expected_len);  // echo
      Serial.print("Echoed: ");
      for (int i = 0; i < expected_len; i++) Serial.printf("%02X ", buff[i]);
      Serial.println();
    }

    // 清 buffer
    len = 0;
    while (mySerial.available()) mySerial.read();  // 清串口緩衝區
  }
}
