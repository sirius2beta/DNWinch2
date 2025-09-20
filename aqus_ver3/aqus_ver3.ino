#define MODBUSRTU_TIMEOUT 6000
#include <ModbusRTU.h>
#define MODBUSRTU_TIMEOUT 6000  // 必須放在 include 前面

// =========== Aqua部分=========
#define AQUA_ID 1                
#define AQUA_SENSOR_COUNT 21     
#define REG_COUNT 7              
#define TIMEOUT_MS 6000 

         
// UART2 for RS485
#define UART1_TX 17
#define UART1_RX 16

const uint16_t aquaSensorAddr[AQUA_SENSOR_COUNT] = {
    5450, // 0x154A 溫度
    5457, // 0x1551 壓力
    5464, // 0x1558 深度
    5471, // 0x155F 水位
    5478, // 0x1566 表面高程
    5506, // 0x1582 實際導電率
    5513, // 0x1589 特定導電率
    5520, // 0x1590 電阻率
    5527, // 0x1597 鹽度
    5534, // 0x159E 總溶解固體
    5541, // 0x15A5 水密度
    5555, // 0x15B3 大氣壓力
    5562, // 0x15BA pH值
    5569, // 0x15C1 pH毫伏
    5576, // 0x15C8 ORP
    5583, // 0x15CF 溶解氧濃度
    5590, // 0x15D6 溶解氧飽和度
    5618, // 0x15F2 濁度
    5653, // 0x1615 氧分壓
    5667, // 0x1623 外部電壓
    5674  // 0x162A 電池容量
};

uint16_t sensorBuf[AQUA_SENSOR_COUNT][REG_COUNT];

ModbusRTU mbAqua;

// ------------------- Poll Control -------------------
volatile int currentSensor = 0;
unsigned long requestStart = 0;
unsigned long roundStart = 0;

bool waitingResponse = false;

// ------------------- Callback -------------------
bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  // 不管 228，只要有資料就存
  if (event == Modbus::EX_SUCCESS) {
    waitingResponse = false;
    uint32_t tmp = ((uint32_t)sensorBuf[currentSensor][0] << 16) | sensorBuf[currentSensor][1];
    float value = 0;
    memcpy(&value, &tmp, sizeof(float));
    Serial.printf("sensor index: %d\n", currentSensor);
    Serial.printf("raw:");
    for(int i=0;i<REG_COUNT;i++){
        Serial.printf(" %X ", sensorBuf[currentSensor][i]);
    }
    Serial.println();
    Serial.printf("value: %f\n", value);
    currentSensor = (currentSensor + 1) % AQUA_SENSOR_COUNT;
  }else{
    Serial.printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!error code: %X \n", event);
    waitingResponse = false;
    uint32_t tmp = ((uint32_t)sensorBuf[currentSensor][0] << 16) | sensorBuf[currentSensor][1];
    float value = 0;
    memcpy(&value, &tmp, sizeof(float));
    Serial.printf("sensor index: %d\n", currentSensor);
    Serial.printf("raw:");
    for(int i=0;i<REG_COUNT;i++){
        Serial.printf(" %X ", sensorBuf[currentSensor][i]);
    }
    Serial.println();
    Serial.printf("value: %f\n", value);
    currentSensor = (currentSensor + 1) % AQUA_SENSOR_COUNT;
  }

  return true; // 告訴 Modbus 回應已處理
}

// ------------------- Send Next Request -------------------
void sendNextRequest() {
  //計算讀取完21個感測器大約需要多少時間
  if(currentSensor == 0){
      unsigned long now = millis();
      Serial.printf("Round time: %lu ms\n", now - roundStart);
      roundStart = now;
  }
  if (!mbAqua.readHreg(AQUA_ID,
                       aquaSensorAddr[currentSensor],
                       sensorBuf[currentSensor],
                       REG_COUNT,
                       cbRead)) {
    Serial.printf("[Sensor %u] Failed to start request\n", aquaSensorAddr[currentSensor]);
  }
  requestStart = millis();
  waitingResponse = true;
}
// ------------------- Task -------------------
void aquaTask(void *pv) {
  for (;;) {
    
    mbAqua.task();
    
    if (!waitingResponse) {
      sendNextRequest();  // 沒在等 → 送下一個
    } else if (millis() - requestStart > TIMEOUT_MS) {
      Serial.printf("[Sensor %u] timeout, skip\n", aquaSensorAddr[currentSensor]);
      waitingResponse = false;  // 強制結束等待
      currentSensor = (currentSensor + 1) % AQUA_SENSOR_COUNT;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);

  Serial1.begin(19200, SERIAL_8E1, UART1_RX, UART1_TX); 
  mbAqua.begin(&Serial1);
  mbAqua.master();

  xTaskCreatePinnedToCore(aquaTask, "aquaTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  // 主迴圈不用管，task 自己跑
}
