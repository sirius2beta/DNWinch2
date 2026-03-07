#include <Arduino.h>
#include <ModbusRTU.h>
#include <AccelStepper.h>
#include "HX711.h"

// Slave address
#define SLAVE_ID 0x12
#define AQUA_ID  0x01

// UART2 for RS485
#define UART2_TX 17
#define UART2_RX 16

// Buttons
#define UPBTN    21
#define DOWNBTN  18
#define STOPBTN  19
#define RESETBTN 13
#define TENSIONBTN 32

bool isAutoCompensating = false; // 新增：是否處於自動補償上捲狀態
unsigned long lastBtnTime = 0;
const unsigned long debounceMs = 120;
bool lastResetBtnState = HIGH;

// 馬達
#define STEP_PIN 14
#define DIR_PIN  12
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

enum WinchState {
  STOPPED,
  RUN_UP,
  RUN_DOWN
};

WinchState winchState = STOPPED;

// 秤重感測器
#define HX711_DT  26
#define HX711_SCK 27
HX711 scale;

int32_t currentTension = 0;
int32_t tensionThreshold = 0;

// tension state
bool tensionHigh = false;

// Modbus
HardwareSerial MainBus(2);
ModbusRTU mb;

// 暫存器定義
enum {
  HR_SPEED = 0,       // 40000: MaxSpeed
  HR_ACC,             // 40001: Acceleration
  HR_THR_H,           // 40002: tensionThreshold 高16位
  HR_THR_L,           // 40003: tensionThreshold 低16位
  HR_MOV_H,           // 40004: moveTo step 高16位
  HR_MOV_L,           // 40005: moveTo step 低16位
  HR_CURPOS_H,        // 40006: set current pos 高16位
  HR_CURPOS_L,        // 40007: set current pos 低16位
  HR_COUNT
  
};



enum {
  IR_TEN_H = 6,       // 30006: currentTension 高16位
  IR_TEN_L,           // 30007: currentTension 低16位
  IR_POS_H,           // 30008: currentPosition 高16位
  IR_POS_L,           // 30009: currentPosition 低16位
  IR_RUN_STATE,      // 30010: Motor Run State (0xFF=running, 0x00=stopped)
  IR_TENSION_BTN     //30011
};

enum {
  COIL_RUN = 0,       // Coil 0: Motor Run/Stop
  COIL_COUNT
};

static inline int32_t combineToInt32(uint16_t high, uint16_t low){
  uint32_t u = ((uint32_t)high << 16) | (uint32_t)low;
  return (int32_t)u;
}


// callback functions
uint16_t cbSetCurPos(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_CURPOS_H);
  uint16_t low  = val;
  int32_t newPos = combineToInt32(high, low);
  stepper.setCurrentPosition(newPos);
  Serial.printf("[CMD] Master set CurrentStep = %ld\n", newPos);
  return val;
}

// Coil 控制 馬達緊急煞車
uint16_t cbMotorRun(TRegister* reg, uint16_t val) {
  if(!COIL_BOOL(val)){
    stepper.setCurrentPosition(stepper.currentPosition());
    Serial.println("[CMD] Motor STOP");
  }
  return val;

}


uint16_t cbSetTargetPos(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_MOV_H);
  uint16_t low  = val;
  int32_t newStep = combineToInt32(high, low);
  if (digitalRead(RESETBTN) == LOW && newStep < stepper.currentPosition()) {
    Serial.println("[MODBUS BLOCK] Reset button pressed, ignore UP command");
    // 可以選擇不執行 moveTo，或強制設回當前位置
    return val; 
  }
  stepper.moveTo(newStep);
  Serial.printf("[CMD] MoveTo step=%ld\n", newStep);
  return val;
}

uint16_t cbSetSpeed(TRegister* reg, uint16_t val) {
  stepper.setMaxSpeed(val);
  Serial.printf("[CMD] Set speed=%d\n", val);
  return val;
}

uint16_t cbSetAcc(TRegister* reg, uint16_t val) {
  stepper.setAcceleration(val);
  Serial.printf("[CMD] Set acc=%d\n", val);
  return val;
}

uint16_t cbSetThr(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_THR_H);
  uint16_t low  = val;
  tensionThreshold = combineToInt32(high, low);
  Serial.printf("[CMD] Set tensionThreshold=%d\n", tensionThreshold);
  return val;
}

// Node0 task
void node0Task(void *pv) {
  (void)pv;
  for (;;) {
    mb.task();
    vTaskDelay(10 / portTICK_PERIOD_MS); // 讓出 CPU
  }
}



// =================== Aqua task ===========================
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

// 讀取模式
enum AquaReadMode {
  READ_ALL = 0,
  READ_DEPTH = 1
};

AquaReadMode currentReadMode = READ_ALL;

// 暫存器定義
enum {
  IR_AQ_READMODE = 19, // 40019: Aqua 讀取模式
  IR_AQ_TEMP_H = 20, // 40020: Aqua 溫度 高16位
  IR_AQ_TEMP_L,     // 40021: Aqua 溫度 低16位
  IR_AQ_PRES_H,     // 40022: Aqua 壓力 高16位
  IR_AQ_PRES_L,     // 40023: Aqua 壓力 低16位
  IR_AQ_DEPTH_H,    // 40024: Aqua 深度 高16位
  IR_AQ_DEPTH_L,    // 40025: Aqua 深度 低16
  // ... 其他感測器暫存器依序類推，最高到 40061 (20個感測器 * 2個reg/感測器 = 40個reg)
};

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
// Protect AquaInputBuff with mutex
SemaphoreHandle_t aquaMutex = NULL;

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
      Serial.print("."); // Indicate waiting
      delay(100);
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
  uint8_t tmp[4];
  tmp[0] = bytes[3];
  tmp[1] = bytes[2];
  tmp[2] = bytes[1];
  tmp[3] = bytes[0];
  float f;
  memcpy(&f, tmp, sizeof(f));
  return f;
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
  if (xSemaphoreTake(aquaMutex, (TickType_t)10 / portTICK_PERIOD_MS) == pdTRUE) {
    AquaInputBuff[currentSensor] = value;
    xSemaphoreGive(aquaMutex);
  } else {
    // fallback: still set volatile (not ideal) but we try
    AquaInputBuff[currentSensor] = value;
  }
  return true;
}

// ------------------- Task -------------------
void aquaTask(void *pv) {
  
  Serial.println("[AquaTask] Sending wakeup command repeatedly...");
  if(!aqua_waked){
    wakeupAqua();
  }
  unsigned long start = millis();
  for (;;) {
    if(currentReadMode == READ_DEPTH){
      currentSensor = 2; // depth sensor index
      float value = 0.0;
      readAquaRegister(currentSensor, value);
      Serial.print("[AquaTask] Depth Value: ");
      Serial.println(value, 4); // 4 decimal places
      vTaskDelay(10 / portTICK_PERIOD_MS); // wait before next read
    }else{
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
        if (xSemaphoreTake(aquaMutex, (TickType_t)10 / portTICK_PERIOD_MS) == pdTRUE){ 
          for(int i=0;i<AQUA_SENSOR_COUNT;i++){
            Serial.printf("%f ", AquaInputBuff[i]);
            if((i+1)%8==0) Serial.println();
            vTaskDelay(10 / portTICK_PERIOD_MS); // 讓出 CPU
          }
          xSemaphoreGive(aquaMutex);
        }
        
      }
    }
  }
}
  

void setup() {
  Serial.begin(115200);

  // 初始化 Mainbus
  MainBus.begin(19200, SERIAL_8E1, UART2_RX, UART2_TX);
  mb.begin(&MainBus);
  mb.slave(SLAVE_ID);

  // 初始化 AquaBus
  AquaBus.begin(19200, SERIAL_8E1, UART1_RX, UART1_TX);

  // Create mutex for aqua buffer
  aquaMutex = xSemaphoreCreateMutex();
  if (aquaMutex == NULL) {
    Serial.println("⚠️ Failed to create aqua mutex");
  }

  // 配置暫存器
  mb.addHreg(HR_SPEED, 2000);
  mb.addHreg(HR_ACC, 1000);
  mb.addHreg(HR_THR_H, 0);
  mb.addHreg(HR_THR_L, 0);
  mb.addHreg(HR_MOV_H, 0);
  mb.addHreg(HR_MOV_L, 0);
  mb.addHreg(HR_CURPOS_H, 0);
  mb.addHreg(HR_CURPOS_L, 0);

  mb.addIreg(IR_TEN_H, 0);
  mb.addIreg(IR_TEN_L, 0);
  mb.addIreg(IR_POS_H, 0);
  mb.addIreg(IR_POS_L, 0);
  mb.addIreg(IR_RUN_STATE, 0);
  mb.addIreg(IR_TENSION_BTN, 0);
  // 新增 Aqua 感測器暫存器, offset 20, 每個sensor有兩個reg, 分別為高16位和低16位
  mb.addIreg(IR_AQ_READMODE, 0);
  for(int i=0;i<AQUA_SENSOR_COUNT;i++){
    //高位
    mb.addIreg(AquaIregOffset + i*2, 0);
    //低位
    mb.addIreg(AquaIregOffset + i*2 + 1, 0);
  }

  mb.addCoil(COIL_RUN, true);

  // 當 master 寫 Hreg 的值時，觸發 cbSetCurPos
  mb.onSetHreg(HR_CURPOS_L, cbSetCurPos);
  mb.onSetHreg(HR_MOV_L, cbSetTargetPos);
  mb.onSetCoil(COIL_RUN, cbMotorRun);
  mb.onSetHreg(HR_SPEED, cbSetSpeed);
  mb.onSetHreg(HR_ACC, cbSetAcc);
  mb.onSetHreg(HR_THR_L, cbSetThr);

  // Buttons
  pinMode(UPBTN, INPUT_PULLUP);
  pinMode(DOWNBTN, INPUT_PULLUP);
  pinMode(STOPBTN, INPUT_PULLUP);
  pinMode(RESETBTN, INPUT_PULLUP);
  pinMode(TENSIONBTN, INPUT_PULLUP);

  // 馬達初始化
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  Serial.println("HX711 ready");
  xTaskCreatePinnedToCore(node0Task, "node0Task", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(aquaTask, "aquaTask", 4096, NULL, 1, NULL, 1);
  Serial.println("Aqua Setup done");

}


void handleButtons() {
  unsigned long now = millis();
  if (now - lastBtnTime < debounceMs) return;
  bool currentResetBtnState = digitalRead(RESETBTN);
  if (digitalRead(UPBTN) == LOW) {   // 按下 UP
    lastBtnTime = now;
    if(winchState != WinchState::RUN_UP){
      if(winchState == WinchState::RUN_DOWN){
        stepper.setCurrentPosition(stepper.currentPosition());
        Serial.println("[BUTTON] Motor STOP");
      }
      Serial.println("[BUTTON] Motor run UP");
      stepper.move(-1000000);
    }
  } else if (digitalRead(DOWNBTN) == LOW) {  // 按下 DOWN
    lastBtnTime = now;
    if(winchState != WinchState::RUN_DOWN){
      if(winchState == WinchState::RUN_UP){
        stepper.setCurrentPosition(stepper.currentPosition());
        Serial.println("[BUTTON] Motor STOP");
      }
      Serial.println("[BUTTON] Motor run DOWN");
      stepper.move(1000000);
    }
  } else if (digitalRead(STOPBTN) == LOW) {  // 按下 STOP
    lastBtnTime = now;
    if(winchState != WinchState::STOPPED){
      stepper.setCurrentPosition(stepper.currentPosition()); // 硬停
      Serial.println("[BUTTON] Motor STOP");
    }
  } else if (currentResetBtnState == LOW && lastResetBtnState == HIGH) { // 避免連續觸發，只有在按鈕從未按下變為按下的瞬間才觸發
    lastBtnTime = now;
    if(stepper.currentPosition()!=0){
      Serial.println("[BUTTON] RESET position to 0");
      stepper.setCurrentPosition(0);
    }
  }
  if(digitalRead(TENSIONBTN) == LOW) {  // 按下 TENSION
    tensionHigh = true;
  }else{
    tensionHigh = false;
  }

  lastResetBtnState = currentResetBtnState;
}

void loop() {
  handleButtons();

  // 更新感測數據
  if (scale.is_ready()) {
    currentTension = (int32_t)scale.read();
  }

  if(stepper.distanceToGo()!=0){
    if(stepper.targetPosition()>stepper.currentPosition()){
      winchState = WinchState::RUN_DOWN;
    } else {
      winchState = WinchState::RUN_UP;
    }
    currentReadMode = READ_DEPTH; // 運行時只讀取深度感測器
  } else {
    winchState = WinchState::STOPPED;
    currentReadMode = READ_ALL; // 停止時讀取全部感測器
    if(isAutoCompensating){
      Serial.println("[SAFETY] Auto Retract Finished (Tension Restored)");
      isAutoCompensating = false; // 新增：是否處於自動補償上捲狀態
    }
    
  }
  //如果 tension<tension threshold，不可下降
  if(winchState == WinchState::RUN_DOWN && !tensionHigh){
    //stepper.setCurrentPosition(stepper.currentPosition());
    //Serial.println("[SAFETY] Tension below threshold, Motor STOP");
    //isAutoCompensating = true; 
    //stepper.move(-5000);
    
  }


  // 更新 Input Register
  uint32_t tens_raw = (uint32_t)currentTension;
  mb.Ireg(IR_TEN_H, (uint16_t)((tens_raw >> 16) & 0xFFFF));
  mb.Ireg(IR_TEN_L, (uint16_t)(tens_raw & 0xFFFF));
  int32_t pos = stepper.currentPosition();
  uint32_t pos_raw = (uint32_t)pos;
  mb.Ireg(IR_POS_H, (uint16_t)((pos_raw >> 16) & 0xFFFF));
  mb.Ireg(IR_POS_L, (uint16_t)(pos_raw & 0xFFFF));
  mb.Ireg(IR_RUN_STATE, (stepper.distanceToGo() != 0) ? 0xFF : 0x00);
  mb.Ireg(IR_TENSION_BTN, tensionHigh);


  // 更新所有的 Aqua 感測器暫存器 from float buffer using safe memcpy
  mb.Ireg(IR_AQ_READMODE, (uint16_t)currentReadMode);
  if (xSemaphoreTake(aquaMutex, (TickType_t)5 / portTICK_PERIOD_MS) == pdTRUE) {
    for (int i = 0; i < AQUA_SENSOR_COUNT; i++) {
      uint32_t raw;
      memcpy(&raw, &AquaInputBuff[i], sizeof(raw)); // reinterpret float bits as uint32
      mb.Ireg(AquaIregOffset + i * 2, (uint16_t)((raw >> 16) & 0xFFFF));
      mb.Ireg(AquaIregOffset + i * 2 + 1, (uint16_t)(raw & 0xFFFF));
    }
    xSemaphoreGive(aquaMutex);
  } else {
    //Serial.println("⚠️ Aqua mutex busy, skipping update");
    // If mutex busy, skip updating this cycle (to avoid blocking)
  }

  
  stepper.run();
  vTaskDelay(1 / portTICK_PERIOD_MS);
}
