#include <Arduino.h>
#include <ModbusRTU.h>
#include <AccelStepper.h>
#include "HX711.h"

// Slave address
#define SLAVE_ID 0x12

// UART2 for RS485
#define UART2_TX 17
#define UART2_RX 16
#define DE_RE    4   // RS485 driver DE/RE control pin (自己選一個 IO 腳位)

// 馬達
#define STEP_PIN 14
#define DIR_PIN  12
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

int motorMaxSpeed = 1000;
int motorAcc      = 200;

// 秤重感測器
#define HX711_DT  26
#define HX711_SCK 27
HX711 scale;

int32_t currentTension = 0;
int32_t tensionThreshold = 0;
bool motorRunning = false;

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
  IR_COUNT
};

enum {
  COIL_RUN = 0,       // Coil 0: Motor Run/Stop
  COIL_COUNT
};

// callback functions
uint16_t cbSetCurPos(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_CURPOS_H);
  uint16_t low  = val;
  int32_t newPos = ((int32_t)high << 16) | low;
  stepper.setCurrentPosition(newPos);
  Serial.printf("[DEBUG] HR_CURPOS_H=%04X, HR_CURPOS_L=%04X\n", high, low);
  Serial.printf("[CMD] Master set CurrentStep = %ld\n", newPos);
  return val;
}

// Coil 控制 馬達緊急煞車
uint16_t cbMotorRun(TRegister* reg, uint16_t val) {
  if(!COIL_BOOL(val)){
    stepper.stop();
    Serial.println("[CMD] Motor STOP");
    motorRunning = false;
  }
  return val;

}


uint16_t cbSetTargetPos(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_MOV_H);
  uint16_t low  = val;
  int32_t newStep = ((int32_t)high << 16) | low;
  stepper.moveTo(newStep);
  motorRunning = true;
  Serial.printf("[CMD] MoveTo step=%ld\n", newStep);
  return val;
}



uint16_t cbSetSpeed(TRegister* reg, uint16_t val) {
  stepper.setMaxSpeed(val);
  Serial.printf("[CMD] Set speed=%d\n", motorMaxSpeed);
  return val;
}

uint16_t cbSetAcc(TRegister* reg, uint16_t val) {
  stepper.setAcceleration(val);
  Serial.printf("[CMD] Set acc=%d\n", motorAcc);
  return val;
}

uint16_t cbSetThr(TRegister* reg, uint16_t val) {
  uint16_t high = mb.Hreg(HR_THR_H);
  uint16_t low  = val;
  tensionThreshold = ((int32_t)high << 16) | low;
  Serial.printf("[CMD] Set tensionThreshold=%d\n", tensionThreshold);
  return val;
}

void setup() {
  Serial.begin(115200);

  // 初始化 RS485
  MainBus.begin(19200, SERIAL_8E1, UART2_RX, UART2_TX);
  mb.begin(&MainBus, DE_RE);
  mb.slave(SLAVE_ID);

  // 配置暫存器
  mb.addHreg(HR_SPEED, motorMaxSpeed);
  mb.addHreg(HR_ACC, motorAcc);
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
  mb.addCoil(COIL_RUN, true);

  // 當 master 寫 Hreg 的值時，觸發 cbSetCurPos
  mb.onSetHreg(HR_CURPOS_L, cbSetCurPos);
  mb.onSetHreg(HR_MOV_L, cbSetTargetPos);
  mb.onSetCoil(COIL_RUN, cbMotorRun);
  mb.onSetHreg(HR_SPEED, cbSetSpeed);
  mb.onSetHreg(HR_ACC, cbSetAcc);
  mb.onSetHreg(HR_THR_L, cbSetThr);



  // 馬達初始化
  stepper.setMaxSpeed(motorMaxSpeed);
  stepper.setAcceleration(motorAcc);

  // HX711
  scale.begin(HX711_DT, HX711_SCK);
  Serial.println("HX711 ready");
}

void loop() {
  mb.task();

  // 更新感測數據
  if (scale.is_ready()) {
    currentTension = (int32_t)scale.get_units();
  }
  //如果 tension<tension threshold，不可下降
  if(stepper.distanceToGo()>0 && currentTension<tensionThreshold){
    stepper.stop();
    motorRunning = false;
    Serial.println("[SAFETY] Tension below threshold, Motor STOP");
  }
  // 更新 Input Register
  mb.Ireg(IR_TEN_H, (currentTension >> 16) & 0xFFFF);
  mb.Ireg(IR_TEN_L, currentTension & 0xFFFF);
  mb.Ireg(IR_POS_H, (stepper.currentPosition() >> 16) & 0xFFFF);
  mb.Ireg(IR_POS_L, stepper.currentPosition() & 0xFFFF);
  mb.Ireg(IR_RUN_STATE, stepper.isRunning() ? 0xFF : 0x00);
  
  stepper.run();

}
