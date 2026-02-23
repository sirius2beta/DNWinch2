#include <Arduino.h>
#include <ModbusRTU.h>
#include <AccelStepper.h>
#include "HX711.h"

// Slave address
#define SLAVE_ID 0x11

// UART2 for RS485
#define UART2_TX 17
#define UART2_RX 16

#define SONAR_PWR_PIN 23

// Modbus
HardwareSerial MainBus(2);
ModbusRTU mb;

enum {
  COIL_SONAR_PWR = 0,       // Coil 0: Sonar Power Control
  COIL_COUNT
};

// Coil 控制 Sonar 電源
uint16_t cbSonarPWR(TRegister* reg, uint16_t val) {
  digitalWrite(SONAR_PWR_PIN, COIL_BOOL(val) ? HIGH : LOW); 
  return val;
}

void setup() {
  Serial.begin(115200);

  // 初始化 RS485 (Mainbus)
  MainBus.begin(19200, SERIAL_8E1, UART2_RX, UART2_TX);
  mb.begin(&MainBus);
  mb.slave(SLAVE_ID);

  // 初始化 Sonar 電源控制腳位
  pinMode(SONAR_PWR_PIN, OUTPUT);
  digitalWrite(SONAR_PWR_PIN, LOW); // 預設關閉

  // Modbus Coil
  mb.addCoil(COIL_SONAR_PWR, false);
  mb.onSetCoil(COIL_SONAR_PWR, cbSonarPWR);
}

void loop() {
  mb.task();
  yield();
}
