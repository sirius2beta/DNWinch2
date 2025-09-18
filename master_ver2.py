from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time


class RS485MasterPymodbus:
    def __init__(self, port: str):
        self.client = ModbusSerialClient(
            port=port,
            baudrate=19200,
            parity='E',
            stopbits=1,
            bytesize=8,
            timeout=1
        )
        self.client.connect()
        self.sonar_addr = 0x11
        self.winch_addr = 0x12

    def close(self):
        self.client.close()

    def setSonar(self, on: bool):
        try:
            coil_addr = 0  # Coil 位址是從 0 起算
            self.client.write_coil(coil_addr, on, slave=self.sonar_addr)
            print(f"[Node1] Set sonar {'ON' if on else 'OFF'}")
        except ModbusException as e:
            print(f"[Node1] Set sonar failed: {e}")
    def startMotor(self, on: bool):
        try:
            coil_addr = 0  # Coil 位址是從 0 起算
            self.client.write_coil(coil_addr, on, slave=self.winch_addr)
            print(f"[Node2] Set motor {'ON' if on else 'OFF'}")
        except ModbusException as e:
            print(f"[Node2] Set motor failed: {e}")

    def setMaxSpeed(self, speed):
        try:
            # ESP 定義在 Hreg[0] → 40001
            self.client.write_register(0, speed, slave=self.winch_addr)
            print(f"[Node2] Set MaxSpeed = {speed}")
        except ModbusException as e:
            print(f"[Node2] Set MaxSpeed failed: {e}")

    def setAcc(self, acc):
        try:
            # ESP 定義在 Hreg[1] → 40002
            self.client.write_register(1, acc, slave=self.winch_addr)
            print(f"[Node2] Set Acc = {acc}")
        except ModbusException as e:
            print(f"[Node2] Set Acc failed: {e}")
    def setTensionThreshold(self, tension):
        try:
            high = (tension >> 16) & 0xFFFF
            low = tension & 0xFFFF
            # ESP 定義在 Hreg[2] 和 Hreg[3] → 40003 和 40004
            self.client.write_registers(2, [high, low], slave=self.winch_addr)
            print(f"[Node2] Set Tension Threshold = {tension}")
        except ModbusException as e:
            print(f"[Node2] Set Tension Threshold failed: {e}")

    def getCurrentStep(self):
        try:
            # ESP32 端 IR_POS_H = 6 → 對應到 30007
            # pymodbus 呼叫時直接用 6
            result = self.client.read_input_registers(8, count=2, slave=self.winch_addr)

            if result.isError():
                print("[Node2] Get Current Step failed")
            else:
                high, low = result.registers
                step = (high << 16) | low
                if step & 0x80000000:  # 補 signed
                    step -= 0x100000000
                print(f"[Node2] Current Step = {step}")
                return step
        except ModbusException as e:
            print(f"[Node2] Get Current Step failed: {e}")
        return None

    def setCurrentStep(self, step):
        try:
            high = (step >> 16) & 0xFFFF
            low = step & 0xFFFF
            # ESP 定義在 Hreg[8] → 40009
            self.client.write_registers(6, [high, low], slave=self.winch_addr)
            print(f"[Node2] Set Current Step = {step}")
        except ModbusException as e:
            print(f"[Node2] Set Current Step failed: {e}")

    def setTargetStep(self, step):
        try:
            high = (step >> 16) & 0xFFFF
            low = step & 0xFFFF
            # ESP 定義在 Hreg[4] → 40005
            self.client.write_registers(4, [high, low], slave=self.winch_addr)
            print(f"[Node2] Set Target Step = {step}")
        except ModbusException as e:
            print(f"[Node2] Set Target Step failed: {e}")
    def getStatus(self):
        # read 30006~30009 (currentTension(32bit), currentStep(32bit))
        try:
            result = self.client.read_input_registers(6, count=5, slave=self.winch_addr)
            if result.isError():
                print("[Node2] Get Status failed")
                return None
            else:
                regs = result.registers
                tension = (regs[0] << 16) | regs[1]
                step = (regs[2] << 16) | regs[3]
                if step & 0x80000000:  # 補 signed
                    step -= 0x100000000
                    runningState = regs[4]
                print(f"[Node2] Status - Tension: {tension}, Step: {step}, RunningState: {'Running' if runningState==0xFF else 'Stopped'}")
                tension = (regs[0] << 16) | regs[1]
                step    = (regs[2] << 16) | regs[3]
                
                return tension, step
        except ModbusException as e:
            print(f"[Node2] Get Status failed: {e}")
            return None


if __name__ == "__main__":
    master = RS485MasterPymodbus(port="COM13")
    try:
        master.startMotor(True)
        time.sleep(1)
        master.setMaxSpeed(1000)
        time.sleep(1)
        master.setAcc(100)
        time.sleep(1)
        master.setTensionThreshold(1500)
        time.sleep(1)
        master.getCurrentStep()
        time.sleep(1)
        master.setCurrentStep(0)
        time.sleep(1)
        master.setTargetStep(-10000)
        time.sleep(1)
        #master.startMotor(False)
        time.sleep(1)
        while True:
            master.getStatus()
            time.sleep(1)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        master.close()
