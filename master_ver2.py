from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import time
import struct

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
        runningState = 0x00
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
        
    def get_aqua_data(self): # 待建置
        # read input registers from 20, all 21 sensors, convert to 32-bit float
        try:
            result = self.client.read_input_registers(20, count=42, slave=self.winch_addr)
            if result.isError():
                print("[Node2] Get Aqua Data failed")
                return None
            else:
                regs = result.registers
                aqua_data = []
                for i in range(0, 42, 2):
                    high = regs[i]
                    low = regs[i+1]
                    combined = (high << 16) | low
                    float_value = struct.unpack('>f', struct.pack('>I', combined))[0]
                    aqua_data.append(float_value)
                print(f"[Node2] Aqua Data: {aqua_data}")
                return aqua_data
        except ModbusException as e:
            print(f"[Node2] Get Aqua Data failed: {e}")
            return None

    def basic_manuver(self):
        self.startMotor(True)
        time.sleep(0.5)
        self.setMaxSpeed(2000)
        time.sleep(0.5)
        self.setAcc(500)
        time.sleep(0.5)
        self.setTensionThreshold(-1500)
        time.sleep(0.5)
        self.getCurrentStep()
        time.sleep(0.5)
        self.setCurrentStep(0)
        time.sleep(1)
        
        count = 0
        target = -10000
        while True:
            if count ==9:
                count = 0
                target = -target
                self.startMotor(False)
                time.sleep(0.5)
            self.setTargetStep(target)
            self.getStatus()
            self.get_aqua_data()
            count += 1
            time.sleep(1)
    def testSonar(self):
        while True:
            self.setSonar(True)
            time.sleep(5)
            self.setSonar(False)
            time.sleep(5)
if __name__ == "__main__":
    master = RS485MasterPymodbus(port="COM13")
    try:
        master.testSonar()
    
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        master.close()

