import serial
import time
import struct

# ===== CRC16 計算 (多項式 0xA001) =====
def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


# ===== 發送封包 (自動加 CRC) =====
def send_packet(ser: serial.Serial, addr: int, function: int, payload: bytes = b""):
    frame = bytes([addr, function]) + payload
    crc = crc16(frame)
    frame += bytes([crc & 0xFF, (crc >> 8) & 0xFF])  # Lo, Hi
    print(">> Send:", frame.hex(" ").upper())
    ser.write(frame)


def expected_length(rx_bytes: bytes) -> int | None:
    """根據功能碼和資料長度，推估預期的封包總長度"""
    if len(rx_bytes) < 2:
        return None  # 太短，無法判斷

    function = rx_bytes[1]

    # Exception 回應: 固定 5 bytes
    if function & 0x80:
        return 5

    # 回應封包格式根據功能碼
    if function == 0x03 or function == 0x04:
        # [Addr][Func][Byte Count][Data...][CRC]
        if len(rx_bytes) >= 3:
            byte_count = rx_bytes[2]
            return 3 + byte_count + 2
    elif function == 0x06 or function == 0x05:
        # 固定長度 8 bytes: [Addr][Func][Reg Hi][Reg Lo][Data Hi][Data Lo][CRC_L][CRC_H]
        return 8
    elif function == 0x10:
        # Write Multiple Registers 回應長度固定為 8 bytes
        return 8
    elif function == 0x11:
        # Report Slave ID: 回傳長度不定
        return 4

    # 若功能碼未知，無法判斷
    return None



# ===== 接收封包 =====
def read_packet(ser: serial.Serial, timeout=0.5, inter_char_timeout=0.05):
    ser.timeout = 0  # 非阻塞讀取
    buffer = bytearray()
    start_time = time.time()
    stable_time = time.time()
    last_len = 0
    expected_len = None

    while time.time() - start_time < timeout:
        waiting = ser.in_waiting
        if waiting:
            buffer.extend(ser.read(waiting))
            stable_time = time.time()

            # 嘗試計算預期封包長度
            expected_len = expected_length(buffer)

            # 若已知預期長度，且收到足夠的資料
            if expected_len and len(buffer) >= expected_len:
                break
        else:
            # 沒有新資料，等待一段時間確認是否已穩定
            if buffer and (time.time() - stable_time > inter_char_timeout):
                break
            time.sleep(0.001)

    elapsed = time.time() - start_time
    if not buffer:
        print("!! No response")
        return None

    print("<< Recv:", buffer.hex(" ").upper())
    print(f"   (elapsed time: {elapsed*1000:.1f} ms)")

    if len(buffer) < 4:
        print("!! Too short for valid Modbus RTU frame")
        return None

    recv_crc = buffer[-2] | (buffer[-1] << 8)
    calc_crc = crc16(buffer[:-2])
    if recv_crc != calc_crc:
        print(f"!! CRC error: recv=0x{recv_crc:04X}, calc=0x{calc_crc:04X}")
        return None

    # 處理異常回應
    function = buffer[1]
    if function & 0x80:
        exception_code = buffer[2]
        print(f"!! Modbus Exception: Code=0x{exception_code:02X} ({exception_message(exception_code)})")
        return None

    print("OK CRC16")
    return buffer

def send_and_read(ser, addr, func, payload=b"", retries=3):
    for i in range(retries):
        send_packet(ser, addr, func, payload)
        resp = read_packet(ser)
        if resp is not None:
            return resp
        print(f"Retry {i+1} failed")
        time.sleep(0.1)
    return None

class RS485Master:
    def __init__(self, port: str):
        self.ser = serial.Serial(port=port, baudrate=19200, parity=serial.PARITY_EVEN,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
        self.sonar_addr = 0x11
        self.winch_addr = 0x12
    def close(self):
        self.ser.close()
    # ===== 喚醒node1 =====
    def wake_up_node1(self):
        send_and_read(self.ser, self.sonar_addr, 0x11)
        
    # ===== switch control =====
    def setSonar(self, on: bool):
        # data : [0x00, 0x01, 0x00, 0x00](off) / [0x00, 0x01, 0xFF, 0x00](on)
        data = struct.pack(">HH", 0x0001, 0xFF00 if on else 0x0000)
        send_and_read(self.ser, self.sonar_addr, 0x05, data)
        
    # ===== Winch 控制 =====
    def wake_up_node2(self):
        send_and_read(self.ser, self.winch_addr, 0x11)

    def setMaxSpeed(self, speed):
        send_and_read(self.ser, self.winch_addr, 0x06, struct.pack(">HH", 0x9C40, speed))

    def setAcc(self, acc):
        send_and_read(self.ser, self.winch_addr, 0x06, struct.pack(">HH", 0x9C41, acc))

    def setCurrentStep(self, step):
        send_and_read(self.ser, self.winch_addr, 0x10, struct.pack(">HHBi", 0x9C42, 0x0002, 0x04, step))

    def setTargetStep(self, step):
        send_and_read(self.ser, self.winch_addr, 0x10, struct.pack(">HHBi", 0x9C44, 0x0002, 0x04, step))

    def reset_highest(self):
        send_and_read(self.ser, self.winch_addr, 0x21, b"\x00")

    def set_speed_acc(self, maxspeed, acc):
        self.send_cmd(struct.pack(">Bhh", 0x01, maxspeed, acc))

    def set_step(self, step):
        self.send_cmd(struct.pack(">Bi", 0x02, step))

    def stop(self):
        self.send_cmd(struct.pack(">B", 0x03))

# ===== 測試主程式 =====
if __name__ == "__main__":
    rs485Master = RS485Master(port="COM13")

    try:
        while True:
            # 喚醒node1
            rs485Master.wake_up_node1()
            time.sleep(1)
            # 開啟超音波模組
            rs485Master.setSonar(on=True)
            time.sleep(1)
            # 關閉超音波模組
            rs485Master.setSonar(on=False)
            time.sleep(1)
            # 喚醒node2
            rs485Master.wake_up_node2()
            time.sleep(1)
            # 控制winch
            rs485Master.setCurrentStep(0)
            time.sleep(1)
            rs485Master.setMaxSpeed(1000)
            time.sleep(1)
            rs485Master.setAcc(200)
            time.sleep(1)
            rs485Master.setTargetStep(0)
            time.sleep(1)


    except KeyboardInterrupt:
        print("Exit by user")
    finally:
        rs485Master.close()
