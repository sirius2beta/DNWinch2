import serial
import struct
import time

class WinchClient:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )
        time.sleep(1)

    # ------------------------------
    # CRC8 (多項式 x^8 + x^2 + x + 1, poly=0x07)
    # ------------------------------
    def crc8(self, data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x07
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    # ------------------------------
    # 封包封裝
    # ------------------------------
    def build_packet(self, payload: bytes) -> bytes:
        length = len(payload)
        packet = bytes([0xAA, length]) + payload
        crc = self.crc8(packet[1:])  # LEN+PAYLOAD
        return packet + bytes([crc])

    def send_cmd(self, payload: bytes, resp_len=0):
        """傳送一個指令並等待回覆"""
        pkt = self.build_packet(payload)
        self.ser.write(pkt)
        if resp_len > 0:
            return self.read_packet(resp_len)
        return None

    def read_packet(self, expected_payload_len):
        """讀取回覆封包"""
        header = self.ser.read(1)
        if not header or header[0] != 0xAA:
            print("⚠️ Invalid header")
            return None

        length = self.ser.read(1)
        if not length:
            return None
        length = length[0]

        payload = self.ser.read(length)
        crc = self.ser.read(1)

        if len(payload) != length or not crc:
            print("⚠️ Length mismatch")
            return None

        calc_crc = self.crc8(bytes([length]) + payload)
        if calc_crc != crc[0]:
            print("⚠️ CRC error")
            return None

        return payload

    # ------------------------------
    # 指令封裝
    # ------------------------------
    def reset_highest(self):
        self.send_cmd(struct.pack(">B", 0x00))

    def set_speed_acc(self, maxspeed, acc):
        self.send_cmd(struct.pack(">Bhh", 0x01, maxspeed, acc))

    def set_step(self, step):
        self.send_cmd(struct.pack(">Bi", 0x02, step))

    def stop(self):
        self.send_cmd(struct.pack(">B", 0x03))

    def get_status(self):
        payload = self.send_cmd(struct.pack(">B", 0x04), resp_len=9)
        if payload:
            step = struct.unpack(">i", payload[0:4])[0]
            tension = struct.unpack(">i", payload[4:8])[0]
            running = payload[8]
            return {"step": step, "tension": tension, "running": running}
        return None

    def set_tension_threshold(self, tension):
        self.send_cmd(struct.pack(">Bi", 0x05, tension))

    def reset_position_zero(self):
        self.send_cmd(struct.pack(">B", 0x06))

    def get_aqua_data(self):
        expected_len = (21 * 4) + 4
        payload = self.send_cmd(struct.pack(">B", 0x07), resp_len=expected_len)
        if payload:
            unpacked = struct.unpack(">21fI", payload)
            keys = [
                "temperature", "pressure", "depth", "water_level_depth", "surface_elevation",
                "conductivity_real", "conductivity_specific", "resistivity", "salinity", "total_dissolved_solids",
                "density", "atmospheric_pressure", "pH_value", "pH_mV", "ORP",
                "DO_concentration", "DO_saturation_percent", "turbidity", "O2_partial_pressure",
                "external_voltage", "battery_capacity"
            ]
            data = {k: unpacked[i] for i, k in enumerate(keys)}
            data["lastUpdateMs"] = unpacked[21]
            return data
        return None
