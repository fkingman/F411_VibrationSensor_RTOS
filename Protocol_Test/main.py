import serial
import struct
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import os
import sys

# ==========================================
# ⚙️ 全局配置 (Global Configuration)
# ==========================================
CONFIG = {
    'PORT': 'COM9',  # 默认串口号
    'BAUD': 9600,  # 波特率
    'ADDR': 0x00,  # 默认目标设备地址 (非广播时使用)
    'TIMEOUT': 2.0,  # 默认超时
    'OTA_FILE': 'F411_VibrationSensor_RTOS.bin',  # OTA固件名
    'SAVE_DIR': 'wave_data',  # 波形保存路径
    'OTA_ERASE_TIME': 8.0,  # OTA擦除等待时间
    'OTA_PACKET_SIZE': 256  # OTA包大小
}

# --- 📋 协议命令码 ---
CMD_FEATURE = 0x02  # 特征值请求
CMD_WAVE = 0x04  # 波形请求 (Snapshot)
CMD_WAVE_PACK = 0x03  # 波形包读取
CMD_DISCOVER = 0x41  # 🔍 发现设备/读取UID
CMD_CONFIG = 0x87  # ⚙️ 设置频率
CMD_OTA_START = 0x50  # OTA 开始
CMD_OTA_DATA = 0x51  # OTA 数据
CMD_OTA_END = 0x52  # OTA 结束


# ==========================================
# 🛠️ 协议工具函数 (Protocol Utils)
# ==========================================

def calc_crc16(data):
    """计算 Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if (crc & 1) != 0:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


def build_frame(addr, cmd, payload=b''):
    """
    构建通用发送帧: [Addr, Cmd, Payload..., CRC_L, CRC_H]
    """
    # 头部
    head = struct.pack('BB', addr, cmd)
    frame_body = head + payload
    # 计算CRC
    crc = calc_crc16(frame_body)
    # 拼接完整帧
    return frame_body + struct.pack('<H', crc)


def open_serial():
    """打开串口并返回对象"""
    try:
        ser = serial.Serial(CONFIG['PORT'], CONFIG['BAUD'], timeout=CONFIG['TIMEOUT'])
        print(f"串口 {CONFIG['PORT']} 打开成功 (Baud: {CONFIG['BAUD']})")
        return ser
    except Exception as e:
        print(f" 串口打开失败: {e}")
        return None


# ==========================================
# 🔍 功能: 发现设备与读取 UID
# ==========================================
def task_discover():
    ser = open_serial()
    if not ser: return

    try:
        print("\n[扫描] 正在发送广播发现命令 (Addr: 0x00)...")
        # 构造广播帧: [00] [41] [CRC_L] [CRC_H]
        frame = build_frame(0x00, CMD_DISCOVER, b'\x00\x00\x00')
        ser.write(frame)

        # 接收响应
        # 响应结构: [Addr] [41] [Len=13] [UID(12B)] [Addr] [CRC_L] [CRC_H]
        # 总长度 = 1 + 1 + 1 + 12 + 1 + 2 = 18 字节
        EXPECTED_LEN = 18

        # 稍微增加超时，因为可能有多个设备或随机延时
        ser.timeout = 2.0
        resp = ser.read(EXPECTED_LEN)

        if len(resp) != EXPECTED_LEN:
            print(f"❌ 响应超时或长度错误 (Len={len(resp)})")
            if len(resp) > 0:
                print(f"   收到数据: {resp.hex()}")
            return

        # 校验 CRC
        if calc_crc16(resp[:-2]) != struct.unpack('<H', resp[-2:])[0]:
            print("❌ CRC 校验失败")
            return

        # 解析数据
        dev_addr = resp[0]
        cmd = resp[1]
        length = resp[2]
        uid_bytes = resp[3:15]
        addr_check = resp[15]

        if cmd != CMD_DISCOVER:
            print(f"❌ 命令码错误: 0x{cmd:02X}")
            return

        uid_str = uid_bytes.hex().upper()

        print("\n" + "=" * 40)
        print("🔍 发现设备")
        print("=" * 40)
        print(f"✅ 设备地址: 0x{dev_addr:02X}")
        print(f"🆔 设备 UID: {uid_str}")
        print(f"   (UID W0: {uid_str[0:8]})")
        print(f"   (UID W1: {uid_str[8:16]})")
        print(f"   (UID W2: {uid_str[16:24]})")
        print("=" * 40 + "\n")

        # 自动更新当前配置地址
        CONFIG['ADDR'] = dev_addr
        print(f"已自动将操作地址更新为: 0x{dev_addr:02X}")

    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        ser.close()


# ==========================================
# ⚙️ 功能: 设置采样频率
# ==========================================
def task_set_frequency():
    print("\n--- 设置传感器采样频率 ---")
    print("1. 25600 Hz")
    print("2. 12800 Hz")
    print("3. 6400 Hz")
    print("4. 3200 Hz")
    print("5. 1600 Hz")

    sel = input("请选择频率 [1-5]: ").strip()
    freq_map = {'1': 25600, '2': 12800, '3': 6400, '4': 3200, '5': 1600}

    if sel not in freq_map:
        print("无效选择")
        return

    target_freq = freq_map[sel]

    ser = open_serial()
    if not ser: return

    try:
        print(f"\n[设置] 正在将频率设置为 {target_freq} Hz...")

        # 构造 Payload
        # C代码: rd_be16(&rx[3]) -> 意味着 rx[0]=Dev, rx[1]=Cmd, rx[2]=Pad, rx[3]=High, rx[4]=Low
        # 所以 Payload 需要先补一个 0x00 字节，再加大端 uint16
        payload = struct.pack('B', 0x00) + struct.pack('>H', target_freq)

        frame = build_frame(CONFIG['ADDR'], CMD_CONFIG, payload)
        ser.write(frame)

        # 接收 ACK
        # 结构: [Dev] [87] [02] [4F] [4B] [CRC_L] [CRC_H] -> 7字节
        ack = ser.read(7)

        if len(ack) == 7:
            if ack[1] == CMD_CONFIG and ack[3] == 0x4F and ack[4] == 0x4B:
                print(f"✅ 设置成功! 传感器已切换至 {target_freq} Hz")
            else:
                print(f"❌ 收到异常响应: {ack.hex()}")
        else:
            print("❌ 等待响应超时")

    except Exception as e:
        print(f"运行时错误: {e}")
    finally:
        ser.close()


# ==========================================
# 📈 功能: 读取特征值与波形
# ==========================================

def parse_features_and_print(raw_data):
    """解析特征值包"""
    if len(raw_data) != 77:
        print(f" 特征值包长度错误: {len(raw_data)} (预期 77)")
        return False

    payload = raw_data[3:-2]
    floats = struct.unpack('>18f', payload)

    print("\n" + "=" * 40)
    print(f"传感器特征值报告 (设备 0x{raw_data[0]:02X})")
    print("=" * 40)
    print(f"【X 轴】 Mean:{floats[0]:.4f}g, RMS:{floats[1]:.4f}mm/s, P-P:{floats[2]:.4f}g, Kurt:{floats[3]:.4f}")
    print(f"【Y 轴】 Mean:{floats[4]:.4f}g, RMS:{floats[5]:.4f}mm/s, P-P:{floats[6]:.4f}g, Kurt:{floats[7]:.4f}")
    print(f"【Z 轴】 Mean:{floats[8]:.4f}g, RMS:{floats[9]:.4f}mm/s, P-P:{floats[10]:.4f}g, Kurt:{floats[11]:.4f}")
    print(f"       主频:{floats[12]:.1f}Hz, 幅值:{floats[13]:.4f}g")
    print(f"       包络RMS:{floats[15]:.4f}g, 包络峰值:{floats[16]:.4f}g")
    print(f"【其他】 温度:{floats[17]:.2f}")
    print("=" * 40 + "\n")
    return True


def task_read_sensor():
    ser = open_serial()
    if not ser: return

    if not os.path.exists(CONFIG['SAVE_DIR']):
        os.makedirs(CONFIG['SAVE_DIR'])

    try:
        # 1. 获取特征值
        print(f"[1/3] 请求特征值 (Addr: 0x{CONFIG['ADDR']:02X})...")
        payload = struct.pack('BBB', 0, 0, 0)
        ser.write(build_frame(CONFIG['ADDR'], CMD_FEATURE, payload))

        feat_resp = ser.read(77)
        if len(feat_resp) == 77:
            parse_features_and_print(feat_resp)
        else:
            print(f"特征值读取失败 (Len={len(feat_resp)})")

        # 2. 请求波形快照
        print(f"[2/3] 请求波形快照...")
        payload = struct.pack('BBB', 0, 0, 0)
        ser.write(build_frame(CONFIG['ADDR'], CMD_WAVE, payload))
        ack = ser.read(7)
        if not (len(ack) == 7 and ack[3] == 0x4F):
            print(f"快照请求失败: {ack.hex()}")
            return

        print("快照锁定成功")

        # 3. 读取波形数据
        TOTAL_POINTS = 4096
        PTS_PER_PKT = 64
        total_pkts = TOTAL_POINTS // PTS_PER_PKT
        all_data = []

        print(f"[3/3] 开始读取波形 ({TOTAL_POINTS}点)...")
        start_time = time.time()
        expected_len = 4 + (PTS_PER_PKT * 4) + 2

        for seq in range(total_pkts):
            payload = struct.pack('BB B', seq, total_pkts, 0x00)
            ser.write(build_frame(CONFIG['ADDR'], CMD_WAVE_PACK, payload))

            resp = ser.read(expected_len)
            if len(resp) != expected_len:
                print(f"\n 包 {seq} 丢失 (Len={len(resp)})")
                break

            floats = np.frombuffer(resp[4:-2], dtype='>f4')
            all_data.extend(floats)
            print(f"\r 进度: {seq + 1}/{total_pkts}", end='')

        print(f"\n 读取完成，耗时 {time.time() - start_time:.2f}s")

        if len(all_data) > 0:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = f"{CONFIG['SAVE_DIR']}/wave_{timestamp}.csv"
            pd.DataFrame(all_data, columns=["Acceleration_g"]).to_csv(csv_path, index_label="Index")
            print(f" CSV已保存: {csv_path}")

            plt.figure(figsize=(10, 5))
            plt.plot(all_data, label='Z-Axis', color='#1f77b4', linewidth=0.8)
            plt.title(f"Waveform - {timestamp}")
            plt.grid(True, alpha=0.5)
            plt.legend()
            img_path = f"{CONFIG['SAVE_DIR']}/plot_{timestamp}.png"
            plt.savefig(img_path, dpi=100)
            print(f"️ 图片已保存: {img_path}")
            plt.show()

    except Exception as e:
        print(f" 运行出错: {e}")
    finally:
        ser.close()


# ==========================================
# 🔄 功能: OTA 固件升级
# ==========================================

def send_and_wait_ota(ser, frame, description, expected_len=7):
    ser.write(frame)
    start_time = time.time()
    received = b''
    while len(received) < expected_len:
        if time.time() - start_time > 3.0: break
        if ser.in_waiting:
            received += ser.read(ser.in_waiting)

    if len(received) != expected_len:
        print(f"\n [OTA] {description} 失败: 长度不符 ({len(received)}/{expected_len})")
        return False
    recv_crc = struct.unpack('<H', received[-2:])[0]
    calc_crc = calc_crc16(received[:-2])
    if recv_crc != calc_crc:
        print(f"\n [OTA] {description} CRC错误")
        return False
    print(f"\r {description} OK", end='')
    return True


def task_ota_update():
    bin_path = CONFIG['OTA_FILE']
    if not os.path.exists(bin_path):
        new_path = input(f"️ 找不到默认固件 '{bin_path}'，请输入路径: ").strip().strip('"')
        if not new_path: return
        bin_path = new_path

    if not os.path.exists(bin_path):
        print(" 文件不存在")
        return

    with open(bin_path, 'rb') as f:
        firmware_data = bytearray(f.read())
    remainder = len(firmware_data) % 32
    if remainder != 0: firmware_data += b'\xFF' * (32 - remainder)
    padded_len = len(firmware_data)
    print(f"\n 固件准备就绪: {padded_len} bytes")

    ser = open_serial()
    if not ser: return
    ser.timeout = 0.1

    try:
        print(" 发送 OTA Start 指令...")
        payload = struct.pack('>I', padded_len)
        frame = build_frame(CONFIG['ADDR'], CMD_OTA_START, payload)
        if not send_and_wait_ota(ser, frame, "OTA Start"): return

        print(f"\n 等待 Flash 擦除 ({CONFIG['OTA_ERASE_TIME']}s)...")
        time.sleep(CONFIG['OTA_ERASE_TIME'])

        print(" 开始发送数据包...")
        offset = 0
        total_chunks = (padded_len + CONFIG['OTA_PACKET_SIZE'] - 1) // CONFIG['OTA_PACKET_SIZE']
        chunk_idx = 0

        while offset < padded_len:
            chunk = firmware_data[offset: offset + CONFIG['OTA_PACKET_SIZE']]
            payload = struct.pack('>I', offset) + struct.pack('>H', len(chunk)) + chunk
            frame = build_frame(CONFIG['ADDR'], CMD_OTA_DATA, payload)
            percent = (chunk_idx / total_chunks) * 100
            if not send_and_wait_ota(ser, frame, f"Packet {chunk_idx + 1}/{total_chunks} ({percent:.1f}%)"):
                print(f"\n 在 Offset {offset} 处中断")
                return
            offset += len(chunk)
            chunk_idx += 1

        print("\n 发送 OTA End 指令...")
        payload = struct.pack('>I', padded_len)
        frame = build_frame(CONFIG['ADDR'], CMD_OTA_END, payload)
        send_and_wait_ota(ser, frame, "OTA End")
        print("\n OTA 升级流程完成!")

    except Exception as e:
        print(f"\n OTA 过程中出错: {e}")
    finally:
        ser.close()


# ==========================================
# 🖥️ 主菜单
# ==========================================
def main():
    while True:
        print("\n" + "=" * 40)
        print("    STM32 振动传感器调试工具 v2.0")
        print("=" * 40)
        print(f"当前状态: Port={CONFIG['PORT']}, Baud={CONFIG['BAUD']}, TargetAddr=0x{CONFIG['ADDR']:02X}")
        print("-" * 40)
        print("1. [数据] 读取特征值 & 波形")
        print("2. [设置] 设置采样频率 (Hz)")
        print("3. [工具] 扫描设备 & 读取 UID")
        print("4. [升级] OTA 固件升级")
        print("5. [配置] 修改串口 & 目标地址")
        print("q. [退出] 退出程序")
        print("=" * 40)

        choice = input("请选择功能: ").strip().lower()

        if choice == '1':
            task_read_sensor()
        elif choice == '2':
            task_set_frequency()
        elif choice == '3':
            task_discover()
        elif choice == '4':
            task_ota_update()
        elif choice == '5':
            p = input(f"输入串口号 (默认 {CONFIG['PORT']}): ").strip()
            if p: CONFIG['PORT'] = p
            b = input(f"输入波特率 (默认 {CONFIG['BAUD']}): ").strip()
            if b: CONFIG['BAUD'] = int(b)
            a = input(f"输入目标地址Hex (默认 {CONFIG['ADDR']:02X}): ").strip()
            if a: CONFIG['ADDR'] = int(a, 16)
        elif choice == 'q':
            print("Bye! ")
            break
        else:
            print("输入无效")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户强制退出")