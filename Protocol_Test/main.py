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
    'ADDR': 0x00,  # 设备地址
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
# 📈 功能1: 读取特征值与波形 (Sensor Task)
# ==========================================

def parse_features_and_print(raw_data):
    """解析特征值包"""
    if len(raw_data) != 77:
        print(f" 特征值包长度错误: {len(raw_data)} (预期 77)")
        return False

    # 提取数据区 (跳过头部3字节[Addr,Cmd,Len]，最后2字节CRC)
    payload = raw_data[3:-2]
    floats = struct.unpack('>18f', payload)

    print("\n" + "=" * 40)
    print(f"传感器特征值报告 (设备 0x{raw_data[0]:02X})")
    print("=" * 40)
    print(f"【X 轴】 Mean:{floats[0]:.4f}g, RMS:{floats[1]:.4f}g, P-P:{floats[2]:.4f}g, Kurt:{floats[3]:.4f}")
    print(f"【Y 轴】 Mean:{floats[4]:.4f}g, RMS:{floats[5]:.4f}g, P-P:{floats[6]:.4f}g, Kurt:{floats[7]:.4f}")
    print(f"【Z 轴】 Mean:{floats[8]:.4f}g, RMS:{floats[9]:.4f}g, P-P:{floats[10]:.4f}g, Kurt:{floats[11]:.4f}")
    print(f"       主频:{floats[12]:.1f}Hz, 幅值:{floats[13]:.4f}g")
    print(f"【其他】 温度:{floats[17]:.2f}")
    print("=" * 40 + "\n")
    return True


def task_read_sensor():
    ser = open_serial()
    if not ser: return

    # 确保保存目录存在
    if not os.path.exists(CONFIG['SAVE_DIR']):
        os.makedirs(CONFIG['SAVE_DIR'])

    try:
        # 1. 获取特征值
        print(f"[1/3] 请求特征值...")
        # 特征值命令Payload: b2=0, b3=0, 00 (兼容旧逻辑)
        payload = struct.pack('BBB', 0, 0, 0)
        ser.write(build_frame(CONFIG['ADDR'], CMD_FEATURE, payload))

        feat_resp = ser.read(77)
        if len(feat_resp) == 77:
            parse_features_and_print(feat_resp)
        else:
            print(f"特征值读取失败 (Len={len(feat_resp)})")
            # 这里不return，继续尝试读波形

        # 2. 请求波形快照
        print(f"[2/3] 请求波形快照...")
        payload = struct.pack('BBB', 0, 0, 0)
        ser.write(build_frame(CONFIG['ADDR'], CMD_WAVE, payload))
        ack = ser.read(7)
        if not (len(ack) == 7 and ack[3] == 0x4F):  # Check 'O'
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
            # 构建包请求: [Seq, Total] + Pad
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

        # 4. 绘图与保存
        if len(all_data) > 0:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

            # CSV
            csv_path = f"{CONFIG['SAVE_DIR']}/wave_{timestamp}.csv"
            pd.DataFrame(all_data, columns=["Acceleration_g"]).to_csv(csv_path, index_label="Index")
            print(f" CSV已保存: {csv_path}")

            # Plot
            plt.figure(figsize=(10, 5))
            plt.plot(all_data, label='Z-Axis', color='#1f77b4', linewidth=0.8)
            plt.title(f"Waveform - {timestamp}")
            plt.grid(True, alpha=0.5)
            plt.legend()

            img_path = f"{CONFIG['SAVE_DIR']}/plot_{timestamp}.png"
            plt.savefig(img_path, dpi=100)
            print(f"️ 图片已保存: {img_path}")
            #print("哪怕绘图窗口打开，你也可以关闭它来回到主菜单。")
            plt.show()

    except Exception as e:
        print(f" 运行出错: {e}")
    finally:
        ser.close()
        print(" 串口已关闭")


# ==========================================
# 🔄 功能2: OTA 固件升级 (OTA Task)
# ==========================================

def send_and_wait_ota(ser, frame, description, expected_len=7):
    """OTA专用的发送接收函数"""
    ser.write(frame)

    start_time = time.time()
    received = b''

    # 循环读取直到满足长度或超时
    while len(received) < expected_len:
        if time.time() - start_time > 3.0: break
        if ser.in_waiting:
            received += ser.read(ser.in_waiting)

    if len(received) != expected_len:
        print(f"\n [OTA] {description} 失败: 长度不符 ({len(received)}/{expected_len})")
        return False

    # 简易校验 (CRC校验建议加上，这里略去以保持简洁，依赖长度和Modbus结构)
    recv_crc = struct.unpack('<H', received[-2:])[0]
    calc_crc = calc_crc16(received[:-2])
    if recv_crc != calc_crc:
        print(f"\n [OTA] {description} CRC错误")
        return False

    print(f"\r {description} OK", end='')
    return True


def task_ota_update():
    bin_path = CONFIG['OTA_FILE']

    # 检查文件
    if not os.path.exists(bin_path):
        new_path = input(f"️ 找不到默认固件 '{bin_path}'，请输入路径 (回车退出): ").strip()
        if not new_path: return
        bin_path = new_path.strip('"')  # 去除可能存在的引号

    if not os.path.exists(bin_path):
        print(" 文件不存在")
        return

    # 读取并处理固件
    with open(bin_path, 'rb') as f:
        firmware_data = bytearray(f.read())

    # 32字节对齐
    remainder = len(firmware_data) % 32
    if remainder != 0:
        firmware_data += b'\xFF' * (32 - remainder)

    padded_len = len(firmware_data)
    print(f"\n 固件准备就绪: {padded_len} bytes")

    ser = open_serial()
    if not ser: return
    ser.timeout = 0.1  # OTA模式需要快速轮询

    try:
        # 1. OTA Start
        print(" 发送 OTA Start 指令...")
        payload = struct.pack('>I', padded_len)
        frame = build_frame(CONFIG['ADDR'], CMD_OTA_START, payload)

        if not send_and_wait_ota(ser, frame, "OTA Start"):
            return

        # 2. 等待擦除
        print(f"\n 等待 Flash 擦除 ({CONFIG['OTA_ERASE_TIME']}s)...")
        time.sleep(CONFIG['OTA_ERASE_TIME'])

        # 3. 发送数据
        print(" 开始发送数据包...")
        offset = 0
        total_chunks = (padded_len + CONFIG['OTA_PACKET_SIZE'] - 1) // CONFIG['OTA_PACKET_SIZE']
        chunk_idx = 0

        while offset < padded_len:
            chunk = firmware_data[offset: offset + CONFIG['OTA_PACKET_SIZE']]
            # OTA Data Payload: Offset(4) + Len(2) + Data(...)
            payload = struct.pack('>I', offset) + struct.pack('>H', len(chunk)) + chunk
            frame = build_frame(CONFIG['ADDR'], CMD_OTA_DATA, payload)

            # 显示进度条
            percent = (chunk_idx / total_chunks) * 100
            desc = f"Packet {chunk_idx + 1}/{total_chunks} ({percent:.1f}%)"

            if not send_and_wait_ota(ser, frame, desc):
                print(f"\n 在 Offset {offset} 处中断")
                return

            offset += len(chunk)
            chunk_idx += 1

        # 4. OTA End
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
# 🖥️ 主菜单 (Main Menu)
# ==========================================
def main():
    while True:
        print("\n" + "=" * 30)
        print("    传感器调试与升级工具")
        print("=" * 30)
        print(f"当前配置: {CONFIG['PORT']} @ {CONFIG['BAUD']}")
        print("1.  读取特征值 & 波形 (Read Data)")
        print("2.  OTA 固件升级 (Firmware Update)")
        print("3.  修改串口配置 (Config Serial)")
        print("q. 退出 (Quit)")

        choice = input("\n请选择功能 [1/2/3/q]: ").strip().lower()

        if choice == '1':
            task_read_sensor()
        elif choice == '2':
            task_ota_update()
        elif choice == '3':
            p = input(f"输入串口号 (默认 {CONFIG['PORT']}): ").strip()
            if p: CONFIG['PORT'] = p
            b = input(f"输入波特率 (默认 {CONFIG['BAUD']}): ").strip()
            if b: CONFIG['BAUD'] = int(b)
        elif choice == 'q':
            print("Bye! ")
            break
        else:
            print("输入无效，请重新选择")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n用户强制退出")