#!/usr/bin/env python3
"""
方法1: 純CAN總線監聽
目的: 只監聽和記錄日行燈相關的CAN信號
適用: 分析原廠行為，不發送任何控制指令
"""

import can
from datetime import datetime

def monitor_drl_signals():
    """
    監聽日行燈相關的CAN信號
    只接收，不發送任何數據
    """
    try:
        # 初始化CAN接口
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        print("=== Toyota DRL信號監聽器 ===")
        print("已連接到CAN總線")
        print("監聽地址: 0x622 (LIGHT_STALK)")
        print("請手動操作車輛燈光開關...")
        print("按 Ctrl+C 停止監聽\n")

        # 記錄文件
        with open(f"drl_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt", "w") as log_file:
            log_file.write("時間,CAN_ID,原始數據,DRL位狀態\n")

            while True:
                message = bus.recv(timeout=1.0)
                if message and message.arbitration_id == 0x622:
                    # 解析DAYTIME_RUNNING_LIGHT位（第31位）
                    drl_bit = (message.data[3] >> 7) & 0x01

                    timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                    data_hex = message.data.hex().upper()

                    print(f"[{timestamp}] ID: 0x{message.arbitration_id:03X} | Data: {data_hex} | DRL: {drl_bit}")

                    # 記錄到文件
                    log_file.write(f"{timestamp},0x{message.arbitration_id:03X},{data_hex},{drl_bit}\n")
                    log_file.flush()

    except KeyboardInterrupt:
        print("\n監聽結束")
        print("數據已保存到日誌文件")
    except Exception as e:
        print(f"錯誤: {e}")

if __name__ == "__main__":
    monitor_drl_signals()
