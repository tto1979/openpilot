#!/usr/bin/env python3
"""
openpilot專用CAN監聽程式
在AGNOS系統中使用openpilot的CAN接口
"""

import sys
import time
from datetime import datetime

# 添加openpilot路徑
sys.path.append('/data/openpilot')

def monitor_with_openpilot_can():
    """使用openpilot的CAN系統監聽"""
    
    print("🚗 使用openpilot CAN系統監聽DRL信號")
    print("=" * 60)

    try:
        # 導入openpilot的CAN模組
        from opendbc.can.parser import CANParser
        from opendbc.can.packer import CANPacker
        from common.realtime import Ratekeeper
        from cereal import messaging

        print("✅ openpilot CAN模組載入成功")

        # 創建CAN消息訂閱
        can_sock = messaging.sub_sock("can")

        print("🔍 監聽CAN消息...")
        print("監聽地址: 0x622 (LIGHT_STALK)")
        print("按 Ctrl+C 停止監聽\n")

        # 創建日誌文件
        log_filename = f"drl_log_openpilot_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

        with open(log_filename, "w") as log_file:
            log_file.write("時間,總線,CAN_ID,原始數據,DRL位狀態\n")

            rk = Ratekeeper(100)  # 100Hz監聽

            while True:
                can_msgs = messaging.recv_sock(can_sock, wait=False)

                if can_msgs is not None:
                    for msg in can_msgs.can:
                        # 檢查是否為LIGHT_STALK消息 (0x622 = 1570)
                        if msg.address == 0x622:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                            # 解析數據
                            data_bytes = msg.dat
                            data_hex = ''.join(f'{b:02X}' for b in data_bytes)

                            # 解析DRL位（第31位 = 字節3第7位）
                            drl_bit = 0
                            if len(data_bytes) >= 4:
                                drl_bit = (data_bytes[3] >> 7) & 0x01

                            # 顯示結果
                            print(f"[{timestamp}] 總線:{msg.src} | ID:0x{msg.address:03X} | Data:{data_hex} | DRL:{drl_bit}")

                            # 記錄到文件
                            log_file.write(f"{timestamp},{msg.src},0x{msg.address:03X},{data_hex},{drl_bit}\n")
                            log_file.flush()

                rk.keep_time()

    except KeyboardInterrupt:
        print(f"\n監聽結束")
        print(f"數據已保存到: {log_filename}")

    except ImportError as e:
        print(f"❌ 無法導入openpilot模組: {e}")
        print("嘗試使用替代方案...")
        return False

    except Exception as e:
        print(f"❌ 監聽錯誤: {e}")
        return False

    return True

def monitor_with_messaging_bridge():
    """使用messaging bridge方式監聽"""

    print("\n🔄 嘗試messaging bridge方式...")

    try:
        from cereal import messaging

        # 訂閱CAN消息
        can_sock = messaging.sub_sock("can")

        print("✅ messaging橋接成功")
        print("開始監聽... 按 Ctrl+C 停止\n")

        log_filename = f"drl_log_bridge_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

        with open(log_filename, "w") as log_file:
            log_file.write("時間,總線,CAN_ID,原始數據,DRL位狀態\n")

            while True:
                # 接收CAN消息
                can_data = messaging.recv_sock(can_sock, wait=True)

                if can_data and can_data.can:
                    for msg in can_data.can:
                        if msg.address == 0x622:  # LIGHT_STALK
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                            data_hex = ''.join(f'{b:02X}' for b in msg.dat)
                            drl_bit = (msg.dat[3] >> 7) & 0x01 if len(msg.dat) >= 4 else 0

                            print(f"[{timestamp}] 總線:{msg.src} | ID:0x{msg.address:03X} | Data:{data_hex} | DRL:{drl_bit}")

                            log_file.write(f"{timestamp},{msg.src},0x{msg.address:03X},{data_hex},{drl_bit}\n")
                            log_file.flush()

    except KeyboardInterrupt:
        print(f"\n監聽結束")
        print(f"數據已保存到: {log_filename}")

    except Exception as e:
        print(f"❌ messaging bridge錯誤: {e}")
        return False

    return True

def monitor_with_direct_cereal():
    """直接使用cereal接口"""

    print("\n📡 嘗試直接cereal接口...")

    try:
        import cereal.messaging as messaging

        # 創建CAN訂閱者
        sm = messaging.SubMaster(['can'])

        print("✅ cereal接口連接成功")
        print("開始監聽CAN消息...\n")

        log_filename = f"drl_log_cereal_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"

        with open(log_filename, "w") as log_file:
            log_file.write("時間,總線,CAN_ID,原始數據,DRL位狀態\n")

            while True:
                sm.update(0)  # 非阻塞更新

                if sm.updated['can']:
                    can_msgs = sm['can']

                    for msg in can_msgs:
                        if msg.address == 0x622:
                            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]

                            data_hex = ''.join(f'{b:02X}' for b in msg.dat)
                            drl_bit = (msg.dat[3] >> 7) & 0x01 if len(msg.dat) >= 4 else 0

                            print(f"[{timestamp}] 總線:{msg.src} | ID:0x{msg.address:03X} | Data:{data_hex} | DRL:{drl_bit}")

                            log_file.write(f"{timestamp},{msg.src},0x{msg.address:03X},{data_hex},{drl_bit}\n")
                            log_file.flush()

                time.sleep(0.01)  # 10ms延遲

    except KeyboardInterrupt:
        print(f"\n監聽結束")
        print(f"數據已保存到: {log_filename}")

    except Exception as e:
        print(f"❌ cereal接口錯誤: {e}")
        return False

    return True

def check_openpilot_status():
    """檢查openpilot運行狀態"""

    print("🔍 檢查openpilot運行狀態...")

    import subprocess

    try:
        # 檢查主要進程
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)

        important_processes = ['manager.py', 'boardd', 'pandad', 'ui']
        running_processes = []

        for process in important_processes:
            if process in result.stdout:
                running_processes.append(process)
                print(f"✅ {process} 正在運行")
            else:
                print(f"❌ {process} 未運行")

        if len(running_processes) >= 2:
            print("✅ openpilot基本服務正常")
            return True
        else:
            print("⚠️ openpilot服務可能異常")
            return False

    except Exception as e:
        print(f"❌ 檢查狀態失敗: {e}")
        return False

def provide_openpilot_guidance():
    """提供openpilot特定的指導"""

    print("\n📋 openpilot CAN監聽指南")
    print("=" * 60)

    guidance = [
        "1. 🚗 確認車輛連接:",
        "   - 確保車輛通電 (至少ACC檔)",
        "   - 檢查openpilot與車輛的連接",
        "   - 確認openpilot正常運行",
        "",
        "2. 🔧 啟動openpilot (如果未運行):",
        "   - cd /data/openpilot",
        "   - ./launch_openpilot.sh",
        "   - 或重啟comma設備",
        "",
        "3. 📊 檢查CAN數據流:",
        "   - tmux a  # 進入openpilot會話",
        "   - 檢查是否有CAN錯誤信息",
        "",
        "4. 🎯 測試燈光控制:",
        "   - 在openpilot運行時操作燈光開關",
        "   - 檢查是否有CAN通信",
        "",
        "5. 🔍 使用openpilot工具:",
        "   - /data/openpilot/tools/ 目錄中的調試工具",
        "   - cabana工具進行CAN分析"
    ]

    for item in guidance:
        print(item)

def main():
    """主函數"""

    print("🚗 openpilot CAN監聽工具 (AGNOS專用)")
    print("=" * 60)

    # 檢查openpilot狀態
    if not check_openpilot_status():
        print("\n⚠️ openpilot服務狀態異常")
        provide_openpilot_guidance()
        return

    print("\n嘗試不同的CAN監聽方法...")

    # 嘗試多種方法
    methods = [
        ("openpilot CAN系統", monitor_with_openpilot_can),
        ("messaging橋接", monitor_with_messaging_bridge),
        ("直接cereal接口", monitor_with_direct_cereal)
    ]

    for method_name, method_func in methods:
        print(f"\n🔄 嘗試: {method_name}")
        try:
            if method_func():
                print(f"✅ {method_name} 成功")
                return
            else:
                print(f"❌ {method_name} 失敗")
        except Exception as e:
            print(f"❌ {method_name} 異常: {e}")

    print("\n❌ 所有方法都失敗")
    provide_openpilot_guidance()

if __name__ == "__main__":
    monitor_drl_signals()
