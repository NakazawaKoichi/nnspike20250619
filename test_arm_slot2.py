#!/usr/bin/env python3
from nnspike.unit import ETRobot
import time

def main():
    try:
        print("シリアルポートに接続中...")
        et = ETRobot()
        print("接続成功")
        
        print("アームを動かします...")
        a = 0
        print("コマンド送信: COMMAND_MOVE_ARM_ID=205, action=", a)
        et.move_arm(action=a)
        
        # 動作状態を確認
        time.sleep(0.5)  # 少し待ってステータスを取得
        status = et.get_spike_status()
        print(f"モーターC（アーム）の状態: {status.motors['C']}")
        
        time.sleep(2)
        print("動作完了")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
        if "Serial" in str(e):
            print("シリアル接続に問題があります")
    finally:
        # 終了処理
        try:
            et.stop()
            print("正常に終了しました")
        except Exception as e:
            print(f"終了処理中にエラーが発生: {e}")

if __name__ == "__main__":
    main()
