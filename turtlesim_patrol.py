#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random
import time

"""
專案名稱: Turtlesim AI Patrol Robot Simulation
描述: 模擬一個搭載 AI 視覺的巡邏機器人。
      1. 在 Turtlesim 視窗範圍內自主巡邏 (PATROL)。
      2. 隨機模擬 "發現入侵者" 事件。
      3. 發現入侵者後，停止巡邏，轉向入侵者方位，並發出警告 (TRACKING)。

=========================================
資料輸入輸出定義 (Data I/O Definition)
=========================================
[輸入 Input]
1. Topic: /turtle1/pose (Type: turtlesim/msg/Pose)
   - 來源: Turtlesim 模擬器
   - 用途: 獲取機器人當前座標 (x, y) 與 朝向 (theta)，用於邊界避障與轉向計算。

2. 模擬感知 (Mock Perception)
   - 來源: 內部隨機函數 (Random Generator)
   - 用途: 模擬 AI 視覺模型的輸出。
   - 資料格式: { 'detected': bool, 'target_angle': float (radians) }

[輸出 Output]
1. Topic: /turtle1/cmd_vel (Type: geometry_msgs/msg/Twist)
   - 目標: Turtlesim 模擬器
   - 用途: 控制機器人的線速度 (linear.x) 與 角速度 (angular.z)。

2. Log/Terminal
   - 用途: 顯示系統狀態、警報訊息 (模擬發出警告聲)。
=========================================
"""

class TurtlePatrolNode(Node):
    def __init__(self):
        super().__init__('turtle_patrol_node')

        # --- 參數設定 ---
        self.patrol_speed = 1.0       # 巡邏速度
        self.turn_speed = 1.0         # 轉向速度
        self.boundary_min = 1.0       # 牆壁邊界 (Turtlesim 範圍約 0-11)
        self.boundary_max = 10.0
        
        # --- 狀態變數 ---
        self.state = "PATROL"         # 初始狀態: PATROL, TRACKING
        self.pose = None              # 儲存當前位置
        self.target_angle = 0.0       # 入侵者的目標角度
        self.tracking_start_time = 0  # 追蹤開始時間

        # --- ROS 通訊介面 ---
        # 1. [Output] 發布速度指令
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # 2. [Input] 訂閱機器人姿態 (用於避障和導航)
        self.subscription_ = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # 3. 定時控制迴圈 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Turtlesim AI 巡邏機器人已啟動！")

    def pose_callback(self, msg):
        """更新機器人當前姿態 [Input Source]"""
        self.pose = msg

    def mock_ai_perception(self):
        """
        [模擬感知層]
        模擬 AI 視覺模型 (如 YOLO) 的行為。
        回傳: (是否發現入侵者, 入侵者相對於地圖的角度)
        """
        # 1% 機率在每個 tick 發現入侵者
        if random.random() < 0.01:
            # 隨機產生一個入侵者角度 (0 到 2*PI)
            intruder_angle = random.uniform(0, 2 * math.pi)
            return True, intruder_angle
        return False, 0.0

    def control_loop(self):
        """主控制邏輯 (核心大腦)"""
        if self.pose is None:
            return # 等待第一次 Pose 資料

        msg = Twist()

        # --- 1. 執行感知 (Perception) ---
        # 只有在巡邏模式下才主動掃描
        if self.state == "PATROL":
            detected, angle = self.mock_ai_perception()
            if detected:
                self.state = "TRACKING"
                self.target_angle = angle
                self.tracking_start_time = time.time()
                self.get_logger().warn(f"!!! 警告：偵測到入侵者 !!! 方位: {math.degrees(angle):.1f} 度")

        # --- 2. 執行決策與行動 (Decision & Action) ---
        
        if self.state == "PATROL":
            # [邏輯]：直走，遇到牆壁就轉彎
            msg.linear.x = self.patrol_speed
            msg.angular.z = 0.0

            # 簡單的邊界檢查 (Wall Avoidance)
            if (self.pose.x > self.boundary_max or self.pose.x < self.boundary_min or
                self.pose.y > self.boundary_max or self.pose.y < self.boundary_min):
                msg.linear.x = 0.5
                msg.angular.z = 2.0 # 撞牆前快速旋轉
            
        elif self.state == "TRACKING":
            # [邏輯]：停止移動，原地旋轉面向入侵者
            msg.linear.x = 0.0 # 停下
            
            # 計算角度誤差 (P-Controller)
            # 目標是讓機器人的朝向 (pose.theta) 對齊 入侵者角度 (target_angle)
            error = self.target_angle - self.pose.theta
            
            # 角度標準化 (確保轉向是走最短路徑，例如 -PI 到 PI 之間)
            while error > math.pi: error -= 2 * math.pi
            while error < -math.pi: error += 2 * math.pi

            # 如果誤差很小，視為已鎖定
            if abs(error) < 0.1:
                msg.angular.z = 0.0
                self.get_logger().warn(">> 已鎖定目標！發送影像至監控中心...", throttle_duration_sec=1.0)
            else:
                # 簡單的 P 控制器：轉向速度與誤差成正比
                msg.angular.z = 2.0 * error 

            # 追蹤 5 秒後恢復巡邏 (模擬入侵者離開或處理完畢)
            if time.time() - self.tracking_start_time > 5.0:
                self.state = "PATROL"
                self.get_logger().info("目標丟失/處理完畢，恢復巡邏模式。")

        # --- 3. [Output] 發送指令 ---
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()