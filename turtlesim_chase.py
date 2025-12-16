#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, Kill, SetPen
from std_srvs.srv import Empty as EmptySrv
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import math
import random
import time

"""
專案名稱: Turtlesim Police Chase v3 (黑色行動版)
升級功能:
  1. 演算法優化: 引入牆壁斥力場 (Wall Repulsion)，解決撞牆問題。
  2. 視覺升級: 黑色背景、高對比軌跡顏色。
  3. 狀態管理: 逮捕後清除路徑 (Clear Path)。
"""

class PoliceChaseNode(Node):
    def __init__(self):
        super().__init__('police_chase_node')
        
        # --- 參數設定 ---
        self.catch_distance = 0.8     # 逮捕判定距離
        self.base_speed = 1.0         # 警察基礎速度
        self.max_speed = 2.5          # 警察最大衝刺速度
        self.intruder_speed = 2.0     # 入侵者速度 (稍微加快)
        self.state = "CHASING"        
        self.arrest_time = 0.0

        # --- 變數初始化 ---
        self.police_pose = None
        self.intruder_pose = None
        
        # --- Service Clients ---
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.police_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.intruder_pen_client = self.create_client(SetPen, '/intruder/set_pen')
        self.clear_client = self.create_client(EmptySrv, '/clear')
        self.param_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        
        # --- 初始化設定 ---
        # 1. 設定背景為黑色
        self.set_background_black()
        
        # 2. 初始化場景
        self.reset_intruder_scenario(first_run=True)
        
        # 3. 設定警察顏色 (青色 Cyan - 在黑色背景下對比度高，且與紅色區分)
        # 若您堅持要紅色，可改為 (255, 0, 0)，但會與入侵者混淆
        self.set_pen_color('turtle1', 0, 255, 255, width=3)

        # --- Publishers ---
        self.pub_police = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_intruder = self.create_publisher(Twist, '/intruder/cmd_vel', 10)

        # --- Subscribers ---
        self.sub_police = self.create_subscription(Pose, '/turtle1/pose', self.police_callback, 10)
        self.sub_intruder = self.create_subscription(Pose, '/intruder/pose', self.intruder_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(0.05, self.control_loop)
        
        # 隨機擾動計時器
        self.noise_timer = 0
        self.noise_angle = 0.0

        self.get_logger().info("智慧警匪追逐系統 v3 (黑色行動) 已啟動！")

    def set_background_black(self):
        """呼叫參數服務將 Turtlesim 背景設為黑色"""
        if not self.param_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("無法連接參數服務，背景顏色可能無法變更")
            return

        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='background_r', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
            Parameter(name='background_g', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
            Parameter(name='background_b', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))
        ]
        
        future = self.param_client.call_async(req)
        # 呼叫 clear 讓顏色生效
        clear_req = EmptySrv.Request()
        self.clear_client.call_async(clear_req)

    def set_pen_color(self, turtle_name, r, g, b, width=2, off=0):
        client = self.police_pen_client if turtle_name == 'turtle1' else self.intruder_pen_client
        if client.service_is_ready():
            req = SetPen.Request()
            req.r = r
            req.g = g
            req.b = b
            req.width = width
            req.off = off
            client.call_async(req)

    def reset_intruder_scenario(self, first_run=False):
        """重置場景"""
        # 如果不是第一次執行，先清除路徑
        if not first_run:
            if self.clear_client.service_is_ready():
                self.clear_client.call_async(EmptySrv.Request())

        # 刪除舊入侵者
        kill_req = Kill.Request()
        kill_req.name = 'intruder'
        self.kill_client.call_async(kill_req) 
        time.sleep(0.5) # 等待刪除

        # 生成新入侵者 (離警察遠一點)
        spawn_req = Spawn.Request()
        spawn_req.x = random.uniform(1.0, 10.0)
        spawn_req.y = random.uniform(1.0, 10.0)
        spawn_req.theta = random.uniform(0, 6.28)
        spawn_req.name = 'intruder'
        
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(self.spawn_callback)

    def spawn_callback(self, future):
        try:
            future.result()
            self.get_logger().info(">>> 入侵者已生成！")
            self.state = "CHASING"
            time.sleep(0.2)
            # 設定入侵者顏色 (紅色)
            self.set_pen_color('intruder', 255, 0, 0, width=2)
        except Exception as e:
            self.get_logger().error(f"生成失敗: {e}")

    def police_callback(self, msg):
        self.police_pose = msg

    def intruder_callback(self, msg):
        self.intruder_pose = msg

    def get_distance(self):
        if not self.police_pose or not self.intruder_pose:
            return 999.0
        return math.sqrt(
            (self.intruder_pose.x - self.police_pose.x)**2 + 
            (self.intruder_pose.y - self.police_pose.y)**2
        )

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def control_loop(self):
        if not self.police_pose or not self.intruder_pose:
            return

        police_cmd = Twist()
        intruder_cmd = Twist()
        distance = self.get_distance()

        if self.state == "CHASING":
            # ==========================================
            # 1. 入侵者演算法: 向量場 (Vector Field)
            # ==========================================
            
            # [力向量 1] 來自警察的斥力 (遠離警察)
            vec_police_x = self.intruder_pose.x - self.police_pose.x
            vec_police_y = self.intruder_pose.y - self.police_pose.y
            
            # [力向量 2] 來自牆壁的斥力 (Wall Repulsion)
            # 牆壁範圍約 0 ~ 11
            vec_wall_x = 0.0
            vec_wall_y = 0.0
            margin = 2.0  # 距離牆壁多少開始產生斥力
            force_gain = 3.0 # 牆壁斥力強度

            if self.intruder_pose.x < margin: 
                vec_wall_x = (margin - self.intruder_pose.x) * force_gain # 向右推
            elif self.intruder_pose.x > (11.0 - margin):
                vec_wall_x = - (self.intruder_pose.x - (11.0 - margin)) * force_gain # 向左推
            
            if self.intruder_pose.y < margin:
                vec_wall_y = (margin - self.intruder_pose.y) * force_gain # 向上推
            elif self.intruder_pose.y > (11.0 - margin):
                vec_wall_y = - (self.intruder_pose.y - (11.0 - margin)) * force_gain # 向下推

            # [合力]
            total_x = vec_police_x + vec_wall_x
            total_y = vec_police_y + vec_wall_y
            
            # 計算目標逃跑角度
            final_escape_angle = math.atan2(total_y, total_x)

            # 加入一點點隨機擾動，避免陷入死循環
            self.noise_timer += 1
            if self.noise_timer > 5:
                self.noise_angle = random.uniform(-0.3, 0.3)
                self.noise_timer = 0
            
            final_escape_angle += self.noise_angle

            # 入侵者控制
            angle_diff_i = self.normalize_angle(final_escape_angle - self.intruder_pose.theta)
            intruder_cmd.angular.z = 4.0 * angle_diff_i
            # 如果轉向角度大，速度放慢，避免原地打轉
            intruder_speed_factor = max(0.3, 1.0 - abs(angle_diff_i))
            intruder_cmd.linear.x = self.intruder_speed * intruder_speed_factor
            
            self.pub_intruder.publish(intruder_cmd)

            # ==========================
            # 2. 警察演算法: 追蹤 (Pursuit)
            # ==========================
            
            dx = self.intruder_pose.x - self.police_pose.x
            dy = self.intruder_pose.y - self.police_pose.y
            target_angle = math.atan2(dy, dx)
            
            angle_diff_p = self.normalize_angle(target_angle - self.police_pose.theta)
            police_cmd.angular.z = 6.0 * angle_diff_p
            
            # 警察在直線時加速
            speed_factor = 1.0 - (abs(angle_diff_p) / math.pi)
            current_speed = self.base_speed + (distance * 0.8)
            current_speed = min(current_speed, self.max_speed)
            
            police_cmd.linear.x = current_speed * max(0.1, speed_factor)

            self.pub_police.publish(police_cmd)

            # ==========================
            # 3. 逮捕判定
            # ==========================
            if distance < self.catch_distance:
                self.state = "ARRESTED"
                self.arrest_time = time.time()
                self.get_logger().warn(f"!!! 逮捕成功 !!!")

        elif self.state == "ARRESTED":
            self.pub_police.publish(Twist())
            self.pub_intruder.publish(Twist())
            
            if time.time() - self.arrest_time > 2.0:
                self.get_logger().info("清除現場，重置追逐...")
                self.reset_intruder_scenario(first_run=False)

def main(args=None):
    rclpy.init(args=args)
    node = PoliceChaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()