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
import numpy as np # PPO 數學運算需要

"""
專案名稱: Turtlesim Police Chase v16 (黃色警報版)
版本說明:
  1. [Visual Update] 逃跑模式路徑顏色: 改為 "黃色 (Yellow)" 且加粗，警示效果更強。
  2. [Spawn Logic] 生成距離優化: 每次重置時，強制確保入侵者生成在距離警察 > 5.0m 的位置。
"""

class PPOUtils:
    """
    PPO (Proximal Policy Optimization) 輔助類別
    """
    def __init__(self):
        self.gamma = 0.99
        self.lmbda = 0.95
        self.epsilon = 0.2 
        
    def get_state_vector(self, police_pose, intruder_pose):
        if not police_pose or not intruder_pose:
            return np.zeros(4)
            
        dx = intruder_pose.x - police_pose.x
        dy = intruder_pose.y - police_pose.y
        dist = math.sqrt(dx**2 + dy**2)
        angle = math.atan2(dy, dx) - intruder_pose.theta
        
        norm_dist = min(dist / 11.08, 1.0)
        norm_angle = angle / math.pi
        norm_wall_x = min(intruder_pose.x / 11.08, 1.0)
        norm_wall_y = min(intruder_pose.y / 11.08, 1.0)
        
        return np.array([norm_dist, norm_angle, norm_wall_x, norm_wall_y])

    def calculate_reward(self, distance, is_caught, near_wall, in_corner):
        reward = 0.0
        
        # 1. 生存與距離獎勵
        reward += 0.1
        reward += distance * 0.05
        
        # 2. 撞牆懲罰
        if near_wall:
            reward -= 0.5
            
        # 3. 角落懲罰
        if in_corner:
            reward -= 1.0
            
        # 4. 結束狀態回饋
        if is_caught:
            reward -= 10.0 
            
        return reward

    def sample_action(self, mean_action, std_dev=0.3):
        """
        隨機策略 (Stochastic Policy)
        """
        sampled_angular = np.random.normal(mean_action, std_dev)
        return sampled_angular

class PoliceChaseNode(Node):
    def __init__(self):
        super().__init__('police_chase_node')
        
        # --- 參數設定 ---
        self.catch_distance = 0.8     
        self.base_speed = 2.0         
        self.max_speed = 3.5          
        self.intruder_speed = 2.5     
        
        # FOV 設定
        self.fov_range = 5.0          
        self.fov_angle = math.radians(90) 
        
        self.state = "SEARCH"         
        self.arrest_time = 0.0

        # --- PPO 模組初始化 ---
        self.ppo = PPOUtils()
        self.episode_reward = 0.0
        self.step_count = 0

        # --- 變數初始化 ---
        self.police_pose = None
        self.intruder_pose = None
        
        self.last_intruder_pose = None
        self.last_time = time.time()
        self.intruder_velocity = (0.0, 0.0)
        
        # 警察巡邏路徑點
        self.police_waypoints = []
        self.police_wp_idx = 0
        
        # 入侵者逃跑路徑點 & 狀態
        self.intruder_waypoints = []
        self.intruder_wp_idx = 0
        self.intruder_mode = "NAVIGATE" # NAVIGATE or PANIC
        
        # --- Service Clients ---
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')
        self.police_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.intruder_pen_client = self.create_client(SetPen, '/intruder/set_pen')
        self.clear_client = self.create_client(EmptySrv, '/clear')
        self.param_client = self.create_client(SetParameters, '/turtlesim/set_parameters')
        
        # --- 初始化設定 ---
        self.set_background_black()
        self.reset_intruder_scenario(first_run=True)
        
        # --- Publishers & Subscribers ---
        self.pub_police = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_intruder = self.create_publisher(Twist, '/intruder/cmd_vel', 10)
        self.sub_police = self.create_subscription(Pose, '/turtle1/pose', self.police_callback, 10)
        self.sub_intruder = self.create_subscription(Pose, '/intruder/pose', self.intruder_callback, 10)

        # --- Timer ---
        self.timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info("智慧警匪追逐系統 v16 (黃色警報版) 已啟動！")

    def set_background_black(self):
        if not self.param_client.wait_for_service(timeout_sec=2.0): return
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='background_r', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
            Parameter(name='background_g', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0)),
            Parameter(name='background_b', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=0))
        ]
        future = self.param_client.call_async(req)
        self.clear_client.call_async(EmptySrv.Request())

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
    
    def generate_waypoints(self, is_police=False):
        """生成隨機路徑點"""
        waypoints = []
        count = 6 if is_police else 4 # 警察巡邏點多一點
        for _ in range(count):
            # 警察可以走得比較靠邊，入侵者稍微往中間一點
            margin = 1.0 if is_police else 1.5
            x = random.uniform(margin, 11.08 - margin)
            y = random.uniform(margin, 11.08 - margin)
            waypoints.append((x, y))
        return waypoints

    def reset_intruder_scenario(self, first_run=False):
        if not first_run and self.clear_client.service_is_ready():
            self.clear_client.call_async(EmptySrv.Request())

        kill_req = Kill.Request()
        kill_req.name = 'intruder'
        self.kill_client.call_async(kill_req) 
        
        time.sleep(0.5)

        # [Logic Upgrade] 生成策略優化
        # 初始嘗試生成座標
        spawn_x = random.uniform(2.0, 9.0)
        spawn_y = random.uniform(2.0, 9.0)

        # 如果已經有警察位置，確保生成的入侵者距離夠遠 (> 5.0m)
        if self.police_pose:
            min_safe_dist = 5.0
            for _ in range(10): # 最多嘗試 10 次，避免無限迴圈
                dx = spawn_x - self.police_pose.x
                dy = spawn_y - self.police_pose.y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > min_safe_dist:
                    break # 距離足夠，使用此座標
                
                # 距離太近，重新生成
                spawn_x = random.uniform(2.0, 9.0)
                spawn_y = random.uniform(2.0, 9.0)
            
            self.get_logger().info(f"生成位置確認: 距離警察 {dist:.2f}m")

        spawn_req = Spawn.Request()
        spawn_req.x = float(spawn_x)
        spawn_req.y = float(spawn_y)
        spawn_req.theta = random.uniform(0, 6.28)
        spawn_req.name = 'intruder'
        
        future = self.spawn_client.call_async(spawn_req)
        future.add_done_callback(self.spawn_callback)
        
        # 重置變數
        self.episode_reward = 0.0
        self.step_count = 0
        self.intruder_mode = "NAVIGATE"
        
        # 生成雙方路徑點
        self.intruder_waypoints = self.generate_waypoints(is_police=False)
        self.intruder_wp_idx = 0
        self.police_waypoints = self.generate_waypoints(is_police=True)
        self.police_wp_idx = 0
        self.get_logger().info(f"戰術更新: 警察巡邏點 x{len(self.police_waypoints)}, 入侵者逃脫點 x{len(self.intruder_waypoints)}")

    def spawn_callback(self, future):
        try:
            future.result()
            self.state = "SEARCH" 
            self.get_logger().info(">>> 新回合開始 (New Episode)")
            time.sleep(0.2)
            self.set_pen_color('intruder', 255, 0, 0, width=2)
            self.set_pen_color('turtle1', 0, 255, 0, width=2)
            self.last_intruder_pose = None
        except Exception as e:
            self.get_logger().error(f"生成失敗: {e}")
            self.state = "ARRESTED" 

    def police_callback(self, msg):
        self.police_pose = msg

    def intruder_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        if self.last_intruder_pose and dt > 0.001:
            vx = (msg.x - self.last_intruder_pose.x) / dt
            vy = (msg.y - self.last_intruder_pose.y) / dt
            self.intruder_velocity = (vx * 0.3 + self.intruder_velocity[0] * 0.7, 
                                      vy * 0.3 + self.intruder_velocity[1] * 0.7)
        self.last_intruder_pose = msg
        self.last_time = current_time
        self.intruder_pose = msg

    def get_distance(self):
        if not self.police_pose or not self.intruder_pose: return 999.0
        return math.sqrt((self.intruder_pose.x - self.police_pose.x)**2 + (self.intruder_pose.y - self.police_pose.y)**2)

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def check_fov(self):
        if not self.police_pose or not self.intruder_pose: return False
        dist = self.get_distance()
        if dist > self.fov_range: return False
        dx = self.intruder_pose.x - self.police_pose.x
        dy = self.intruder_pose.y - self.police_pose.y
        target_angle = math.atan2(dy, dx)
        angle_diff = abs(self.normalize_angle(target_angle - self.police_pose.theta))
        return angle_diff < (self.fov_angle / 2.0)

    def smart_move(self, twist_msg, current_speed, angle_diff, stop_threshold=1.0, current_pose=None):
        twist_msg.angular.z = 6.0 * angle_diff 
        
        # 邊界硬體鎖
        if current_pose:
            margin_limit = 0.5
            is_out_of_bounds = (current_pose.x < margin_limit or current_pose.x > 11.08 - margin_limit or
                                current_pose.y < margin_limit or current_pose.y > 11.08 - margin_limit)
            if is_out_of_bounds:
                current_speed *= 0.1 
        
        if abs(angle_diff) > stop_threshold:
            twist_msg.linear.x = 0.0
        else:
            denominator = max(1.0, stop_threshold)
            speed_factor = 1.0 - (abs(angle_diff) / denominator)
            twist_msg.linear.x = current_speed * max(0.2, speed_factor)
            
        return twist_msg

    def control_loop(self):
        if not self.police_pose or not self.intruder_pose: return

        police_cmd = Twist()
        intruder_cmd = Twist()
        distance = self.get_distance()
        is_visible = self.check_fov()
        
        current_state = self.ppo.get_state_vector(self.police_pose, self.intruder_pose)

        # ==========================
        # 1. 警察控制 logic
        # ==========================
        if self.state == "SEARCH":
            if is_visible:
                self.state = "CHASING"
                self.get_logger().warn("發現目標！鎖定！")
                self.set_pen_color('turtle1', 0, 255, 255, width=3)
            else:
                if self.police_waypoints:
                    target_x, target_y = self.police_waypoints[self.police_wp_idx]
                    dx = target_x - self.police_pose.x
                    dy = target_y - self.police_pose.y
                    dist_wp = math.sqrt(dx**2 + dy**2)
                    target_angle = math.atan2(dy, dx)
                    
                    if dist_wp < 0.5: 
                        self.police_wp_idx = (self.police_wp_idx + 1) % len(self.police_waypoints)
                    
                    angle_diff = self.normalize_angle(target_angle - self.police_pose.theta)
                    police_cmd = self.smart_move(police_cmd, 2.0, angle_diff, stop_threshold=0.5, current_pose=self.police_pose)
                else:
                    police_cmd.linear.x = 0.0

        elif self.state == "CHASING":
            if not is_visible and distance > 1.5:
                self.state = "SEARCH"
                self.get_logger().info("丟失目標 -> SEARCH")
                self.set_pen_color('turtle1', 0, 255, 0, width=2)
            else:
                lookahead_time = min(distance / self.max_speed, 1.0) 
                pred_x = self.intruder_pose.x + self.intruder_velocity[0] * lookahead_time
                pred_y = self.intruder_pose.y + self.intruder_velocity[1] * lookahead_time
                
                dx = pred_x - self.police_pose.x
                dy = pred_y - self.police_pose.y
                target_angle = math.atan2(dy, dx)
                angle_diff = self.normalize_angle(target_angle - self.police_pose.theta)
                
                chase_speed = self.base_speed + (distance * 0.8) 
                chase_speed = min(chase_speed, self.max_speed)
                
                police_cmd = self.smart_move(police_cmd, chase_speed, angle_diff, stop_threshold=1.0, current_pose=self.police_pose)

                if distance < self.catch_distance:
                    self.state = "ARRESTED"
                    self.arrest_time = time.time()
                    self.get_logger().warn(f"!!! 逮捕成功 !!! 總獎勵: {self.episode_reward:.1f}")

        elif self.state == "ARRESTED":
            if time.time() - self.arrest_time > 2.0:
                self.state = "RESETTING"
                self.reset_intruder_scenario(first_run=False)

        elif self.state == "RESETTING":
            pass

        # ==========================
        # 2. 入侵者控制 (Intruder Logic with Panic Mode)
        # ==========================
        if self.state in ["SEARCH", "CHASING"]:
            # Feature 1: 警察斥力 (Repulsive)
            dx = self.intruder_pose.x - self.police_pose.x
            dy = self.intruder_pose.y - self.police_pose.y
            dist_p = math.sqrt(dx**2 + dy**2)
            if dist_p > 0:
                vec_p_x, vec_p_y = (dx / dist_p) * 2.0, (dy / dist_p) * 2.0
            else:
                vec_p_x, vec_p_y = 0, 0
            
            # Feature 2: 角落彈射
            corner_margin = 1.5
            is_in_corner = False
            corner_vec_x, corner_vec_y = 0.0, 0.0
            
            in_left = self.intruder_pose.x < corner_margin
            in_right = self.intruder_pose.x > (11.08 - corner_margin)
            in_bot = self.intruder_pose.y < corner_margin
            in_top = self.intruder_pose.y > (11.08 - corner_margin)
            
            if (in_left and in_bot) or (in_right and in_bot) or (in_left and in_top) or (in_right and in_top):
                is_in_corner = True
                center_x, center_y = 5.54, 5.54
                dir_x = center_x - self.intruder_pose.x
                dir_y = center_y - self.intruder_pose.y
                norm = math.sqrt(dir_x**2 + dir_y**2)
                if norm > 0:
                    corner_vec_x = (dir_x / norm) * 8.0
                    corner_vec_y = (dir_y / norm) * 8.0
            
            # Feature 3: 基礎牆壁斥力
            vec_w_x, vec_w_y = 0.0, 0.0
            wall_margin = 1.5
            wall_gain = 5.0 

            if self.intruder_pose.x < wall_margin: vec_w_x += (wall_margin - self.intruder_pose.x) * wall_gain
            if self.intruder_pose.x > (11.08 - wall_margin): vec_w_x += - (self.intruder_pose.x - (11.08 - wall_margin)) * wall_gain
            if self.intruder_pose.y < wall_margin: vec_w_y += (wall_margin - self.intruder_pose.y) * wall_gain
            if self.intruder_pose.y > (11.08 - wall_margin): vec_w_y += - (self.intruder_pose.y - (11.08 - wall_margin)) * wall_gain

            # Feature 4: 路徑點吸引力 (Conditional)
            vec_wp_x, vec_wp_y = 0.0, 0.0
            
            # [Intruder Upgrade] 判斷是否處於危險狀態 (Panic Mode)
            # 距離小於 3.5m 視為危險，忽視路徑點
            is_threatened = (dist_p < 3.5)
            
            # [Visual Update] 狀態改變時切換顏色
            if is_threatened and self.intruder_mode != "PANIC":
                self.intruder_mode = "PANIC"
                self.get_logger().warn("入侵者恐慌！切換為逃跑模式 (黃色軌跡)")
                self.set_pen_color('intruder', 255, 255, 0, width=3) # 黃色，加粗
            elif not is_threatened and self.intruder_mode != "NAVIGATE":
                self.intruder_mode = "NAVIGATE"
                self.set_pen_color('intruder', 255, 0, 0, width=2) # 回復紅色
            
            if not is_threatened and self.intruder_waypoints:
                target_x, target_y = self.intruder_waypoints[self.intruder_wp_idx]
                dx_wp = target_x - self.intruder_pose.x
                dy_wp = target_y - self.intruder_pose.y
                dist_wp = math.sqrt(dx_wp**2 + dy_wp**2)
                
                if dist_wp < 0.5:
                    self.intruder_wp_idx = (self.intruder_wp_idx + 1) % len(self.intruder_waypoints)
                else:
                    # 正常導航
                    vec_wp_x = (dx_wp / dist_wp) * 3.0
                    vec_wp_y = (dy_wp / dist_wp) * 3.0
            
            # 如果處於危險狀態，增強警察斥力 (Panic Force)
            if is_threatened:
                vec_p_x *= 2.0 
                vec_p_y *= 2.0

            # 合成向量
            total_x = vec_p_x + vec_w_x + corner_vec_x + vec_wp_x
            total_y = vec_p_y + vec_w_y + corner_vec_y + vec_wp_y
            mean_angle = math.atan2(total_y, total_x)
            
            # PPO 適應性探索
            near_wall_critical = (self.intruder_pose.x < 0.8 or self.intruder_pose.x > 10.2 or 
                                  self.intruder_pose.y < 0.8 or self.intruder_pose.y > 10.2)
            
            adaptive_std = 0.1 if (near_wall_critical or is_in_corner or is_threatened) else 0.4
            
            final_angle = mean_angle + self.ppo.sample_action(mean_action=0.0, std_dev=adaptive_std)
            angle_diff = self.normalize_angle(final_angle - self.intruder_pose.theta)
            
            threshold = 0.2 if (near_wall_critical or is_in_corner) else 1.0
            intruder_cmd = self.smart_move(intruder_cmd, self.intruder_speed, angle_diff, stop_threshold=threshold, current_pose=self.intruder_pose)
            
            # PPO 獎勵計算
            is_caught = (distance < self.catch_distance)
            step_reward = self.ppo.calculate_reward(distance, is_caught, near_wall_critical, is_in_corner)
            self.episode_reward += step_reward
            self.step_count += 1
            
            if self.step_count % 20 == 0:
                mode_str = "PANIC" if is_threatened else "NAVIGATE"
                print(f"[PPO {mode_str}] Step: {self.step_count}, Reward: {step_reward:.2f}, Total: {self.episode_reward:.1f}")

        self.pub_police.publish(police_cmd)
        self.pub_intruder.publish(intruder_cmd)

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