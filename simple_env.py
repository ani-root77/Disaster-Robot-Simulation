import math
import time
import numpy as np
import gymnasium as gym

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty as EmptySrv


class DisasterSimpleEnv(gym.Env):
    metadata = {"render.modes": []}

    def __init__(
        self,
        collision_dist: float = 0.18,
        goal_pos=(6.0, 0.0),
        goal_radius: float = 0.6,
        linear_speed: float = 0.6,
        angular_speed: float = 1.0,
        lidar_size: int = 360,
        lidar_max_range: float = 10.0,
        max_steps: int = 300,
        debug: bool = False,
    ):
        super().__init__()

        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass
        self.node = Node("disaster_simple_env")

        # --- config ---
        self.collision_dist = float(collision_dist)
        self.goal_x, self.goal_y = goal_pos
        self.goal_radius = float(goal_radius)
        self.linear_speed = float(linear_speed)
        self.angular_speed = float(angular_speed)
        self.lidar_size = int(lidar_size)
        self.lidar_max_range = float(lidar_max_range)
        self.max_steps = int(max_steps)
        self.debug = debug

        # --- Gym spaces ---
        self.action_space = gym.spaces.Discrete(4)
        self.observation_space = gym.spaces.Box(
            low=0.0, high=self.lidar_max_range, shape=(4,), dtype=np.float32
        )

        # --- ROS topics ---
        self._scan = np.ones(self.lidar_size, dtype=np.float32) * self.lidar_max_range
        self.sub_scan = self.node.create_subscription(LaserScan, "/scan", self._scan_cb, 10)
        self.sub_odom = self.node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.pub_cmd = self.node.create_publisher(Twist, "/cmd_vel", 10)

        # --- Gazebo reset service ---
        self.reset_world_client = self.node.create_client(EmptySrv, "/reset_world")

        # --- state ---
        self.steps = 0
        self.pose = (0.0, 0.0)
        self.prev_distance = float("inf")
        self._angle_min = None
        self._angle_inc = None

        rclpy.spin_once(self.node, timeout_sec=0.05)

    # -------------------- Callbacks --------------------
    def _scan_cb(self, msg: LaserScan):
        r = np.array(msg.ranges, dtype=np.float32)
        bad = (~np.isfinite(r)) | (r <= 0.0)
        r[bad] = msg.range_max if hasattr(msg, "range_max") else self.lidar_max_range
        if r.size < self.lidar_size:
            tmp = np.ones(self.lidar_size, dtype=np.float32) * self.lidar_max_range
            tmp[: r.size] = r
            r = tmp
        elif r.size > self.lidar_size:
            r = r[: self.lidar_size]
        self._scan = np.clip(r, 0.0, self.lidar_max_range)
        self._angle_min = msg.angle_min
        self._angle_inc = msg.angle_increment

    def _odom_cb(self, msg: Odometry):
        self.pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

    # -------------------- Utilities --------------------
    def _publish(self, lin=0.0, ang=0.0):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.pub_cmd.publish(t)
        rclpy.spin_once(self.node, timeout_sec=0.02)

    def _stop(self, times=2):
        for _ in range(times):
            self._publish(0.0, 0.0)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _sector_min(self, center_deg: float, width_deg: float) -> float:
        if self._angle_min is None or self._angle_inc is None:
            idx_center = int(round(center_deg)) % self.lidar_size
            half = int(round(width_deg / 2.0))
        else:
            angle = math.radians(center_deg)
            idx_center = int(round((angle - self._angle_min) / self._angle_inc))
            half = int(round(math.radians(width_deg) / self._angle_inc))
        lo = max(0, idx_center - half)
        hi = min(self.lidar_size - 1, idx_center + half)
        return float(np.min(self._scan[lo:hi + 1]))

    def _observe(self):
        W = 60.0
        front = self._sector_min(0.0, W)
        left = self._sector_min(90.0, W)
        right = self._sector_min(-90.0, W)
        back = self._sector_min(180.0, W)
        return np.array([front, left, right, back], dtype=np.float32)

    def _collision(self) -> bool:
        if self.steps < 5:
            return False
        r = np.copy(self._scan)
        r = r[np.isfinite(r)]
        if r.size == 0:
            return False
        r_sorted = np.sort(r)
        min_smooth = np.mean(r_sorted[:10])
        front_half = r[: self.lidar_size // 2]
        front_min = np.mean(np.sort(front_half)[:10])
        return (min_smooth < self.collision_dist) or (front_min < self.collision_dist)

    def _reached_goal(self) -> bool:
        x, y = self.pose
        return math.hypot(x - self.goal_x, y - self.goal_y) < self.goal_radius

    def _distance_to_goal(self):
        x, y = self.pose
        return math.hypot(x - self.goal_x, y - self.goal_y)

    def _reset_gazebo_world(self):
        if self.reset_world_client.wait_for_service(timeout_sec=1.0):
            req = EmptySrv.Request()
            fut = self.reset_world_client.call_async(req)
            start = time.time()
            while rclpy.ok() and not fut.done():
                rclpy.spin_once(self.node, timeout_sec=0.05)
                if time.time() - start > 1.5:
                    break
            self.node.get_logger().info("[env] Gazebo world reset.")
        else:
            self.node.get_logger().warn("[env] /reset_world not available!")

    # -------------------- Reward Function --------------------
    def _calculate_reward(self, info):
        reward = 0.0
        distance_to_goal = info.get("distance_to_goal", float("inf"))
        prev_distance = info.get("prev_distance_to_goal", float("inf"))

        # progress reward
        if distance_to_goal < prev_distance:
            reward += 1.0
        else:
            reward -= 0.5

        # collision penalty
        if info.get("collision", False):
            reward -= 10.0

        # goal reward
        if info.get("reached_goal", False):
            reward += 100.0

        # time penalty
        reward -= 0.1
        return reward

    # -------------------- Gym API --------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._stop(times=4)
        self.steps = 0
        self.pose = (0.0, 0.0)
        self.prev_distance = float("inf")
        self._reset_gazebo_world()
        rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self._observe()
        return obs, {}

    def step(self, action: int):
        if action == 0:
            self._publish(self.linear_speed, 0.0)
        elif action == 1:
            self._publish(-self.linear_speed, 0.0)
        elif action == 2:
            self._publish(0.0, self.angular_speed)
        elif action == 3:
            self._publish(0.0, -self.angular_speed)
        else:
            self._publish(0.0, 0.0)

        rclpy.spin_once(self.node, timeout_sec=0.02)

        obs = self._observe()
        collided = self._collision()
        reached = self._reached_goal()

        dist_to_goal = self._distance_to_goal()
        info = {
            "collision": collided,
            "reached_goal": reached,
            "distance_to_goal": dist_to_goal,
            "prev_distance_to_goal": self.prev_distance,
        }

        reward = self._calculate_reward(info)
        self.prev_distance = dist_to_goal

        self.steps += 1
        terminated = collided or reached
        truncated = self.steps >= self.max_steps

        if self.debug:
            self.node.get_logger().info(
                f"[step {self.steps}] dist={dist_to_goal:.2f} coll={collided} goal={reached} reward={reward:.2f}"
            )

        if terminated or truncated:
            self._reset_gazebo_world()
            self._stop(times=4)

        import inspect
        caller = inspect.stack()[1].filename
        if "stable_baselines3" not in caller:
            # classic (obs, reward, done, info)
            done = terminated or truncated
            return obs, reward, done, info

        return obs, reward, terminated, truncated, info

    def close(self):
        try:
            self._stop(times=6)
            self._reset_gazebo_world()
        except Exception as e:
            self.node.get_logger().warn(f"[env.close] Exception: {e}")
        try:
            self.node.destroy_node()
        except Exception:
            pass


# -------- test run --------
if __name__ == "__main__":
    env = DisasterSimpleEnv(debug=True)
    obs, _ = env.reset()
    for i in range(200):
        a = env.action_space.sample()
        o, r, done, info = env.step(a)
        print(f"{i}: act={a} r={r:.1f} done={done} info={info}")
        if done:
            obs, _ = env.reset()
    env.close()
