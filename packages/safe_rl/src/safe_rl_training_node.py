#!/usr/bin/env python3
import os
import random
import threading

import numpy as np
import rospy
import torch
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, FSMState, LanePose, Twist2DStamped, WheelsCmdStamped
from geometry_msgs.msg import Point32, Polygon
from sensor_msgs.msg import Range

from twin_delayed import ReplayBuffer, TD3

"""The main RL training agent."""


class SafeRLTrainingNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLTrainingNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
        )
        self._initialized = False
        self._loop_started = False
        self.pub_wheels_stop = None

        self.tof_distance = float("inf")
        self.tof_min_range = 0.0
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.obstacle_detected = False
        self.object_avoided = False
        self.collision_detected = False
        self.collision_samples = 0
        self.timeout_detected = False
        self.episode_end_reason = None
        self.reward = 0
        self.lane_offset = 0.0
        self.lane_heading = 0.0

        self.episode_count = 0
        self.max_episodes = rospy.get_param("~max_episodes", 30)
        self.start_timesteps = rospy.get_param("~start_timesteps", 20)
        self.total_timesteps = 0
        self.std_noise = rospy.get_param("~std_noise", 0.1)
        self.weights_path = rospy.get_param("~weights_path", "/data/safe_rl_weights")

        self.episode_start_time = 0.0
        self.episode_timeout = rospy.get_param("~episode_timeout", 5.0)
        self.collision_distance = rospy.get_param("~collision_distance", 0.05)
        self.collision_count_threshold = rospy.get_param("~collision_count_threshold", 3)

        self.state_dim = 2
        self.action_dim = 1
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)
        self.replay_buffer = ReplayBuffer()

        self.sub_obst_detected = rospy.Subscriber(
            "tof_obstacle_detection_node/obstacle_detected",
            BoolStamped,
            self.cb_obstacle_detected,
            queue_size=1,
        )
        self.sub_obst_cleared = rospy.Subscriber(
            "tof_obstacle_detection_node/obstacle_cleared",
            BoolStamped,
            self.cb_obstacle_cleared,
            queue_size=2,
        )
        self.sub_tof = rospy.Subscriber(
            "front_center_tof_driver_node/range",
            Range,
            self.cb_tof_range,
        )
        self.sub_avoidance_done = rospy.Subscriber(
            "avoiders_controller_node/avoidance_done",
            BoolStamped,
            self.cb_avoidance_done,
        )
        self.sub_lane = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.cb_lane)
        self.sub_car_cmd = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cb_car_cmd)

        self.state = None
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cb_state_change)
        self.pub_object_avoided = rospy.Publisher("~object_avoided", BoolStamped, queue_size=1)
        self.pub_avoidance_path = rospy.Publisher("avoiders_controller_node/avoidance_path", Polygon, queue_size=1)
        self.pub_collision = rospy.Publisher("~collision_detected", BoolStamped, queue_size=1)
        self.pub_timeout = rospy.Publisher("~timeout", BoolStamped, queue_size=1)
        self.pub_wheels_stop = rospy.Publisher("wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
        self._initialized = True
        rospy.on_shutdown(self._on_shutdown)
        if self.switch:
            self.on_switch_on()

    def cb_lane(self, lane_msg):
        if not self.switch:
            return
        self.lane_offset = lane_msg.d
        self.lane_heading = lane_msg.phi

    def cb_tof_range(self, tof_msg):
        if not self.switch:
            return
        self.tof_distance = tof_msg.range
        self.tof_min_range = tof_msg.min_range
        rospy.loginfo_throttle(
            1.0,
            "[safe_rl_training] front_center_tof=%.3fm (min=%.3fm)",
            self.tof_distance,
            self.tof_min_range,
        )

    def cb_avoidance_done(self, avoidance_msg):
        if not self.switch:
            return
        if avoidance_msg.data:
            self.object_avoided = True
            rospy.loginfo("[safe_rl_training] cb_avoidance_done received: %s; tof=%.3f", avoidance_msg.data, self.tof_distance)

    def cb_car_cmd(self, cmd_msg):
        if not self.switch:
            return
        self.previous_velocity = self.current_velocity
        self.current_velocity = cmd_msg.v

    def cb_obstacle_detected(self, msg):
        if not self.switch:
            return
        self.obstacle_detected = msg.data

    def cb_obstacle_cleared(self, msg):
        if not self.switch:
            return
        if msg.data:
            self.obstacle_detected = False

    def cb_state_change(self, msg):
        self.state = msg.state
        if msg.state in ("EPISODE_RESET", "EMERGENCY_STOP"):
            rospy.loginfo("[safe_rl_training] FSM entered %s, publishing emergency stop", msg.state)
            self.publish_stop()

    def collision_threshold(self):
        return self.collision_distance

    def publish_collision(self):
        if self.collision_detected:
            return
        self.collision_detected = True
        rospy.logwarn(
            "[safe_rl_training] Collision detected at %.3fm (threshold %.3fm)",
            self.tof_distance,
            self.collision_threshold(),
        )
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.pub_collision.publish(msg)

    def check_collision(self):
        if self.tof_distance <= self.collision_threshold():
            self.collision_samples += 1
            if self.collision_samples >= self.collision_count_threshold:
                self.publish_collision()
                return True
        else:
            self.collision_samples = 0
        return False

    def compute_reward(self):
        critical_distance = 0.2
        collision = self.check_collision()

        if self.obstacle_detected:
            if collision:
                self.reward = -10
            elif self.tof_distance < critical_distance:
                self.reward = -1
            elif self.tof_distance > critical_distance:
                self.reward = +10

        if self.object_avoided:
            self.reward = +10
        # NOTE: removed velocity-based reward — velocity is not controlled
        # by the avoider in the current architecture (short-term fix).
        if self.timeout_detected:
            self.reward = -5

    def state_observation(self):
        return np.array([
            self.tof_distance,
            self.lane_offset,
        ])

    def publish_stop(self, repeat=5, sleep_s=0.02):
        pub_wheels_stop = getattr(self, "pub_wheels_stop", None)
        if pub_wheels_stop is None:
            rospy.logwarn("[safe_rl_training] wheel stop publisher unavailable, skipping emergency stop publish")
            return

        for _ in range(repeat):
            stop_msg = WheelsCmdStamped()
            stop_msg.header.stamp = rospy.Time.now()
            stop_msg.vel_left = 0.0
            stop_msg.vel_right = 0.0
            try:
                pub_wheels_stop.publish(stop_msg)
                rospy.sleep(sleep_s)
            except rospy.ROSException:
                break

    def step(self):
        """Training step with exploration noise, replay buffer, and training."""
        self.collision_detected = False
        self.collision_samples = 0
        self.timeout_detected = False
        self.object_avoided = False
        self.episode_end_reason = None

        state = self.state_observation()

        if self.total_timesteps < self.start_timesteps:
            action = np.array([random.uniform(-self.max_action, self.max_action)])
        else:
            action = self.agent.select_action(state)
            # ensure action is array-like, then add noise
            noise = np.random.normal(0, self.std_noise, size=self.action_dim)
            action = (np.array(action) + noise).clip(-self.max_action, self.max_action)

        self.execute_action(action)
        self.total_timesteps += 1

        rate = rospy.Rate(10)
        while self.switch:
            if rospy.get_time() - self.episode_start_time > self.episode_timeout:
                rospy.loginfo("[safe_rl_training] Episode timed out")
                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = True
                self.pub_timeout.publish(msg)
                self.episode_end_reason = "timeout"
                self.timeout_detected = True
                break
            if self.check_collision():
                self.episode_end_reason = "collision"
                break
            if self.object_avoided:
                break
            rate.sleep()

        new_state = self.state_observation()

        self.compute_reward()
        reward = self.reward

        done = self.timeout_detected or self.collision_detected or self.check_obstacle_cleared()
        if done and self.episode_end_reason is None:
            if self.timeout_detected:
                self.episode_end_reason = "timeout"
            else:
                self.episode_end_reason = "collision" if self.collision_detected else "cleared"

        self.replay_buffer.add((state, new_state, action, reward, done))

        if self.total_timesteps % 15 == 0:
            self.save_weights()

        if len(self.replay_buffer.storage) > 20:
            self.agent.train(self.replay_buffer, iterations=1)

        self.reward = 0

        if done and not self.collision_detected and not self.timeout_detected:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_object_avoided.publish(msg)

        return done

    def execute_action(self, action):
        """Convert [omega] into three waypoints for the avoider."""
        if isinstance(action, (list, tuple, np.ndarray)):
            omega = float(action[0])
        else:
            omega = float(action)

        msg = Polygon()
        p1 = Point32()
        p1.x = 0.2
        p1.y = float(omega) * 0.1
        p1.z = 0.0

        p2 = Point32()
        p2.x = 0.4
        p2.y = float(omega) * 0.2
        p2.z = 0.0

        p3 = Point32()
        p3.x = 0.6
        p3.y = float(omega) * 0.3
        p3.z = 0.0

        msg.points = [p1, p2, p3]
        self.pub_avoidance_path.publish(msg)

    def check_obstacle_cleared(self):
        critical_distance = 0.2
        if self.object_avoided and self.tof_distance > critical_distance:
            return True
        return False

    def save_weights(self):
        os.makedirs(self.weights_path, exist_ok=True)
        torch.save(self.agent.actor.state_dict(), os.path.join(self.weights_path, "actor.pth"))
        torch.save(self.agent.critic.state_dict(), os.path.join(self.weights_path, "critic.pth"))
        rospy.loginfo(f"[safe_rl_training] Saved weights to {self.weights_path}")

    def on_switch_on(self):
        if not self._initialized:
            rospy.logwarn("[safe_rl_training] switch-on received before initialization finished")
            return
        if self._loop_started:
            return
        self._loop_started = True
        """
        Called automatically by DTROS when the FSM activates this node
        (i.e., when FSM enters OBJECT_AVOIDANCE state).

        Starts the RL loop in a separate thread so that the DTROS switch
        service call returns immediately and the FSM is not blocked.
        """
        rospy.loginfo("[safe_rl] switched on, starting RL loop")
        t = threading.Thread(target=self._training_loop)
        t.daemon = True
        t.start()

    def _training_loop(self):
        """
        Training loop - one activation = one episode
        FSM handles episode resets via EPISODE_RESET state
        We save the weights needed for deployment in THIS FUNCTION, after max episodes are reached
        """

        self.episode_start_time = rospy.get_time()
        self.episode_count += 1
        rospy.loginfo(f"[safe_rl_training] Episode {self.episode_count}/{self.max_episodes} started")

        if self.episode_count >= self.max_episodes:
            rospy.loginfo("[safe_rl_training] Max episodes reached!")
            self.publish_stop()
            self.save_weights()
            self._loop_started = False
            return

        while self.switch:
            done = self.step()
            if done:
                self.publish_stop()
                rospy.loginfo(
                    "[safe_rl_training] episode ended, reason=%s",
                    self.episode_end_reason or "unknown",
                )
                break
        self._loop_started = False

        if len(self.replay_buffer.storage) > 0:
            self.save_weights()

        rospy.loginfo(f"[safe_rl_training] Episode {self.episode_count} ended, total timesteps: {self.total_timesteps}")

    def on_switch_off(self):
        rospy.loginfo("[safe_rl_training] switched off, publishing emergency stop")
        self.publish_stop()
        self._loop_started = False

    def _on_shutdown(self):
        rospy.loginfo("[safe_rl_training] shutdown requested, publishing emergency stop")
        self.publish_stop()


if __name__ == "__main__":
    node = SafeRLTrainingNode(node_name="safe_rl_training_node")
    rospy.spin()
