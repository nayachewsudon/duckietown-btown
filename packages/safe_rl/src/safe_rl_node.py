#!/usr/bin/env python3
import os
import threading

import numpy as np
import rospy
import torch
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, FSMState, LanePose, Twist2DStamped
from geometry_msgs.msg import Point32, Polygon
from sensor_msgs.msg import Range

from twin_delayed import TD3

"""The node that deploys the trained RL model."""


class SafeRLNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
        )
        self._initialized = False
        self._loop_started = False

        self.tof_distance = float("inf")
        self.tof_min_range = 0.0
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.obstacle_detected = False
        self.obstacle_cleared = False
        self.awaiting_obstacle_clear = False
        self.object_avoided = False
        self.collision_detected = False
        self.collision_samples = 0
        self.lane_offset = 0.0
        self.lane_heading = 0.0
        self.state = None
        self.collision_distance = rospy.get_param("~collision_distance", 0.03)
        self.collision_count_threshold = rospy.get_param("~collision_count_threshold", 3)
        self.avoidance_lateral_scale = rospy.get_param("~avoidance_lateral_scale", 0.2)
        self.obstacle_clear_distance = rospy.get_param("~obstacle_clear_distance", 0.05)

        self.state_dim = 2
        self.action_dim = 1
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)

        weights_path = rospy.get_param("~weights_path", "/data/safe_rl_weights")
        self.load_weights(weights_path)

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
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cb_state_change)

        self.pub_object_avoided = rospy.Publisher("~object_avoided", BoolStamped, queue_size=1)
        self.pub_avoidance_path = rospy.Publisher("avoiders_controller_node/avoidance_path", Polygon, queue_size=1)
        self.pub_collision = rospy.Publisher("~collision_detected", BoolStamped, queue_size=1)
        self._initialized = True
        if self.switch:
            self.on_switch_on()

    def load_weights(self, path):
        try:
            self.agent.actor.load_state_dict(torch.load(os.path.join(path, "actor.pth")))
            rospy.loginfo(f"[safe_rl] Loaded weights from {path}")
        except Exception as e:
            rospy.logwarn(f"[safe_rl] Could not load weights: {e}")

    def cb_state_change(self, msg):
        self.state = msg.state

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
            "[safe_rl] front_center_tof=%.3fm (min=%.3fm)",
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
        if msg.data:
            self.obstacle_cleared = False
            self.awaiting_obstacle_clear = True

    def cb_obstacle_cleared(self, msg):
        if not self.switch:
            return
        if msg.data and self.awaiting_obstacle_clear:
            self.obstacle_detected = False
            self.obstacle_cleared = True
            self.awaiting_obstacle_clear = False

    def state_observation(self):
        return np.array([
            self.tof_distance,
            self.lane_offset,
        ])

    def collision_threshold(self):
        return self.collision_distance

    def publish_collision(self):
        if self.collision_detected:
            return
        self.collision_detected = True
        rospy.logwarn(
            "[safe_rl] Collision detected at %.3fm (threshold %.3fm)",
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

    def check_obstacle_cleared(self):
        if self.object_avoided and (self.obstacle_cleared or self.tof_distance > self.obstacle_clear_distance):
            return True
        return False

    def execute_action(self, action):
        # action is now 1D: [omega]
        if isinstance(action, (list, tuple, np.ndarray)):
            omega = float(action[0])
        else:
            omega = float(action)

        msg = Polygon()
        p1 = Point32()
        p1.x = 0.2
        p1.y = float(omega) * self.avoidance_lateral_scale
        p1.z = 0.0

        p2 = Point32()
        p2.x = 0.4
        p2.y = float(omega) * (2.0 * self.avoidance_lateral_scale)
        p2.z = 0.0

        p3 = Point32()
        p3.x = 0.6
        p3.y = float(omega) * (3.0 * self.avoidance_lateral_scale)
        p3.z = 0.0

        msg.points = [p1, p2, p3]
        self.pub_avoidance_path.publish(msg)

    def step(self):
        """Deployment step."""
        self.collision_detected = False
        self.collision_samples = 0
        self.obstacle_cleared = False
        self.awaiting_obstacle_clear = self.obstacle_detected
        self.object_avoided = False

        state = self.state_observation()
        action = self.agent.select_action(state)
        self.execute_action(action)

        rate = rospy.Rate(10)
        while self.switch:
            if self.check_collision():
                return "collision"
            if self.object_avoided and self.check_obstacle_cleared():
                break
            rate.sleep()

        if self.check_obstacle_cleared():
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_object_avoided.publish(msg)
            return "cleared"

        return "running"

    def on_switch_on(self):
        if not self._initialized:
            rospy.logwarn("[safe_rl] switch-on received before initialization finished")
            return
        if self._loop_started:
            return
        self._loop_started = True
        rospy.loginfo("[safe_rl] switched on, starting RL loop")
        t = threading.Thread(target=self._rl_loop)
        t.daemon = True
        t.start()

    def _rl_loop(self):
        rospy.loginfo("[safe_rl] RL loop started")
        while self.switch:
            outcome = self.step()
            if outcome == "cleared":
                rospy.loginfo("[safe_rl] obstacle cleared, returning to lane following")
                break
            if outcome == "collision":
                rospy.loginfo("[safe_rl] collision reported, yielding control to FSM")
                break
        self._loop_started = False


if __name__ == "__main__":
    node = SafeRLNode(node_name="safe_rl_node")
    rospy.spin()
