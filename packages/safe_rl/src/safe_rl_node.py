#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import random
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped, FSMState
from geometry_msgs.msg import Polygon, Point32
from twin_delayed import TD3
import threading
import torch
import os

"""The node that deploys the trained RL model"""

class SafeRLNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
        )

        self.tof_distance = float('inf')
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.obstacle_detected = False
        self.object_avoided = False
        self.lane_offset = 0.0
        self.lane_heading = 0.0
        self.state = None 

        self.state_dim = 3
        self.action_dim = 2
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)

        weights_path = rospy.get_param("~weights_path", "/data/safe_rl_weights")
        self.load_weights(weights_path)

        self.sub_obst_detected = rospy.Subscriber("tof_obstacle_detection_node/obstacle_detected", BoolStamped, self.cb_obstacle_detected, queue_size=1)
        self.sub_obst_cleared = rospy.Subscriber("tof_obstacle_detection_node/obstacle_cleared", BoolStamped, self.cb_obstacle_cleared, queue_size=2)
        self.sub_tof = rospy.Subscriber("tof_obstacle_detection_node/front_center_tof/range", Range, self.cb_tof_range)
        self.sub_avoidance_done = rospy.Subscriber("avoiders_controller_node/avoidance_done", BoolStamped, self.cb_avoidance_done)
        self.sub_lane = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.cb_lane)
        self.sub_car_cmd = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cb_car_cmd)
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cb_state_change)  # ← deduplicated

        self.pub_object_avoided = rospy.Publisher("~object_avoided", BoolStamped, queue_size=1)
        self.pub_avoidance_path = rospy.Publisher("avoiders_controller_node/avoidance_path", Polygon, queue_size=1)
        self.pub_collision = rospy.Publisher("~collision_detected", BoolStamped, queue_size=1)  # ← added

    def load_weights(self, path):
        try:
            self.agent.actor.load_state_dict(
                torch.load(os.path.join(path, "actor.pth")))
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

    def cb_avoidance_done(self, avoidance_msg):
        if not self.switch:
            return
        if avoidance_msg.data:
            self.object_avoided = True

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

    def state_observation(self):
        return np.array([
            self.tof_distance,
            self.lane_offset,
            self.current_velocity
        ])

    def check_collision(self):  # ← added
        COLLISION_DISTANCE = 0.05
        if self.tof_distance <= COLLISION_DISTANCE:
            rospy.logwarn("[safe_rl] Collision detected!")
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_collision.publish(msg)
            return True
        return False

    def check_obstacle_cleared(self):
        CRITICAL_DISTANCE = 0.2
        if self.object_avoided and self.tof_distance > CRITICAL_DISTANCE:
            return True
        return False

    def execute_action(self, action):
        v, omega = action[0], action[1]
        msg = Polygon()
        p1 = Point32(); p1.x = 0.2; p1.y = float(omega) * 0.1; p1.z = 0.0
        p2 = Point32(); p2.x = 0.4; p2.y = float(omega) * 0.2; p2.z = 0.0
        p3 = Point32(); p3.x = 0.6; p3.y = float(omega) * 0.3; p3.z = 0.0
        msg.points = [p1, p2, p3]
        self.pub_avoidance_path.publish(msg)

    def step(self):
        """Deployment step"""
        state = self.state_observation()
        action = self.agent.select_action(state)
        self.execute_action(action)

        rate = rospy.Rate(10)
        while not self.object_avoided and self.switch:
            if self.check_collision():  # ← added collision check in wait loop
                return False
            rate.sleep()

        done = self.check_obstacle_cleared()
        self.object_avoided = False

        if done:
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_object_avoided.publish(msg)

        return done

    def on_switch_on(self):
        rospy.loginfo("[safe_rl] switched on, starting RL loop")
        t = threading.Thread(target=self._rl_loop)
        t.daemon = True
        t.start()

    def _rl_loop(self):
        rospy.loginfo("[safe_rl] RL loop started")
        while self.switch:
            done = self.step()
            if done:
                rospy.loginfo("[safe_rl] obstacle cleared, returning to lane following")
                break

if __name__ == "__main__":
    node = SafeRLNode(node_name="safe_rl_node")
    rospy.spin()