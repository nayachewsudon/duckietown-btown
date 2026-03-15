#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import BoolStamped, TurnIDandType, SegmentList, Segment
from duckietown.dtros import DTROS, NodeType

"""The main RL agent."""

class SafeRLNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLNode, self).__init__(
            node_name=node_name,
            node_type = NodeType.CONTROL,
            fsm_controlled = True
        )

        #Topics from tof_obstacle_detection_node
        self.sub_obst_detected = rospy.Subscriber("tof_obstacle_detection_node/obstacle_detected", BoolStamped, queue_size = 1)
        self.sub_obst_cleared = rospy.Subscriber("tof_obstacle_detection_node/obstacle_cleared", BoolStamped, queue_size = 2)

        #Published Topics

    #Called exactly once per timestep
    def compute_reward():
        """
        CRITICAL_DISTANCE = 0.10 m #(example), used to define how close the bot can get to the object
        If car detects object:
            If car collides: -10
        If car distance < critical distance: 
                Reward = -1
            If car slows down: 
                Velocity change = current_velocity - previous_velocity
                If decelerating (velocity_change < 0): 
        Reward = +1
                If car stops: 
        Reward = +2
            If car distance > critical distance: 
                Reward = +1
            If car slows down: 
                Velocity change = current_velocity - previous_velocity
                If decelerating (velocity_change < 0): 
        Reward = +1
                If car stops: 
        Reward = +2
        If object is not there : 
            If car speeds up: reward = +1


        """

    def state_observation(): 
        pass
  