#!/usr/bin/env python3


import rospy
from cv_bridge import CvBridge
from duckietown.dtros import DTParam, DTROS, NodeType
from duckietown_msgs.msg import BoolStamped, StopLineReading, LEDPattern, FSMState
from duckietown_msgs.srv import ChangePattern, SetCustomLEDPattern
from sensor_msgs.msg import Range


class TOFObstacleDetectionNode(DTROS):
    """
    The ``TOFObstacleDetectorNode`` detects obstacles in front of the vehicle.

    This is designed to work with the ``lane_following_pedestrians.yaml`` FSM config file. If running
    in the duckiematrix you can also run the ``loop_pedestrians`` map file which has duckie
    pedestrians which cross the road.

    Args:
        node_name (:obj:`str`): a unique, descriptive name for the node that ROS will use

    Configuration:
        ~tof_distance_threshold (:obj:`float`): The minimum value for the tof range (distance to obstacle) before
        an obstacle is reported as present

    Subscriber:
        ~front_center_tof/range (:obj:`sensor_msgs.msg.Range`): The data coming from the TOF sensor

    Publisher
        ~

    """
    def __init__(self, node_name):
        # Initialize the DTROS parent class
        super(TOFObstacleDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION,
                                                    fsm_controlled=True)

        self._obstacle_present = False
        rospy.Subscriber('~front_center_tof/range', Range, self.cb_tof_range)
        self._tof_threshold = rospy.get_param("~tof_distance_threshold")

        self.pub_obst_detected = rospy.Publisher("~obstacle_detected", BoolStamped, queue_size=1)
        self.pub_obst_cleared = rospy.Publisher("~obstacle_cleared", BoolStamped, queue_size=1)


    def cb_tof_range(self, msg: Range):
        obstacle_detected = False
        if msg.range < self._tof_threshold:
            obstacle_detected = True
        if obstacle_detected and not self._obstacle_present:
            # this means that we just detected this obstacle and we should report it
            self._obstacle_present = True
            obstacle_detected_msg = BoolStamped()
            obstacle_detected_msg.header = msg.header
            obstacle_detected_msg.data = True
            self.pub_obst_detected.publish(obstacle_detected_msg)
        if self._obstacle_present and not obstacle_detected:
            # this means that the obstacle was just cleared and we should report it
            self._obstacle_present = False
            obstacle_cleared_msg = BoolStamped()
            obstacle_cleared_msg.header = msg.header
            obstacle_cleared_msg.data = True
            self.pub_obst_cleared.publish(obstacle_cleared_msg)




if __name__ == "__main__":
    tof_obstacle_detection_node = TOFObstacleDetectionNode(node_name="tof_obstacle_detection_node")
    rospy.spin()
