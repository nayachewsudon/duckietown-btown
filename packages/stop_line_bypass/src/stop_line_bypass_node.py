#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import BoolStamped
from duckietown.dtros import DTROS, NodeType

class StopLineBypassNode(DTROS):
    def __init__(self, node_name):
        super(StopLineBypassNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
            fsm_controlled=True
        )
        
        self.pub = rospy.Publisher(
            "~intersection_done",
            BoolStamped,
            queue_size=1
        )
        
        self.sub = rospy.Subscriber(
            "stop_line_filter_node/at_stop_line",
            BoolStamped,
            self.cb_at_stop_line
        )
        
        self.timer = None

    def cb_at_stop_line(self, msg):
        if self.switch: 
            return
        if msg.data and self.timer is None:
            rospy.loginfo("[bypass] Stop line detected, will resume in 2 seconds")
            self.timer = rospy.Timer(
                rospy.Duration(2.0),
                self.publish_done,
                oneshot=True
            )

    def publish_done(self, _):
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.pub.publish(msg)
        rospy.loginfo("[bypass] Publishing intersection_done, resuming lane following")
        self.timer = None

if __name__ == "__main__":
    node = StopLineBypassNode(node_name = "stop_line_bypass_node")
    rospy.spin()