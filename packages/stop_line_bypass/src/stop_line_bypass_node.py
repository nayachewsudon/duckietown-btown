#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import BoolStamped, FSMState
from duckietown.dtros import DTROS, NodeType

class StopLineBypassNode(DTROS):
    def __init__(self, node_name):
        super(StopLineBypassNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
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

        self.sub_mode = rospy.Subscriber(
            "fsm_node/mode",
            FSMState,
            self.cb_state_change
        )
        
        self.timer = None
        self.cooldown = False
        self.state = None  # ← add this

    def cb_at_stop_line(self, msg):
        if not self.switch or self.cooldown:  # ← fixed
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
        self.cooldown = True

        rospy.Timer(
            rospy.Duration(3.0),
            self.reset_cooldown,
            oneshot=True
        )
    
    def reset_cooldown(self, _):
        self.cooldown = False
        rospy.loginfo("[bypass] Cooldown reset, ready for next stop line")

    def cb_state_change(self, msg):
        self.state = msg.state

if __name__ == "__main__":
    node = StopLineBypassNode(node_name="stop_line_bypass_node")
    rospy.spin()