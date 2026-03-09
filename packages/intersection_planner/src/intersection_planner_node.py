#!/usr/bin/env python3
import rospy
from duckietown_msgs.msg import BoolStamped, TurnIDandType, SegmentList, Segment
from duckietown.dtros import DTROS, NodeType
import random

"""Based on the detected intersection, decide which turn to take at random. A small node made to assist autonomous turning decisions in the AT_STOP_LINE state."""""

class IntersectionPlannerNode(DTROS):
    def __init__(self, node_name):
        super(IntersectionPlannerNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.CONTROL,
            fsm_controlled = True
        )

        #Publishes for unicorn_intersection node. Queue_size = 1 means keep most recent message - we want to keep latest camera detections and not old ones
        self.pub_turn_id_and_type = rospy.Publisher("unicorn_intersection_node/turn_id_and_type", TurnIDandType, queue_size = 1)
        
        #Subscribes to line_detector_node Segment List. cb_segment_list defined below. 
        self.sub_segment_list = rospy.Subscriber("line_detector_node/segment_list", SegmentList, self.cb_segment_list)

        self.valid_turns = []

    """Identifies if the intersection is 3 or 4 way. X and Y are currently placeholders, run them by running rostopic echo line_detector_node/segment_list when stopping at an intersection and check actual x/y values"""
    def cb_segment_list(self, msg):
        #Ignore all incoming segment messages if FSM hasn't activated
        if not self.switch:
            return
        
        has_left = False
        has_straight = False
        has_right = False

        #Calculate the average x and y position of the segment (midpoint)
        for segment in msg.segments:
            x = (segment.pixels_normalized[0].x + segment.pixels_normalized[1].x)/2
            y = (segment.pixels_normalized[0].y + segment.pixels_normalized[1].y)/2

            if y > 0.45: #Ignores bottom half of the image - shows the road right in front of the bot and not the intersection ahead
                continue

            #Bucket by color and x position
            if segment.color == Segment.WHITE:
                if x < 0.35:
                    has_left = True
                elif x > 0.65:
                    has_right = True
            elif segment.color == Segment.YELLOW:
                if 0.35 <= x <= 0.65:
                    has_straight = True
            
        #updates self.valid_turns with integer turn codes
        self.valid_turns = []
        if has_left: self.valid_turns.append(0)
        if has_straight: self.valid_turns.append(1)
        if has_right: self.valid_turns.append(2)


    def on_switch_on(self):
        rospy.sleep(1.0)

        #Fallback if no valid turns after 1 second
        if not self.valid_turns:
            self.valid_turns = [0, 1, 2] 
        
        #Pick random turn
        chosen_turn = random.choice(self.valid_turns) 

        msg = TurnIDandType()
        msg.turn_type = chosen_turn
        self.pub_turn_id_and_type.publish(msg)

        self.valid_turns = [] #Reset

if __name__ == "__main__":
    node = IntersectionPlannerNode(node_name="intersection_planner_node")
    rospy.spin()