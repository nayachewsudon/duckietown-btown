#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType
import numpy as np
import random
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped
from geometry_msgs.msg import Polygon
from twin_delayed import TD3, ReplayBuffer

"""The main RL agent."""

class SafeRLNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLNode, self).__init__(
            node_name=node_name,
            node_type = NodeType.CONTROL,
            fsm_controlled = True
        )

        #Topics from tof_obstacle_detection_node
        self.sub_obst_detected = rospy.Subscriber("tof_obstacle_detection_node/obstacle_detected", BoolStamped, self.cb_obstacle_detected, queue_size = 1)
        self.sub_obst_cleared = rospy.Subscriber("tof_obstacle_detection_node/obstacle_cleared", BoolStamped, self.cb_obstacle_cleared, queue_size = 2)
        self.sub_tof = rospy.Subscriber("tof_obstacle_detection_node/front_center_tof/range", Range, self.cb_tof_range)

        #Topic from avoider
        self.sub_avoidance_done = rospy.Subscriber("avoiders_controller_node/avoidance_done", BoolStamped, self.cb_avoidance_done)
        self.sub_lane = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.cb_lane)

        #Topic from car_cmd
        self.sub_car_cmd = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cb_car_cmd)

        #Publisher topic: 
        self.pub_object_avoided = rospy.Publisher("~object_avoided", BoolStamped, queue_size=1)
        #Variables
        self.tof_distance = float('inf')
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.obstacle_detected = False
        self.object_avoided = False
        self.reward = 0

        #RL components
        self.state_dim = 3 #[tof_distance, lane_offset, current_velocity]
        self.action_dim = 2 #[v, omega]
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)
        self.replay_buffer = ReplayBuffer()

    def cb_lane(self, lane_msg):
        if not self.switch: 
            return
        self.lane_offset = lane_msg.d #Should I also add lane_offset and lane_heading in variables in __init__?
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

    #Called exactly once per timestep
    def compute_reward(self):
        """
        CRITICAL_DISTANCE = 0.20 m, as defined in obstacledetection/config/default.yaml
        COLLISION_DISTANCE = 0.05
        If object_detected:
            If tof_distance <= COLLISION DISTANCE: --> extreme case of being too close
                Reward = -10
            elif tof distance < CRITICAL DISTANCE: 
                Reward = -1
                
            If car distance > critical distance: 
                reward = +10

        If object not detected : 
            If current velocity > previous velocity (we want the car to accelerate a little) : 
                Reward = +1

        If object_avoided (from avoiders node):
            Reward = +10

        """

        CRITICAL_DISTANCE = 0.2
        COLLISION_DISTANCE = 0.05

        if self.obstacle_detected: 
            if self.tof_distance <= COLLISION_DISTANCE:
                self.reward = -10
            elif self.tof_distance < CRITICAL_DISTANCE:
                self.reward = -1
            
            if self.tof_distance > CRITICAL_DISTANCE:
                self.reward = +10
        
        if self.object_avoided:
            self.reward =+10

        if not self.obstacle_detected:
            if self.current_velocity > self.previous_velocity:
                self.reward =+1

    #Packages sensor readings into a state vector
    def state_observation(self): 
        return np.array([
            self.tof_distance,
            self.lane_offset,
            self.current_velocity
        ])
    
    #TODO: Creating the main RL Loop
    def step(self):
        #1. Observe current state
        state = self.state_observation()

        #2. Agent picks action
        action = self.agent.select_action(state)

        #3. Execute action -> send waypoints to avoider
        self.execute_action(action)

        #4. Wait for avoider to finish
        rate = rospy.Rate(10)
        while not self.object_avoided and self.switch:
            rate.sleep()
        
        #5. Observe new state
        new_state = self.state_observation()

        #6. Compute reward
        self.compute_reward()
        reward = self.reward

        #7. Check if obstacle is cleared
        done = self.check_obstacle_cleared()

        #8. Store in replay buffer
        self.replay_buffer.add((state, new_state, action, reward, done))

        #9. Train agent
        if len(self.replay_buffer.storage) > 100: 
            self.agent.train(self.replay_buffer, iterations=1)

        #10. Reset flags for next step
        self.object_avoided = False
        self.reward = 0

        # 11. 
        if done: #Check_obstacle_cleared() returned True = avoiedr done and ToF clear
            msg - BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_object_avoided.publish(msg) #We tell FSM it's safe to return to lane following

        return done

    def execute_action(self, action):
        #action = [v, omega]
        #convert to 3 waypoints for avoider
        v, omega = action[0], action[1]

        msg = Polygon()
        p1 = Point32()
        p1.x = 0.2


    def check_obstacle_cleared(self):
        CRITICAL_DISTANCE = 0.2
        if self.object_avoided and self.tof_distance > CRITICAL_DISTANCE:
            return True #episode is done, back to lane following
        return False
