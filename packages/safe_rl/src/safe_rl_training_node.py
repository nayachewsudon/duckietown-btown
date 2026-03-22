#!/usr/bin/env python3
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import random
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped, FSMState
from geometry_msgs.msg import Polygon, Point32
from twin_delayed import TD3, ReplayBuffer
import threading
import torch
import os

"""The main RL training agent."""

class SafeRLTrainingNode(DTROS):
    def __init__(self, node_name):
        super(SafeRLTrainingNode, self).__init__(
            node_name=node_name,
            node_type = NodeType.CONTROL,
            #fsm_controlled = True
        )

        #Variables
        self.tof_distance = float('inf')
        self.current_velocity = 0.0
        self.previous_velocity = 0.0
        self.obstacle_detected = False
        self.object_avoided = False
        self.reward = 0
        self.lane_offset = 0.0
        self.lane_heading = 0.0

        #Variables for training
        self.episode_count = 0
        self.max_episodes = rospy.get_param("~max_episodes", 30)
        self.start_timesteps = rospy.get_param("~start_timesteps", 20)
        self.total_timesteps = 0
        self.std_noise = rospy.get_param("~std_noise", 0.1)
        self.weights_path = rospy.get_param("~weights_path", "/data/safe_rl_weights") 

        #Timeout variables
        self.episode_start_time = 0.0
        self.episode_timeout = rospy.get_param("~episode_timeout", 10.0) #Timeout to eps reset in 10 seconds
        
        #RL components
        self.state_dim = 3 #[tof_distance, lane_offset, current_velocity]
        self.action_dim = 2 #[v, omega]
        self.max_action = 1.0
        self.agent = TD3(self.state_dim, self.action_dim, self.max_action)
        self.replay_buffer = ReplayBuffer()

        #Subscribers from tof_obstacle_detection_node
        self.sub_obst_detected = rospy.Subscriber("tof_obstacle_detection_node/obstacle_detected", BoolStamped, self.cb_obstacle_detected, queue_size = 1)
        self.sub_obst_cleared = rospy.Subscriber("tof_obstacle_detection_node/obstacle_cleared", BoolStamped, self.cb_obstacle_cleared, queue_size = 2)
        self.sub_tof = rospy.Subscriber("tof_obstacle_detection_node/front_center_tof/range", Range, self.cb_tof_range)

        #Subscribers from avoider
        self.sub_avoidance_done = rospy.Subscriber("avoiders_controller_node/avoidance_done", BoolStamped, self.cb_avoidance_done)
        self.sub_lane = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.cb_lane)

        #Subscribers from car_cmd
        self.sub_car_cmd = rospy.Subscriber("lane_controller_node/car_cmd", Twist2DStamped, self.cb_car_cmd)

        self.state = None
        self.sub_mode = rospy.Subscriber("fsm_node/mode", FSMState, self.cb_state_change)
        #Publishers: 
        self.pub_object_avoided = rospy.Publisher("~object_avoided", BoolStamped, queue_size=1)
        self.pub_avoidance_path = rospy.Publisher("avoiders_controller_node/avoidance_path", Polygon, queue_size=1)
        self.pub_collision = rospy.Publisher("~collision_detected", BoolStamped, queue_size=1)
        self.pub_timeout = rospy.Publisher("~timeout", BoolStamped, queue_size=1)

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

    def cb_state_change(self, msg):
        self.state = msg.state
    
    def compute_reward(self):

        CRITICAL_DISTANCE = 0.2
        COLLISION_DISTANCE = 0.05

        if self.obstacle_detected: 
            if self.tof_distance <= COLLISION_DISTANCE:
                self.reward = -10

                #Publish collision for episode reset
                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = True
                self.pub_collision.publish(msg)

            elif self.tof_distance < CRITICAL_DISTANCE:
                self.reward = -1
            elif self.tof_distance > CRITICAL_DISTANCE:
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
    
    def step(self):
        """Training step with exploration noise, replay buffer, and training"""
        #1. Observe current state
        state = self.state_observation()

        #Early episodes: random actions to fill replay buffer with diverse experience
        if self.total_timesteps < self.start_timesteps: 
            action = np.array([
                random.uniform(-self.max_action, self.max_action),
                random.uniform(-self.max_action, self.max_action)
            ])
        else:
            #TD3 acction + Gaussian noise for exploration (based on https://medium.com/@amit25173/reinforcement-learning-in-continuous-action-spaces-4fc60897fa55)
            action = self.agent.select_action(state)
            noise = np.random.normal(0, self.std_noise, size=self.action_dim)
            action = (action + noise).clip(-self.max_action, self.max_action)
        
        # Execute and wait until avoider finishes maneuver
        self.execute_action(action)
        self.total_timesteps +=1

        rate = rospy.Rate(10)
        while not self.object_avoided and self.switch:
            if rospy.get_time() - self.episode_start_time > self.episode_timeout: 
                rospy.loginfo("[safe_rl_training] Episode timed out")
                msg = BoolStamped()
                msg.header.stamp = rospy.Time.now()
                msg.data = True
                self.pub_timeout.publish(msg)
                return True

            rate.sleep()
        
        #Observe new state
        new_state = self.state_observation()

        #Compute reward
        self.compute_reward()
        reward = self.reward

        #Check if obstacle is cleared
        done = self.check_obstacle_cleared()

        #Stores full experience tuple (s, a, r, s') in replay buffer, which is what TD3 learns from
        self.replay_buffer.add((state, new_state, action, reward, done))

        #Save weights every 15 timesteps so we don't lose progress if bot crashes or shuts down
        if self.total_timesteps % 15 == 0:
            self.save_weights()

        #Train agent only after 20 experiences are collected 
        if len(self.replay_buffer.storage) > 20: 
            self.agent.train(self.replay_buffer, iterations=1)

        # Reset flags for next step
        self.object_avoided = False
        self.reward = 0

        # Publish object_avoided
        if done: #Check_obstacle_cleared() returned True = avoiedr done and ToF clear
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_object_avoided.publish(msg) #We tell FSM it's safe to return to lane following

        return done
    
    def execute_action(self, action):
        """Takes TD3 agent's output [v, omega], converts it into 3 physical waypoints that the avoider node can follow
        omega = 0 (go straight)
        omega = 1.0 (turn left)
        omega = -1.0 (turn right)
        
        TD3 outputs action = [v, omega]
        → execute_action converts to 3 waypoints
            → publishes Polygon to avoider
                → avoider drives through waypoints
                    → bot physically maneuvers around obstacle"""
        #action = [v, omega]
        #convert to 3 waypoints for avoider
        v, omega = action[0], action[1]

        msg = Polygon()
        p1 = Point32() #represents a 3D point with x, y, z coordinates
        p1.x = 0.2
        p1.y = float(omega) * 0.1
        p1.z = 0.0

        p2 = Point32()
        p2.x = 0.4
        p2.y = float(omega) *0.2
        p2.z = 0.0

        p3 = Point32()
        p3.x = 0.6
        p3.y = float(omega) * 0.3
        p3.z = 0.0

        msg.points = [p1, p2, p3]
        self.pub_avoidance_path.publish(msg)


    def check_obstacle_cleared(self):
        CRITICAL_DISTANCE = 0.2
        if self.object_avoided and self.tof_distance > CRITICAL_DISTANCE:
            return True #TODO: check episode is done, back to lane following
        return False
    
    def save_weights(self):
        os.makedirs(self.weights_path, exist_ok=True)
        torch.save(self.agent.actor.state_dict(),
                   os.path.join(self.weights_path, "actor.pth"))
        torch.save(self.agent.critic.state_dict(),
                   os.path.join(self.weights_path, "critic.pth"))
        rospy.loginfo(f"[safe_rl_training] Saved weights to {self.weights_path}")
    
    def on_switch_on(self):
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
        self.episode_count +=1
        rospy.loginfo(f"[safe_rl_training] Episode {self.episode_count}/{self.max_episodes} started")
        
        if self.episode_count >= self.max_episodes:
            rospy.loginfo("[safe_rl_training] Max episodes reached!")
            self.save_weights()
            return
        
        while self.switch:
            #"""Timeout logic published"""
            #if rospy.get_time() - self.episode_start_time > self.episode_timeout:
            #    rospy.loginfo("[safe_rl training] Episode timed out")
            #    msg = BoolStamped()
            #    msg.header.stamp = rospy.Time.now()
            #    msg.data = True
            #    self.pub_timeout.publish(msg)
            #    break

            done = self.step()
            if done:
                if self.object_avoided:
                    rospy.loginfo("[safe_rl] obstacle cleared successfully")
                else:
                    rospy.loginfo("[safe_rl] episode ended (timeout or collision)")
                break

        #Saves every episode
        #To restart from scratch, delete the weights using: rm -rf /data/safe_rl_weights/
        if len(self.replay_buffer.storage) > 0:
            self.save_weights()

        
        rospy.loginfo(f"[safe_rl_training] Episode {self.episode_count} ended, total timesteps: {self.total_timesteps}")
    
if __name__ == "__main__":
    node = SafeRLTrainingNode(node_name="safe_rl_training_node")
    rospy.spin()