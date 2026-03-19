#!/usr/bin/env python3
"""
Unit test script for safe_rl_training_node.
Tests that all required topics are publishing correctly before running the RL node.

Run this on the bot with:
    python3 test_safe_rl_topics.py

Make sure the bot is running (fsm_lane_following.launch) before running this script.
"""

import rospy
import sys
import time
from sensor_msgs.msg import Range
from duckietown_msgs.msg import Twist2DStamped, LanePose, BoolStamped
from geometry_msgs.msg import Polygon

# ─────────────────────────────────────────────
# Test results tracker
# ─────────────────────────────────────────────
results = {}

def pass_test(name, msg=""):
    results[name] = ("PASS", msg)
    print(f"  ✅ PASS | {name}" + (f" — {msg}" if msg else ""))

def fail_test(name, msg=""):
    results[name] = ("FAIL", msg)
    print(f"  ❌ FAIL | {name}" + (f" — {msg}" if msg else ""))

def warn_test(name, msg=""):
    results[name] = ("WARN", msg)
    print(f"  ⚠️  WARN | {name}" + (f" — {msg}" if msg else ""))

# ─────────────────────────────────────────────
# Topic checkers
# ─────────────────────────────────────────────

def check_topic(topic, msg_type, timeout=5.0):
    """Wait for a single message on a topic and return it, or None on timeout."""
    try:
        msg = rospy.wait_for_message(topic, msg_type, timeout=timeout)
        return msg
    except rospy.ROSException:
        return None

# ─────────────────────────────────────────────
# Individual tests
# ─────────────────────────────────────────────

def test_tof_range():
    print("\n[1] Testing ToF range topic...")
    topic = "tof_obstacle_detection_node/front_center_tof/range"
    msg = check_topic(topic, Range)
    if msg is None:
        fail_test("tof_range", f"No message on {topic} within 5s — is tof_obstacle_detection_node running?")
        return
    if msg.range == float('inf') or msg.range <= 0:
        warn_test("tof_range", f"range={msg.range:.3f}m — unusual value, check sensor")
    else:
        pass_test("tof_range", f"range={msg.range:.3f}m")

def test_obstacle_detected():
    print("\n[2] Testing obstacle_detected topic...")
    topic = "tof_obstacle_detection_node/obstacle_detected"
    msg = check_topic(topic, BoolStamped)
    if msg is None:
        fail_test("obstacle_detected", f"No message on {topic} within 5s")
        return
    pass_test("obstacle_detected", f"data={msg.data}")

def test_obstacle_cleared():
    print("\n[3] Testing obstacle_cleared topic...")
    topic = "tof_obstacle_detection_node/obstacle_cleared"
    msg = check_topic(topic, BoolStamped)
    if msg is None:
        fail_test("obstacle_cleared", f"No message on {topic} within 5s")
        return
    pass_test("obstacle_cleared", f"data={msg.data}")

def test_lane_pose():
    print("\n[4] Testing lane_pose topic...")
    topic = "lane_filter_node/lane_pose"
    msg = check_topic(topic, LanePose)
    if msg is None:
        fail_test("lane_pose", f"No message on {topic} within 5s — is lane_filter_node running?")
        return

    d = msg.d
    phi = msg.phi

    # Check d value
    if abs(d) > 0.5:
        warn_test("lane_pose_d", f"d={d:.3f}m — bot may be far from lane center")
    else:
        pass_test("lane_pose_d", f"d={d:.3f}m")

    # Check phi value - THIS IS THE KEY CHECK after the extrinsic calibration fix
    if abs(phi) > 1.0:
        fail_test("lane_pose_phi", f"phi={phi:.3f}rad ({phi*57.3:.1f}deg) — CRITICAL: extrinsic calibration is wrong! phi should be close to 0")
    elif abs(phi) > 0.5:
        warn_test("lane_pose_phi", f"phi={phi:.3f}rad ({phi*57.3:.1f}deg) — slightly high, check calibration")
    else:
        pass_test("lane_pose_phi", f"phi={phi:.3f}rad ({phi*57.3:.1f}deg)")

def test_car_cmd():
    print("\n[5] Testing car_cmd topic...")
    topic = "lane_controller_node/car_cmd"
    msg = check_topic(topic, Twist2DStamped)
    if msg is None:
        fail_test("car_cmd", f"No message on {topic} within 5s — is lane_controller_node running?")
        return
    pass_test("car_cmd", f"v={msg.v:.3f} omega={msg.omega:.3f}")

def test_avoidance_done():
    print("\n[6] Testing avoidance_done topic...")
    topic = "avoiders_controller_node/avoidance_done"
    # This topic only publishes when avoider finishes - just check it exists
    msg = check_topic(topic, BoolStamped, timeout=2.0)
    if msg is None:
        warn_test("avoidance_done", f"No message on {topic} — normal if avoider hasn't run yet")
    else:
        pass_test("avoidance_done", f"data={msg.data}")

def test_avoidance_path_publishable():
    print("\n[7] Testing avoidance_path publisher...")
    topic = "avoiders_controller_node/avoidance_path"
    try:
        pub = rospy.Publisher(topic, Polygon, queue_size=1)
        rospy.sleep(0.5)  # wait for publisher to register
        
        # Send a test polygon
        msg = Polygon()
        from geometry_msgs.msg import Point32
        p1 = Point32(); p1.x = 0.2; p1.y = 0.1; p1.z = 0.0
        p2 = Point32(); p2.x = 0.4; p2.y = 0.2; p2.z = 0.0
        p3 = Point32(); p3.x = 0.6; p3.y = 0.3; p3.z = 0.0
        msg.points = [p1, p2, p3]
        pub.publish(msg)
        pass_test("avoidance_path_publisher", f"Successfully published test waypoints to {topic}")
    except Exception as e:
        fail_test("avoidance_path_publisher", str(e))

def test_object_avoided_publishable():
    print("\n[8] Testing object_avoided publisher...")
    topic = "/duckie2/safe_rl_training_node/object_avoided"
    try:
        pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        rospy.sleep(0.5)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        pub.publish(msg)
        pass_test("object_avoided_publisher", f"Successfully published to {topic}")
    except Exception as e:
        fail_test("object_avoided_publisher", str(e))

def test_collision_publishable():
    print("\n[9] Testing collision_detected publisher...")
    topic = "/duckie2/safe_rl_training_node/collision_detected"
    try:
        pub = rospy.Publisher(topic, BoolStamped, queue_size=1)
        rospy.sleep(0.5)
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        pub.publish(msg)
        pass_test("collision_publisher", f"Successfully published to {topic}")
    except Exception as e:
        fail_test("collision_publisher", str(e))

def test_reward_logic():
    print("\n[10] Testing reward logic (unit test, no ROS needed)...")
    import numpy as np

    CRITICAL_DISTANCE = 0.2
    COLLISION_DISTANCE = 0.05

    # Simulate compute_reward logic
    test_cases = [
        # (obstacle_detected, tof_distance, object_avoided, current_vel, prev_vel, expected_reward, description)
        (True, 0.03, False, 0.0, 0.0, -10, "collision distance → reward=-10"),
        (True, 0.15, False, 0.0, 0.0, -1,  "critical distance → reward=-1"),
        (True, 0.25, False, 0.0, 0.0, +10, "safe distance → reward=+10"),
        (False, 0.5, True,  0.0, 0.0, +10, "object avoided → reward=+10"),
        (False, 0.5, False, 0.3, 0.1, +1,  "accelerating, no obstacle → reward=+1"),
        (False, 0.5, False, 0.1, 0.3, 0,   "decelerating, no obstacle → reward=0"),
    ]

    all_passed = True
    for obstacle_detected, tof_distance, object_avoided, current_vel, prev_vel, expected, desc in test_cases:
        reward = 0

        if obstacle_detected:
            if tof_distance <= COLLISION_DISTANCE:
                reward = -10
            elif tof_distance < CRITICAL_DISTANCE:
                reward = -1
            elif tof_distance > CRITICAL_DISTANCE:
                reward = +10

        if object_avoided:
            reward = +10

        if not obstacle_detected:
            if current_vel > prev_vel:
                reward = +1

        if reward == expected:
            print(f"    ✅ {desc}")
        else:
            print(f"    ❌ {desc} — expected {expected}, got {reward}")
            all_passed = False

    if all_passed:
        pass_test("reward_logic", "All reward cases correct")
    else:
        fail_test("reward_logic", "Some reward cases failed")

# ─────────────────────────────────────────────
# Summary
# ─────────────────────────────────────────────

def print_summary():
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    passed = sum(1 for v in results.values() if v[0] == "PASS")
    warned = sum(1 for v in results.values() if v[0] == "WARN")
    failed = sum(1 for v in results.values() if v[0] == "FAIL")
    print(f"  ✅ Passed: {passed}")
    print(f"  ⚠️  Warned: {warned}")
    print(f"  ❌ Failed: {failed}")
    print("="*60)

    if failed > 0:
        print("\nFailed tests:")
        for name, (status, msg) in results.items():
            if status == "FAIL":
                print(f"  ❌ {name}: {msg}")

    if warned > 0:
        print("\nWarnings:")
        for name, (status, msg) in results.items():
            if status == "WARN":
                print(f"  ⚠️  {name}: {msg}")

# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────

if __name__ == "__main__":
    print("="*60)
    print("safe_rl_training_node Topic & Logic Tests")
    print("="*60)
    print("Make sure fsm_lane_following.launch is running on the bot!")
    print("="*60)

    rospy.init_node("safe_rl_test_node", anonymous=True)

    # Run all tests
    test_tof_range()
    test_obstacle_detected()
    test_obstacle_cleared()
    test_lane_pose()
    test_car_cmd()
    test_avoidance_done()
    test_avoidance_path_publishable()
    test_object_avoided_publishable()
    test_collision_publishable()
    test_reward_logic()

    print_summary()