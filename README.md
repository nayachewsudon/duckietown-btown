# duckietown-btown

A Duckietown project running on a **DB21J** robot with the **daffy** software stack. This project extends autonomous lane following with RL-based obstacle avoidance using a Twin Delayed Deep Deterministic Policy Gradient (TD3) agent.

## Check out our Demo Video:
https://youtu.be/JUI11DJULSc

## What It Does

The robot operates using a Finite State Machine (FSM) with two main behaviors:

1. **Lane Following** — The robot follows the lane continuously using the standard Duckietown lane filter and controller pipeline. It drives straight through intersections without stopping.

2. **RL Obstacle Avoidance** — When the front-facing ToF (Time-of-Flight) sensor detects an obstacle, the FSM switches to `OBJECT_AVOIDANCE` state. A trained TD3 agent takes over and sends waypoints to an avoider controller node to navigate around the obstacle. Once the obstacle is cleared, the robot returns to lane following.

3. **Emergency Stop** — If a collision is imminent (ToF distance ≤ 5cm), the FSM transitions to `EMERGENCY_STOP`. The robot resumes lane following once the obstacle is cleared.

### FSM States

```
LANE_FOLLOWING ──(object_detected)──► OBJECT_AVOIDANCE
                                            │
                              (object_avoided)│(collision_detected)
                                            │        │
                                            ▼        ▼
                                     LANE_FOLLOWING  EMERGENCY_STOP
                                                          │
                                              (obstacle_cleared)│
                                                          ▼
                                                    LANE_FOLLOWING
```

### Packages

| Package | Description |
|---|---|
| `our_fsm` | Custom FSM configuration for deployment and training |
| `safe_rl` | TD3-based RL agent for obstacle avoidance (deployment + training nodes) |
| `obstacle_detection` | ToF-based obstacle detection node (ported from ente) |

---

## Prerequisites

- DB21J Duckiebot running **daffy** software stack
- Docker and `dts` (Duckietown Shell) installed on your laptop
- Robot and laptop on the same network
- Calibrated camera (intrinsic + extrinsic)

Verify your bot is reachable:
```bash
ping duckie2.local
```

---

## Setup

### 1. Clone the repository onto the bot

SSH into your bot and clone the repo:
```bash
ssh duckie@duckie2.local
git clone https://github.com/nayachewsudon/duckietown-btown.git
cd duckietown-btown
```

### 2. Verify robot config files

Make sure these files exist on the bot:
```bash
cat /data/config/robot_type    # should output: duckiebot
cat /data/config/robot_model   # should output: DB21J
```

If missing, create them:
```bash
echo "duckiebot" | sudo tee /data/config/robot_type
echo "DB21J" | sudo tee /data/config/robot_model
docker restart kvstore ros-interface device-proxy car-interface
```

### 3. Verify calibration

Make sure intrinsic and extrinsic calibration files exist:
```bash
ls /data/config/calibrations/camera_intrinsic/
ls /data/config/calibrations/camera_extrinsic/
```

If not calibrated, follow the [Duckietown calibration guide](https://docs.duckietown.com/daffy/opmanual-duckiebot/operations/calibration_camera/index.html).

---

## Launching

### Lane Following (with obstacle avoidance)

From your laptop, run the project inside the Duckietown Docker container:

```bash
dts duckiebot demo \
  --demo_name fsm_lane_following \
  --duckiebot_name duckie2 \
  --package_name duckietown_btown \
  --image duckietown/dt-core:daffy-arm32v7
```

Or if running directly on the bot via SSH:

```bash
roslaunch duckietown_btown fsm_lane_following.launch veh:=duckie2
```

### RL Training (optional)

To train the TD3 agent from scratch, use the training FSM:

```bash
roslaunch duckietown_btown fsm_lane_following.launch veh:=duckie2 fsm_file_name:=fsm_training
```

Trained weights are saved to `/data/safe_rl_weights/` on the bot every 50 timesteps.

---

## Key Topics

| Topic | Description |
|---|---|
| `lane_filter_node/lane_pose` | Lane pose estimate (d, phi) |
| `tof_obstacle_detection_node/obstacle_detected` | Obstacle detected by ToF sensor |
| `tof_obstacle_detection_node/obstacle_cleared` | Obstacle no longer in path |
| `safe_rl_node/object_avoided` | RL agent successfully avoided obstacle |
| `safe_rl_node/collision_detected` | Imminent collision detected |
| `fsm_node/mode` | Current FSM state |

---
## Notes
- RL weights must be pre-trained and present at `/data/safe_rl_weights/` before running the deployment node
- The `safe_rl_training_node` and the corresponding training FSM must be run before `safe_rl_node` to generate weights
- Customize the distance threshold based on your preference

## Acknowledgement

- Dt-core stack: https://github.com/duckietown/dt-core
- Twin Delayed Algorithm, taken from: [https://github.com/Rafael1s/Deep-Reinforcement-Learning-Algorithms/blob/master/BipedalWalker-TwinDelayed-DDPG%20(TD3)/BipedalWalker_1795ep_300-5sc_9h44m.ipynb](https://github.com/Rafael1s/Deep-Reinforcement-Learning-Algorithms/tree/master/BipedalWalker-TwinDelayed-DDPG%20(TD3))](https://github.com/Rafael1s/Deep-Reinforcement-Learning-Algorithms/tree/master/BipedalWalker-TwinDelayed-DDPG%20(TD3))
