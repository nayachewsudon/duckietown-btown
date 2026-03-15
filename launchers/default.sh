#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking pro
# launching FSM to control lane following with state management
CAMERA_TOPIC="${CAMERA_TOPIC:-camera_node}"

dt-exec roslaunch --wait duckietown_btown fsm_lane_following.launch \
	veh:=${VEHICLE_NAME} \
	camera_topic:=${CAMERA_TOPIC}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
