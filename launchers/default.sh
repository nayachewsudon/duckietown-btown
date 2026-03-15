#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking pro
# launching FSM to control lane following with state management
CAMERA_TOPIC_DEFAULT="camera_node"
if [[ "${ROBOT_CONFIGURATION}" == DB21* ]]; then
	CAMERA_TOPIC_DEFAULT="camera_driver_front_center"
fi
CAMERA_TOPIC="${CAMERA_TOPIC:-${CAMERA_TOPIC_DEFAULT}}"

dt-exec roslaunch --wait duckietown_btown fsm_lane_following.launch \
	veh:=${VEHICLE_NAME} \
	camera_topic:=${CAMERA_TOPIC}


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
