#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching all nodes required by FSM
dt-exec bash -c "
  roslaunch anti_instagram anti_instagram_node.launch veh:=\$VEHICLE_NAME &
  roslaunch line_detector line_detector_node.launch veh:=\$VEHICLE_NAME &
  roslaunch lane_filter lane_filter_node.launch veh:=\$VEHICLE_NAME &
  roslaunch ground_projection ground_projection_node.launch veh:=\$VEHICLE_NAME &
  roslaunch lane_control lane_control_node.launch veh:=\$VEHICLE_NAME &
  roslaunch led_emitter led_emitter_node.launch veh:=\$VEHICLE_NAME &
  roslaunch led_joy_mapper led_joy_mapper_node.launch veh:=\$VEHICLE_NAME &
  
  # Launch FSM last (it controls the other nodes via services)
  roslaunch fsm fsm_node.launch veh:=\$VEHICLE_NAME
"


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
