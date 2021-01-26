#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init --quiet

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

echo 'Hallo'

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

roscore &
sleep 5

# launching app
#dt-exec roslaunch my_package nodePurePursuit.launch
#dt-exec roslaunch my_package nodeTest.launch
dt-exec roslaunch my_package nodePID.launch
#dt-exec roslaunch my_package nodeCascadePID.launch
#dt-exec roslaunch my_package nodeCascadePID_2T.launch
#dt-exec roslaunch my_package nodeStateFeedback.launch
#dt-exec roslaunch my_package nodeStanley.launch
#dt-exec roslaunch my_package nodeLQR.launch
#dt-exec roslaunch my_package nodeparallelPID.launch
#dt-exec roslaunch my_package nodeGotoAngle.launch
#dt-exec roslaunch my_package noderealPID.launch
#dt-exec roslaunch my_package nodePolePlacement.launch

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join --quiet
