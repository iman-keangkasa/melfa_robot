#!/bin/bash
if [ $# -eq 0 ]
then 
  delay=5
else
  delay=$1
fi
figlet "${delay} seconds delay"
sleep ${delay}
roslaunch rv7fr_table_moveit_config _rv7fr_planning_execution.launch
exit 0
