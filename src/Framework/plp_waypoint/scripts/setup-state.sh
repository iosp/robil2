#!/bin/bash

# sets-up the state of the bobcat,
# as in publishes the tasks and missions.

echo Tasks...
rostopic pub /OCU/SMME/NavigationTask robil_msgs/AssignNavTask -1 -f  ../missions-and-tasks/440.yaml
rostopic pub /OCU/SMME/NavigationTask robil_msgs/AssignNavTask -1 -f  ../missions-and-tasks/441.yaml
rostopic pub /OCU/SMME/NavigationTask robil_msgs/AssignNavTask -1 -f  ../missions-and-tasks/442.yaml
rostopic pub /OCU/SMME/NavigationTask robil_msgs/AssignNavTask -1 -f  ../missions-and-tasks/999.yaml

echo Missions...
rostopic pub /OCU/SMME/MissionPlan robil_msgs/AssignMission -1 -f  ../missions-and-tasks/Mission440.yaml
rostopic pub /OCU/SMME/MissionPlan robil_msgs/AssignMission -1 -f  ../missions-and-tasks/Mission441+442.yaml
rostopic pub /OCU/SMME/MissionPlan robil_msgs/AssignMission -1 -f  ../missions-and-tasks/Mission441.yaml
rostopic pub /OCU/SMME/MissionPlan robil_msgs/AssignMission -1 -f  ../missions-and-tasks/Mission999.yaml
