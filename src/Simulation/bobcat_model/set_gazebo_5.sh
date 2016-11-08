cd src
var="#define MY_GAZEBO_VER 5"
sed -i "2s/.*/$var/" bobcat_drive_plugin.cc
sed -i "2s/.*/$var/" bobcat_tracked_drive_plugin.cc
sed -i "2s/.*/$var/" bobtank_drive_plugin.cc
