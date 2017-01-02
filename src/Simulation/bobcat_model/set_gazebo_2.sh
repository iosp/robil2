cd src
var="#define MY_GAZEBO_VER 2"
sed -i "2s/.*/$var/" bobcat_drive_plugin.cc
sed -i "2s/.*/$var/" bobcat_tracked_drive_plugin.cc
sed -i "2s/.*/$var/" bobtank_drive_plugin.cc
