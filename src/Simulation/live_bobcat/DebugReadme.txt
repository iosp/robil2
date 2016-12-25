Debugging gazebo:
roslaunch live_bobcat --screen -v test.launch paused:=true debug:=true

Debugging a rosnode see testframework.launch: add tag "launch-prefix="xterm -e gdb --args"

For example" 
<node name="ipon2ros" type="shiffon2ros_node" pkg="shiffon2ros" args="127.0.0.1" output="screen" respawn="true" launch-prefix="xterm -e gdb --args"/>
