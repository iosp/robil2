#!/bin/sh


#locate smartest package
smartest_dir=$(rospack find smartest)
TARGET_DIR=$(rospack find live_bobcat)/sdf_models/

#replacing environment models in .gazebo in order to avoid collisions in the greading process
cp -R $smartest_dir/world_components_models/. $HOME/.gazebo/models/

find $TARGET_DIR -name "*.stl" -exec bash -c 'meshlabserver -i $0 -o ${0/stl/obj}' {} \;
find $TARGET_DIR -name "*.STL" -exec bash -c 'meshlabserver -i $0 -o ${0/STL/obj}' {} \;
echo "TARGET_DIR = $TARGET_DIR"

exit 0
