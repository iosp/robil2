#!/bin/sh


#locate SRVSS package
SRVSS_dir=$(rospack find SRVSS)

cd $ROBIL2/src/3party/Image_Generation_VisOdo/Data_Base/tree1
rosrun gazebo_ros spawn_model -file $ROBIL2/src/3party/Image_Generation_VisOdo/models/FLEA3/model.sdf -sdf -model flea3
rosrun Image_Generation_VisOdo Image_Generation_VisOdo
#cd ~/Desktop/Sagi
#rosrun imagesub imagesub



