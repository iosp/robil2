#!/bin/sh

##########################
#Initializing environment#
##########################

#making sure the ROS environment is set up
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS environment isn't defined. forced exit"
    exit 1
fi 

#get gazebo prefix
gazebo_prefix=$(pkg-config --variable=prefix gazebo)

#setup file path
setup_file=$gazebo_prefix/share/gazebo/setup.sh
sudo chmod 777 $setup_file

#locate bobcat package
bobcat_dir=$(rospack find bobcat)

#locate srvss_pkg package
srvss_pkg_dir=$(rospack find srvss_pkg)

############################################
#Adding .gazebo to the simulation resources#
############################################

printf "Adding .gazebo to the simulation resources --- "
if grep -q 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo' $setup_file; then
	printf "already configured\n"
else
	echo 'export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:~/.gazebo' >> $setup_file
	printf "done\n"
fi


##########################################################
#Generate a model of the bobcat in the gazebo environment#
##########################################################

printf "setting the sahar model as an sdf model in the gazebo --- "
#create the model dir if it doesn't exist
if [ ! -d $HOME/.gazebo/models/bobcat ]; then
	$(mkdir -p $HOME/.gazebo/models/bobcat/meshes)
fi

#copy meshes
$(cp -u $bobcat_dir/meshes/* $HOME/.gazebo/models/bobcat/meshes)

#overite model.sdf
if [ -a $HOME/.gazebo/models/bobcat/model.sdf ]; then
	$(rm -f $HOME/.gazebo/models/bobcat/model.sdf)
	$(rm -f $HOME/.gazebo/models/bobcat/modelTemp.sdf)
fi

#generate basic sdf file
$($gazebo_prefix/bin/gzsdf "print" $bobcat_dir/urdf/BOBCAT.URDF 1> $HOME/.gazebo/models/bobcat/modelTemp.sdf 2>/dev/null)

#add the sensors to the model
while read line
do
    if [[ $line == *"</model>"* ]]; then
	$(cat $srvss_pkg_dir/utils/models/sensorPosition.txt >> $HOME/.gazebo/models/bobcat/model.sdf)
    fi
	echo $line >> $HOME/.gazebo/models/bobcat/model.sdf
done < $HOME/.gazebo/models/bobcat/modelTemp.sdf

$(rm -f $HOME/.gazebo/models/bobcat/modelTemp.sdf)

#overite model.config
if [ -a $HOME/.gazebo/models/bobcat/model.config ]; then
	$(rm -f $HOME/.gazebo/models/bobcat/model.config)
fi

echo '<?xml version="1.0"?>' >> $HOME/.gazebo/models/bobcat/model.config
echo '<model>' >> $HOME/.gazebo/models/bobcat/model.config
echo '  <name>srvss_bobcat</name>' >> $HOME/.gazebo/models/bobcat/model.config
echo '  <version>1.0</version>' >> $HOME/.gazebo/models/bobcat/model.config
echo '  <sdf version="1.4">model.sdf</sdf>' >> $HOME/.gazebo/models/bobcat/model.config
echo '</model>' >> $HOME/.gazebo/models/bobcat/model.config

printf "done\n"

#########################################################
#Generate testing ground model in the gazebo environment#
#########################################################

printf "adding worlds and terrain models to database --- "
#create the worlds dir if it doesn't exist
if [ ! -d $HOME/.gazebo/worlds ]; then
	$(mkdir -p $HOME/.gazebo/worlds)
fi

$(cp -u $srvss_pkg_dir/utils/worlds/* $HOME/.gazebo/worlds)

#create the textures dir if it doesn't exist
if [ ! -d $HOME/.gazebo/media/materials/textures ]; then
	$(mkdir -p $HOME/.gazebo/media/materials/textures )
fi

$(cp -u $srvss_pkg_dir/utils/textures/* $HOME/.gazebo/media/materials/textures)


printf "done\n"
