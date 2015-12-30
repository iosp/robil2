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
bobcat_gz=$(rospack find bobcat_gazebo)
bobtank_dir=$(rospack find bobtank)
#locate live_bobcat package
live_bobcat_dir=$(rospack find live_bobcat)

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
if [ ! -d $HOME/.gazebo/models/bobtank ]; then
	$(mkdir -p $HOME/.gazebo/models/bobtank/meshes_bobcat_with_boggies)
fi

#copy meshes
$(cp -u $bobtank_dir/meshes_bobcat_with_boggies/* $HOME/.gazebo/models/bobtank/meshes_bobcat_with_boggies)

#overite model.sdf
if [ -a $HOME/.gazebo/models/bobtank/model.sdf ]; then
	$(rm -f $HOME/.gazebo/models/bobtank/model.sdf)
	$(rm -f $HOME/.gazebo/models/bobtank/modelTemp.sdf)
fi

#generate basic sdf file
$(cp -u $bobtank_dir/urdf/bobcat_with_tracks_new.sdf $HOME/.gazebo/models/bobtank/modelTemp.sdf)
#$($gazebo_prefix/bin/gzsdf "print" $bobcat_dir/urdf/BOBCAT.URDF 1> $HOME/.gazebo/models/bobcat/modelTemp.sdf 2>/dev/null)


#add the sensors and joints to the model
while read line
do
    if [[ $line == *"</model>"* ]]; then
	$(cat $live_bobcat_dir/scripts/sensorPosition.txt >> $HOME/.gazebo/models/bobtank/model.sdf)
    fi
	echo $line >> $HOME/.gazebo/models/bobtank/model.sdf
done < $HOME/.gazebo/models/bobtank/modelTemp.sdf

$(rm -f $HOME/.gazebo/models/bobtank/modelTemp.sdf)

#overite model.config
if [ -a $HOME/.gazebo/models/bobtank/model.config ]; then
	$(rm -f $HOME/.gazebo/models/bobtank/model.config)
fi

#copy tracks

if [ ! -d $HOME/.gazebo/models/tracks ]; then
	$(mkdir -p $HOME/.gazebo/models/tracks)
fi


#copy meshes
$(cp -u -r $bobtank_dir/meshes_trucks $HOME/.gazebo/models/tracks)
$(cp -u -r $bobtank_dir/meshes_trucks_colission $HOME/.gazebo/models/tracks)
$(cp -u -r $bobtank_dir/meshes_bobcat_with_boggies $HOME/.gazebo/models/bobcat_gazebo)

echo '<?xml version="1.0"?>' >> $HOME/.gazebo/models/bobtank/model.config
echo '<model>' >> $HOME/.gazebo/models/bobtank/model.config
echo '  <name>bobcat</name>' >> $HOME/.gazebo/models/bobtank/model.config
echo '  <version>1.0</version>' >> $HOME/.gazebo/models/bobtank/model.config
echo '  <sdf version="1.4">model.sdf</sdf>' >> $HOME/.gazebo/models/bobtank/model.config
echo '</model>' >> $HOME/.gazebo/models/bobtank/model.config

printf "done\n"

#########################################################
#Generate testing ground model in the gazebo environment#
#########################################################

printf "adding worlds and terrain models to database --- "
#create the worlds dir if it doesn't exist
if [ ! -d $HOME/.gazebo/worlds ]; then
	$(mkdir -p $HOME/.gazebo/worlds)
fi

$(cp -u -r $live_bobcat_dir/utils/models/* $HOME/.gazebo/models)



printf "done\n"
