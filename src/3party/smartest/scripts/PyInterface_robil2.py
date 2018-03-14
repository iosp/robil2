#!/usr/bin/env python

import roslib
import roslaunch
from roslaunch.scriptapi import ROSLaunch
from roslaunch.core import Node as ROSNode
import rospy
#from std_srvs.srv import Empty
import std_srvs.srv

import time
import os

import rospkg

roslib.load_manifest("rosparam")
import rosparam

from controller_manager_msgs.srv import *

class ScenarioLauncher:

    def __init__(self):
	print "I am ScenarioLauncher in python !!"

    def start_launcher(self):
	self.launcher = ROSLaunch()
	self.launcher.start()

    def stop_launcher(self):
	self.launcher.stop()
        time.sleep(60)


    def runGazeboServer(self, Scenarin_folder):
 	arguments = "-u " + Scenarin_folder + "/scenarioEnv.world"     # -u starts the simulation in pause mode
	node = ROSNode("gazebo_ros","gzserver",name="gazebo" ,args=arguments ,output="screen" , respawn="false")      	   # name="gazebo" in order for replay and grader to work (name space)
	self.launcher.launch(node)
        time.sleep(3)
	return True
	
    def runGazeboClient(self):
	node = ROSNode("gazebo_ros", "gzclient",name="gazebo_client" ,output="screen", respawn="false")
	self.launcher.launch(node)	
        time.sleep(3)
	return True	

    def setScenarioEnv(self,Scenarin_folder):
	rospy.set_param('/use_sim_time',True)	#synchronizing ros to gazebo

	rospack = rospkg.RosPack()
	srvss_pkg_path = rospack.get_path('smartest')
	os.environ["GAZEBO_MODEL_PATH"] = srvss_pkg_path+"/world_components_models/:" + Scenarin_folder + "/scenarioSystemModels/"
	return True


    def launch_platform_controls_spawner(self):  #used by the srvss_dummy
	return True

    
    def launch_platform_controls_unspawner(self): #used by the srvss_dummy
	return True
    
    
    
    
 
    def launch_WPdriver(self,Scenarin_folder):
	#rospack = rospkg.RosPack()
	#robil2conf_pkg_path = rospack.get_path('robil2conf')
	#robil2conf_yaml = robil2conf_pkg_path +"/configuration.yaml"
	#paramlist=rosparam.load_file(robil2conf_yaml, default_namespace='' ,verbose=True)
	#for params, ns in paramlist:
	#	rosparam.upload_params(ns,params)	
	
	#<!-- == GENERAL ===== -->
	print "Loading GENERAL !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	arguments = "ODOM	GPS			0.000  0.000  0.000	0.000  0.000  0.000 \
		     ODOM	INS			0.266  0.155 -1.683	0.000  0.000  0.000 \
		     ODOM	IBEO			0.310  0.170 -0.230	0.000  0.215  0.149 \
		     ODOM	TRACKS_BOTTOM		0.000  0.000 -2.260	0.000  0.000  0.000 \
		     ODOM	ROBOT_CENTER	       -0.380  0.155 -1.683	0.000  0.000  0.000"
		     
		     
	node = ROSNode("robil2tf", "robil2TF_node",name="robil2TF_publisher", args=arguments ,output="screen",  respawn="true")
	self.launcher.launch(node)

	node = ROSNode("robil2tf", "world_gazebo_to_WORLD_TF_pub.py",name="world_gazebo_to_WORLD_TF_pub",output="screen", respawn="true")
	self.launcher.launch(node)
	# ================ -->

	
	#<!-- == SENSORS INTERFACEs == -->
	node = ROSNode("shiffon2ros", "shiffon2ros_node",name="ipon2ros",args="127.0.0.1", output="screen")
	self.launcher.launch(node)
	node = ROSNode("sick_ldmrs", "sickldmrs.py",name="ibeo2ros",args="127.0.0.1", output="screen")
	self.launcher.launch(node)
	time.sleep(3)
	# ================ -->


        #<!-- == PLATFORM INTERFACE == -->    
        #node = ROSNode("lli", "lli_node",name="ros2qinetiq",args="127.0.0.1",output="screen", respawn="true")
	#self.launcher.launch(node)
	# ================ -->


	# == MONITORING == -->
	print "Loading MONITORING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	node = ROSNode("ssm", "ssm_fsm_states_tracker_node",name="ssm_fsm_states_tracker_node",output="screen")
	self.launcher.launch(node)
	node = ROSNode("ssm", "ssm_heartbeat_monitor_node",name="ssm_heartbeat_monitor_node",output="screen")
	self.launcher.launch(node)
	node = ROSNode("ssm", "ssm_node",name="ssm_node",output="screen")
	self.launcher.launch(node)
	# ================ -->

	
	#<!-- == LOCALIZATION == -->
	print "Loading LOCALIZATION node  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	node = ROSNode("loc", "loc_node",name="loc_node",output="screen",respawn="true")
	self.launcher.launch(node)
	time.sleep(1)
	# ================ -->

	# == PERCEPTION == -->
	print "Loading PERCEPTION !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	node = ROSNode("per", "per_node",name="per_node",output="screen",respawn="true")
	self.launcher.launch(node)
	

	#<!-- == OPERATOR CONTROL UNIT == -->
	node = ROSNode("ocu", "ocu_node",name="ocu_node",output="screen")
	self.launcher.launch(node)
	# ================ -->


	# -- MISSION CONTROL  -->
	print "Loading MISSION CONTRO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	node = ROSNode("smme", "smme_node",name="smme_node",output="screen",respawn="true")
	self.launcher.launch(node)
	node = ROSNode("wsm", "wsm_node",name="wsm_node",output="screen",respawn="true")
	self.launcher.launch(node)
	# ================ -->
	



	#<!-- == LOW LEVEL CONTROL == -->
	print "Loading LOW LEVEL CONTROL node  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	node = ROSNode("llc", "llc_node",name="llc_node",output="screen",respawn="true")
	self.launcher.launch(node)
        time.sleep(3)


	#<!-- == PATH PLANING  +   WAY POINT DRIVER == -->
	print "Loading PATH PLANING  +   WAY POINT DRIVER !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
	rospack = rospkg.RosPack()
	pp_pkg_path = rospack.get_path('pp')
	costmap_common_params = pp_pkg_path +"/params/costmap_common_params.yaml"
	paramlist=rosparam.load_file(costmap_common_params, default_namespace='move_base/global_costmap' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)

	paramlist=rosparam.load_file(costmap_common_params, default_namespace='move_base/local_costmap' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)

	local_costmap_params = pp_pkg_path +"/params/local_costmap_params.yaml"
	paramlist=rosparam.load_file(local_costmap_params, default_namespace='move_base' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)


	global_costmap_params = pp_pkg_path +"/params/global_costmap_params.yaml"
	paramlist=rosparam.load_file(global_costmap_params, default_namespace='move_base' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)


	base_local_planner_params = pp_pkg_path +"/params/base_local_planner_params.yaml"
	paramlist=rosparam.load_file(base_local_planner_params, default_namespace='move_base' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)


	node = ROSNode("move_base", "move_base",name="move_base",output="screen")
	self.launcher.launch(node)

	node = ROSNode("pp", "pp_node",name="pp_node",output="screen")
	self.launcher.launch(node)

	node = ROSNode("wpd", "wpd_node",name="wpd_node",output="screen")
	self.launcher.launch(node)
        # ================ -->



	#<!-- == Navigation Mission Load  == -->
	arguments = "-d 20 "+ Scenarin_folder + "/scenarioMission.bag"
	node = ROSNode("rosbag", "play",name="rosbag_Mission_node",output="screen",respawn="true", args=arguments)
	self.launcher.launch(node)
        # ================ -->





    def launch_robot_tf_broadcaster(self):
      	node = ROSNode("bobcat_model", "world_to_body_TF_pub.py",name="world_to_body_TF_pub", output="screen", respawn="true")
	self.launcher.launch(node)
        time.sleep(3)
        
        rospack = rospkg.RosPack()
      	bobcat_pkg_path = rospack.get_path('bobcat_model')
	urdf_file = bobcat_pkg_path +"/urdf_models/TF_pub_for_dynamic_arm.URDF"
        
        
        
        robot_urdf_file = open(urdf_file)
	robot_urdf = robot_urdf_file.read()
	rospy.set_param("/robot_description", robot_urdf )
      
        
	remaping = [ ("/joint_states" , "/Sahar/joint_states") ] 
	node = ROSNode("robot_state_publisher", "robot_state_publisher",name="robot_state_broadcaster_node", remap_args=remaping ,output="screen", respawn="false")
	self.launcher.launch(node)
        time.sleep(3)	



    def launch_recorder(self, Scenarin_folder):
	arguments = "-O " + Scenarin_folder + "/scen_rec.bag --all --exclude /SENSORS/CAM/(.*)|/SENSORS/IBEO/(.*)|/Sahar/(.*)|/PER/Map|/WSM/(.*)|/flea3/(.*)|/right_sick/(.*)|/left_sick/(.*)|/gazebo/model_states|/clock"
#       arguments = "-O " + Scenarin_folder + "/scen_rec.bag --all --exclude /SENSORS/(.*)|/LOC/Velocity|/Sahar/(.*)|/OCU/(.*)|/LLC/(.*)|/PER/(.*)|/WPD/(.*)|/WSM/(.*)|/flea3/(.*)|/right_sick/(.*)|/left_sick/(.*)|/heartbeat|/gazebo/model_states|/clock"
#	arguments = "-O " + Scenarin_folder + "/scen_rec.bag --all --exclude /SENSORS/(.*)|/LOC/Pose|/LOC/Velocity|/Sahar/(.*)|/OCU/(.*)|/LLC/(.*)|/PER/(.*)|/WPD/(.*)|/WSM/(.*)|/flea3/(.*)|/right_sick/(.*)|/left_sick/(.*)|/heartbeat|/gazebo/model_states|/clock"
	node = ROSNode("rosbag", "record", name="rosbag_recorde_node", args=arguments)
	self.launcher.launch(node)
        time.sleep(3)	




    def launch_grader(self, Scenarin_folder):
	arguments = Scenarin_folder + " " + Scenarin_folder+"/scen.SFV"
	node = ROSNode("smartest", "grader_node", name="grader_node", output="screen", args=arguments)
	self.launcher.launch(node)
        time.sleep(3)	




    def Gazebo_UnPause(self):
        print " i am inside Gazebo_UnPause !!!!!!!!! "
	name = '/gazebo/unpause_physics'
	msg_class = std_srvs.srv.Empty()
	print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! wait for service " + name
     	rospy.wait_for_service(name)
        print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! done wating"
        try:
            service = rospy.ServiceProxy(name, msg_class)
            resp1 = service()
            return True
        except rospy.ServiceException, e:
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Service call failed: %s"%e
        return True



    def Gazebo_Pause(self):
	name = '/gazebo/pause_physics'
	msg_class = std_srvs.srv.Empty()
	print "wait for service " + name
     	rospy.wait_for_service(name)
        print "done wating"
        try:
            service = rospy.ServiceProxy(name, msg_class)
            resp1 = service()
            return True
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return True




















