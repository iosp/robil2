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
        time.sleep(10)


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
	smartest_pkg_path = rospack.get_path('smartest')
	os.environ["GAZEBO_MODEL_PATH"] = smartest_pkg_path+"/world_components_models/:" + Scenarin_folder + "/scenarioSystemModels/"

	srvss_bobcat_pkg_path = rospack.get_path('srvss_bobcat')
	urdf_file = srvss_bobcat_pkg_path +"/urdf/BOBCAT.URDF"
        robot_urdf_file = open(urdf_file)
	robot_urdf = robot_urdf_file.read()
	rospy.set_param("/robot_description", robot_urdf )
	return True


    def launch_platform_controls_spawner(self):
	rospack = rospkg.RosPack()
	srvss_bobcat_pkg_path = rospack.get_path('srvss_bobcat')
	srvss_bobcat_controllers_yaml = srvss_bobcat_pkg_path +"/controllers/srvss_bobcat_controllers.yaml"
	paramlist=rosparam.load_file(srvss_bobcat_controllers_yaml, default_namespace='' ,verbose=True)
	for params, ns in paramlist:
    		rosparam.upload_params(ns,params)

	# if the controllers do not load it possible to increase the time of waiting for the server in the spawner
	# it located in /home/userws3/gz_ros_cws/src/ros_control/controller_manager/scripts
	node = ROSNode("controller_manager", "spawner", name="platform_controllers_spawner" ,namespace="/srvss_bobcat", output="screen", respawn="false" ,
				args="joint_state_controller \
					front_right_wheel_velocity_controller \
					front_left_wheel_velocity_controller \
					back_right_wheel_velocity_controller \
					back_left_wheel_velocity_controller")
	self.launcher.launch(node)	
        time.sleep(10)

	node = ROSNode("srvss_bobcat", "srvss_bobcat_rate2effort_node", name="srvss_bobcat_RateToEffort_node", cwd="node", output="screen") 
	self.launcher.launch(node)	
        time.sleep(3)


    def launch_platform_controls_unspawner(self):
	node = ROSNode("controller_manager", "unspawner" ,name="platform_controllers_unspawner" ,namespace="/srvss_bobcat", output="screen", respawn="false" ,
				args="joint_state_controller \
					front_right_wheel_velocity_controller \
					front_left_wheel_velocity_controller \
					back_right_wheel_velocity_controller \
					back_left_wheel_velocity_controller")
	self.launcher.launch(node)	
        time.sleep(10)



    def launch_WPdriver(self,Scenarin_folder):
	arguments = "-file " + Scenarin_folder + "/scenarioMission.txt"
	node = ROSNode("srvss_wp_driver", "srvss_wp_driver_node",name="srvss_wp_driver_node", args=arguments , respawn="false") # output="screen"
	self.launcher.launch(node)	
        time.sleep(3)


    def launch_robot_tf_broadcaster(self):
	remaping = [ ("/joint_states" , "/srvss_bobcat/joint_states") ] 
	node = ROSNode("robot_state_publisher", "robot_state_publisher",name="robot_state_broadcaster_node", remap_args=remaping ,output="screen", respawn="false")
	self.launcher.launch(node)
        time.sleep(3)	

	node = ROSNode("srvss_bobcat", "world_to_bobcat_tf_broadcaster_node",name="worltd_to_bobcat_tf_broadcaster_node" ,output="screen", respawn="false")
	self.launcher.launch(node)
        time.sleep(3)

	node = ROSNode("tf", "static_transform_publisher",name="sick_link_tf_broadcaster_node", args="1 0 0.2 0 0 0 body front_sick 100" ,output="screen", respawn="false")
	self.launcher.launch(node)	
        time.sleep(3)


    def launch_recorder(self, Scenarin_folder):
	arguments = "-a -O " + Scenarin_folder + "/scen_rec.bag"
	node = ROSNode("rosbag", "record", name="rosbag_recorde_node", args=arguments)
	self.launcher.launch(node)
        time.sleep(3)	


    def launch_grader(self, Scenarin_folder):
	arguments = Scenarin_folder + " " + Scenarin_folder+"/scen.SFV"
	print "I am hear arguments = " + arguments
	node = ROSNode("smartest", "grader_node", name="grader_node", output="screen", args=arguments)
	self.launcher.launch(node)
        time.sleep(3)	



    def Gazebo_UnPause(self):
	name = '/gazebo/unpause_physics'
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




















