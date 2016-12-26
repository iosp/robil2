#!/usr/bin/env python
 
import rospy
from std_msgs.msg import *
from robil_msgs.msg import *
from lxml import etree
import os
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

WSMstepsID = ["","WSMBladeAngle","WSMBladeHeight","WSMClamp","WSMAdvance","WSMTurn"]

def addProperties(stepType):
  plpFile=open(__location__+"/monitor/PLPs/"+WSMstepsID[stepType]+".plp", "r")
  loc = plpFile.read().find('Success probability:')
  plpFile.seek(loc+20)
  prob=plpFile.read(4)
  plpFile.close()
  return(prob,"","")  

def callback(data):
  rospy.loginfo("start listenning...")
  msgPLPData=[]
  for t in data.stepsType:
    print(t)
    xmlFile=open(__location__+"/monitor/PLPs/"+WSMstepsID[t]+".plp", "r")
    loc = xmlFile.read().find(data.field)
    xmlFile.seek(loc+len(data.field))
    prob=xmlFile.read(4)
    print prob
    msgPLPData.append(prob)
  pub = rospy.Publisher("/Monitor/PLPs/prob", PLPData,latch=True)
  r = rospy.Rate(10) # 10hz
  #while not rospy.is_shutdown():
    #rospy.loginfo(prob)
  pub.publish(msgPLPData)
    #r.sleep()  
  print("herePLPserver!")
  rospy.loginfo("done!")
  
      
def listener():
  rospy.init_node('PlanAttrib', anonymous=True)
  rospy.Subscriber("/Monitor/PLPs/step_type", StepType, callback)
  rospy.spin()
  
  

if __name__ == "__main__":
  listener()
  
  
  
##todo:
#1. call to this script from anywhere not just my folder
#2. read distributions  - not done yet
#3. rosnode that reads the plp data and provides a msg of prob and dist 
