#!/usr/bin/env python
 
import rospy
from std_msgs.msg import *
from robil_msgs.msg import *
from lxml import etree
import os


__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

WSMstepsID = ["","WSMBladeAngle","WSMBladeHeight","WSMClamp","WSMAdvance","WSMTurn"]
PROB = "0.0"
FLAG = True
UB = 0.4 
LB = 0.5
  
def creatPlan(name, stepsList):
  root = etree.Element("plan")
  seq = etree.SubElement(root, "seq", name=name)
  msgStepsType=StepType()
  stepsType=[]
  field = 'Success probability:'
  for step in stepsList:
    stepsType.append(step.type)
 
  msgStepsType.stepsType= stepsType
  msgStepsType.field= field
  pub = rospy.Publisher("/Monitor/PLPs/step_type", StepType,latch=True)
  r = rospy.Rate(10) # 10hz
  #while not rospy.is_shutdown():
    #rospy.loginfo(step.type)
  pub.publish(msgStepsType)
    #r.sleep()  
      
  print("hereserver!")
    
  rospy.Subscriber("/Monitor/PLPs/prob", PLPData, callbackProb)
  
  #-----find dist-----
  #msgStepsType.stepsType= stepsType
  #msgStepsType.field= "velocity"  
  #pub.publish(msgStepsType)
  #rospy.Subscriber("/Monitor/PLPs/prob", PLPData, callbackDist)
  
  rospy.sleep(1)
  print(PROB)
  for i in range(len(PROB)):
    v = stepsList[i].value
    if (i==0):
      rootAttrib = etree.Element("tsk",name=str(stepsList[i].id), probability=PROB[i],Successdistribution="U["+str(v/LB)+","+str(v/UB)+"]",Failuredistribution="U["+str(2*v/LB)+","+str(2*v/UB)+"]")
    else:
      etree.SubElement(rootAttrib,"tsk", name=str(stepsList[i].id), probability=PROB[i],Successdistribution="U["+str(v/LB)+","+str(v/UB)+"]",Failuredistribution="U["+str(2*v/LB)+","+str(2*v/UB)+"]")
    etree.SubElement(seq, "tsk", name=str(stepsList[i].id))
  xmlFile=open(__location__+"/monitor/plan.xml", "w+r")
  xmlFile.write(etree.tostring(root, pretty_print=True))  
  xmlFileAttrib=open(__location__+"/monitor/plan_attrib.xml", "w+r")
  xmlFileAttrib.write(etree.tostring(rootAttrib, pretty_print=True))  
  xmlFile.close()
  xmlFileAttrib.close()
  execfile('config.py')  
  
def callbackProb(data):
  global PROB
  global FLAG
  PROB = data.PLPData
  print PROB
  FLAG = False
  
def callbackDist(data):
  global UB,LB
  global FLAG
  #dist = data.PLPData.split(",")
  dist=["0.2","0.5"]
  UB=float(dist[0])
  LB=float(dist[1])
  FLAG = False
  
def callback(data):
  rospy.loginfo("start creating a plan ...")
  creatPlan(data.task_description, data.steps)
  rospy.loginfo("done!")
  
      
def listener():
  rospy.init_node('constructPlan', anonymous=True)
  rospy.Subscriber("/SMME/WSM/Task", AssignManipulatorTask, callback)
  rospy.spin()
  

if __name__ == "__main__":
  listener()
  
  
  
##todo:
#1. call to this script from anywhere not just my folder

