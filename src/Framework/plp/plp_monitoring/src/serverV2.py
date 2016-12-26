#!/usr/bin/env python
 
import rospy
from std_msgs.msg import *
from robil_msgs.msg import *
from lxml import etree
import os
from monitor import Compute
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import *

__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

WSMstepsID = ["","WSMBladeAngle","WSMBladeHeight","WSMClamp","WSMAdvance","WSMTurn"]
PROB = "0.0"
FLAG = True
UB = 0.4 
LB = 0.5
global tree
global T
global samples
  
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
  pub.publish(msgStepsType)
      
  print("hereserver!")
    
  rospy.Subscriber("/Monitor/PLPs/prob", PLPData, callbackProb)
  rospy.sleep(1)
  print(PROB)
  for i in range(len(PROB)):
    v = stepsList[i].value
    if (i==0):
      rootAttrib = etree.Element("tsk",name=str(stepsList[i].id), id=str(stepsList[i].id), probability=PROB[i],Successdistribution="U["+str(abs(v/LB))+","+str(abs(v/UB))+"]",Failuredistribution="U["+str(abs(2*v/LB))+","+str(abs(2*v/UB))+"]")
    else:
      etree.SubElement(rootAttrib,"tsk", name=str(stepsList[i].id),id=str(stepsList[i].id), probability=PROB[i],Successdistribution="U["+str(abs(v/LB))+","+str(abs(v/UB))+"]",Failuredistribution="U["+str(abs(2*v/LB))+","+str(abs(2*v/UB))+"]")
    etree.SubElement(seq, "tsk", name=str(stepsList[i].id), id=str(stepsList[i].id))
  xmlFile=open(__location__+"/monitor/plan.xml", "w+r")
  xmlFile.write(etree.tostring(root, pretty_print=True))  
  xmlFileAttrib=open(__location__+"/monitor/plan_attrib.xml", "w+r")
  xmlFileAttrib.write(etree.tostring(rootAttrib, pretty_print=True))  
  xmlFile.close()
  xmlFileAttrib.close()
  global tree
  tree=config() 
  rospy.Subscriber("/monitor/task_time", Header, callbackUpdatePlan)
  

  #execfile(__location__+'/config.py') 
  
  
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
  dist=["0.3","0.6"]
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
  

def config():
    global T
    global samples
    fd=open(__location__+"/monitor/output/outputFile","w")
    filePlanName=__location__+"/monitor/plan.xml"
    filePlanAttribName=__location__+"/monitor/plan_attrib.xml"
    tree=Compute.durLessThanT(fd,filePlanName,filePlanAttribName,float(T),int(samples))
    fd.close()
    print "done"
    return tree  

def callbackUpdatePlan(data):
    global tree
    global T
    (probSucc,avg,std)=Compute.updateNode(str(data.seq),data.frame_id,float(T),tree)
    strOut = str(probSucc)+" "+str(avg)+" "+str(std)
    pub1 = rospy.Publisher("/Monitor/PLPs/output", String,latch=True)
    pub1.publish(strOut)
    
  
    
if __name__ == "__main__":
  global T
  T = sys.argv[1]
  global samples
  samples = sys.argv[2]
  listener()
  
  
  
##todo:
#1. call to this script from anywhere not just my folder

