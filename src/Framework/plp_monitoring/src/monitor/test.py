



import rospy
from std_msgs.msg import *
from robil_msgs.msg import *
from lxml import etree
import os
__location__ = os.path.realpath(os.path.join(os.getcwd(), os.path.dirname(__file__)))

WSMstepsID = ["","WSMBladeAngle","WSMBladeHeight","WSMClamp","WSMAdvance","WSMTurn"]

def findInPLP(step,fields):
  plpFd=open(__location__+"/PLPs/"+WSMstepsID[step]+".plp", "r")
  for field in fields:    
    loc = plpFd.read().find(field)    
    plpFd.seek(loc+len(field))
    print field
    if plpFd.read(1)=='':
      loclb = xmlFile.read().find("Lower bound:",loc)
      locub = xmlFile.read().find("Upper bound:",loc)
    else:  
      prob=plpFd.read(4)
  print prob
  
  
if __name__ == "__main__":
  findInPLP(1,["velocity","Success probability:"])
  
  