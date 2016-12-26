# -*- coding: utf-8 -*-

from tree import xmlTree
from Node import node
from computeTree import *
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import *

    
def test29():
    start = time.time()
    tree = xmlTree("tests/event3_no_tsk_attrib.xml", None,"tests/event3_tsk_attrib.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/testE291.xml")  
    print("test 29.1: success!")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0)
    for i in range(100):
        root.runPlan(1)
    root.treeToXml("output/testE292.xml") 
    print("test 29.2: success!")
    print "Success probability in offline mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    print "-------Debug mode-------"
    node.debugMode = True
    for i in range(100):
        root.runPlan(0)
    for i in range(100):
        root.runPlan(1)
    root.treeToXml("output/testE293.xml") 
    print("test 29.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed

def test30():
    start = time.time()
    tree = xmlTree("tests/small_test_integration.xml", None,"tests/small_test_integration_tsk_attrib.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

#    root.treeToXml("output/event3_m.xml")  
    print("test 30.1: success!")
 
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)
#    for i in range(100):
#        root.runPlan(1)
    root.treeToXml("output/small_test_integration.xml") 
    print("test 30.2: success!")
    print "Success probability in offline mode: %f" % root.getChild(0).getProbAtIndex(0)
    print "Average success time = %f" % root.getChild(0).getAverageSuccTime(0)

    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    
    
    
def test31():
    start = time.time()
    tree = xmlTree("tests/event3_m.xml",None,"tests/event3_m_tsk_attribs.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/small_test_event3_m.xml") 
    print("test 31.1: success!")
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)
    root.treeToXml("output/small_test_event3_m_after_run.xml") 
    print("test 31.2: success!")
    print "Success probability in offline mode: %f" % root.getChild(0).getProbAtIndex(0)
    print "Average success time = %f" % root.getChild(0).getAverageSuccTime(0)

    elapsed = (time.time() - start)
    print "Time: %f" %elapsed


def test32():
    start = time.time()
    node.parmetersInTheWorld = 1
    PARAM=0
    constructTree("tests/skill4.xml", PARAM)	
    print("test 32.1: success - construct tree!") 
    monitordNode = "ae9c53ae-b42c-4f21-ba6a-7b1fa8741c2d"
   
    (prob, sd, E) = getNodeInfo(monitordNode, PARAM)
  
    print "Success probability in offline mode monitor: Mission: %f" % prob
    print "Average success time monitor: Mission= %f" % E
    print "SD success time monitor: Mission= %f" % sd
    plotDist(monitordNode,PARAM,0)
    monitordNode = "b9e18714-4869-422c-bc38-cb2dca88c530"
    
    (prob, sd, E) = getNodeInfo(monitordNode, PARAM)
    print "Success probability in offline mode monitor: ExitFromCar: %f" % prob
    print "Average success time monitor: ExitFromCar= %f" % E
    print "SD success time monitor: ExitFromCar= %f" % sd
    plotDist(monitordNode,PARAM,0)
    monitordNode = ""
    (prob, sd, E) = getNodeInfo(monitordNode, PARAM)
    print "Success probability in offline mode monitor: root: %f" % prob
    print "Average success time monitor: root= %f" % E
    print "SD success time monitor: root= %f" % sd
    plotDist(monitordNode,PARAM,0)

    elapsed = (time.time() - start)
    print "Time: %f" %elapsed    
#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.#thanks
def _createComputedDist(string = None):
    from distributions.computed import Computed
    return Computed()
    
#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.
def _createNormalDist(parmM,parmG):
   from distributions.normal import Normal
   return Normal(float(parmM),float(parmG))

#changed by RAZ -- we can now import from dist.* files, since the directory has an empty __init__.py file, and python recognizes it as a module.        
def _createUniformDist(parmA,parmB):
   from distributions.uniform import Uniform
   return Uniform(float(parmA),float(parmB))

if __name__ == "__main__":
    
    test32()
