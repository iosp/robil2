# -*- coding: utf-8 -*-
#! /usr/bin/env python
from tree import xmlTree
from Node import node
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import *

class Ctree:
    myTree = None
    filePath =""
def __init__(self,inputTree):
    pass


def nodeDataInDebugMode(nodeTime, nodeSuccFail, nodeID, monitordNodeID, numOfIter, param):
    newTree =  xmlTree("output/"+Ctree.filePath[6:-4]+"_after_run_debug_false.xml")
    newTree.createWrapperTreeMap("id")
    monitorNode = None
   # print "E="+ str(Ctree.myTree.getWrappedNode(monitordNodeID).getAverageSuccTime(0))  
   # print "prob="+str(Ctree.myTree.getWrappedNode(monitordNodeID).getProbAtIndex(0))
    
    #get pointer to the monitored node
    finished_node = newTree.getWrappedNode(nodeID)
    finished_node.setDebug(str(nodeSuccFail) + " " + str(nodeTime))
    newTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_"+monitordNodeID+"_after_run_debug_true.xml")
    node.debugMode = True
    newTree = xmlTree("output/"+Ctree.filePath[6:-4]+"_"+monitordNodeID+"_after_run_debug_true.xml")
    root = newTree.getRoot()
    for i in range(numOfIter):  
        root.runPlan(param)
    #print tree to xml file
    newTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_"+monitordNodeID+"_after_run_debug_true.xml")
    newTree = xmlTree("output/"+Ctree.filePath[6:-4]+"_"+monitordNodeID+"_after_run_debug_true.xml")
    newTree.createWrapperTreeMap("id")
    #check if the node monitor is the root
    if monitordNodeID == "":
        root = newTree.getRoot()
        monitorNode = root.getChild(0)
    else:
        monitorNode = newTree.getWrappedNode(monitordNodeID)    
    
    #take E,prob and standarte deviation  from the debug node,
    E =  monitorNode.getAverageSuccTime(param)
    prob = monitorNode.getProbAtIndex(param)
    sd = monitorNode.getSDSuccTime(param)
    #clear tree from calculations
    root = newTree.getRoot()
    root.clearWholeTree()
    
    #print update tree to xml file
    newTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_"+monitordNodeID+"_after_run_debug_true.xml")
    
    return (prob, sd, E)


def constructTree(event_file, param):
    # if the tree is not null then don't calculate the tree.
    if Ctree.myTree != None:
        return
    Ctree.filePath = event_file
#    event_file_full_path = event_file
    #tsk_attributes_full_path = event_file_full_path[0:-4] + "_tsk_attribs.xml"
    Ctree.myTree = xmlTree(Ctree.filePath, None,Ctree.filePath[0:-4]+"_tsk_attribs.xml")
    print Ctree.filePath[0:-4]+"_tsk_attribs.xml"
    Ctree.myTree.createWrapperTreeMap("id")
    root = Ctree.myTree.getRoot()
    
    node.debugMode = False	
    for i in range(1000):
        root.runPlan(param)
    Ctree.myTree.treeToXml("output/"+Ctree.filePath[6:-4]+"_after_run_debug_false.xml")
    Ctree.myTree  = xmlTree("output/"+Ctree.filePath[6:-4]+"_after_run_debug_false.xml")  
    Ctree.myTree.createWrapperTreeMap("id")
    
def getNodeInfo(nodeID, param):
    Ctree.myTree.createWrapperTreeMap("id")
    if nodeID == "":
	root = Ctree.myTree.getRoot()
	infoNode =  root.getChild(0)
    else:
	infoNode = Ctree.myTree.getWrappedNode(nodeID)
	
    E =  infoNode.getAverageSuccTime(param)
    prob = infoNode.getProbAtIndex(param)
    sd = infoNode.getSDSuccTime(param)
    
    return (prob, sd, E)
    
def plotDist (nodeID, param, succ):
    Ctree.myTree.createWrapperTreeMap("id")
    if nodeID == "":
	root = Ctree.myTree.getRoot()
	infoNode =  root.getChild(0)
    else:
	infoNode = Ctree.myTree.getWrappedNode(nodeID) 
    (duration, times) =  infoNode.getDistData(param, succ)
    plt.bar(duration, times,align='center')
    plt.show() 
    
    
    
    
#if __name__ == "__main__":
#  constructTree("tests/skill4.xml", 0)
#  print nodeDataInDebugMode(50, True, "cca761ab-b52d-43e8-a753-b4c458aea4db","b9e18714-4869-422c-bc38-cb2dca88c530", 50,0)
#  
#  
  