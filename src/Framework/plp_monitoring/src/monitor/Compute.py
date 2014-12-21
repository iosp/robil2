# -*- coding: utf-8 -*-
"""
Created on Wed Aug 27 20:11:12 2014

@author: liat
"""
from tree import xmlTree
from Node import node
from computeTree import *
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import *

__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

def coputeSuccProb(fd,stat, filePlanName,filePlanAttribName, runtime=0, samples=0):
   #if flagCompute1.get()==0:
   
   node.parmetersInTheWorld = 1
   #root.treeToXml("output/testforGUI.xml")  
   node.debugMode = False 
   
   if stat[0]:
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       root.treeToXml(__location__+"/output/testforGUI.xml")  
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)       
       fd.write("Accurate: Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml(__location__+"/output/testGUI1.xml") 
     
   if stat[1]: 
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           for i in range(int(samples)):           
               root.runPlan(j)           
           fd.write("Sampeling: # samples: "+str(samples)+" Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml(__location__+"/output/testGUI.xml") 
     
       
def coputeDur(fd,stat, filePlanName,filePlanAttribName, T, runtime=0, accuracy=0 ,samples=0):   
   node.parmetersInTheWorld = 1
   node.debugMode = False            
   
   if stat[0]:
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       tree.createWrapperTreeMap("id")
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)
           tmp = root.getChild(0).getLessThenTProb(j,float(T))       
       fd.write("Accurate: "+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml(__location__+"/output/testGUI1.xml")  
       return tree
       
   if stat[1]:  
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       tree.createWrapperTreeMap("id")
       for j in range(node.parmetersInTheWorld):
           root.runPlanApproximate(j,float(accuracy),float(T))
           tmp = root.getChild(0).getLessThenTProb(j,float(T))
       
       fd.write("Approxite: "+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml(__location__+"/output/testGUI2.xml") 
       return tree
       
   if stat[2]: 
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       tree.createWrapperTreeMap("id")
       for j in range(node.parmetersInTheWorld):
           for i in range(int(samples)):           
               root.runPlan(j)
           tmp = root.getChild(0).getLessThenTProb(j,float(T))
       fd.write("Sampeling: # samples: "+str(samples)+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml(__location__+"/output/testGUI.xml") 
       return tree

       
       
def durLessThanT(fd,filePlanName,filePlanAttribName, T ,samples):
    tree = xmlTree(filePlanName, None,filePlanAttribName)
    root = tree.getRoot()
    tree.createWrapperTreeMap("id")
    for j in range(node.parmetersInTheWorld):
	for i in range(int(samples)):           
	    root.runPlan(j)
	tmp = root.getChild(0).getLessThenTProb(j,float(T))
    fd.write("-------------------------------Astimated Duration-----------------------------------------\n")	  
    fd.write("Sampeling: # samples: "+str(samples)+" Prob: "+str(root.getChild(0).getProbAtIndex(j))+" \n") 
    fd.write("Sampeling: # samples: "+str(samples)+" T:"+str(T)+" Duration Distribution: "+str(tmp)+" \n") 
    root.treeToXml(__location__+"/output/testGUI.xml") 
    return tree
    
def updateNode(idStr, dur,T, tree):
    fd=open(__location__+"/output/outputFile","a")
    finished_node = tree.getWrappedNode(idStr)
    finished_node.setDebug("True" + " " + dur)
    tree.treeToXml(__location__+"/output/"+Ctree.filePath[6:-4]+"_after_run_debug_true.xml")
    tree = xmlTree(__location__+"/output/"+Ctree.filePath[6:-4]+"_after_run_debug_true.xml")   
    root = tree.getRoot()
    node.debugMode = True  
    for i in range(1000):
        root.runPlan(0)
    
    if root:
	fd.write("-------------------------------Astimated Duration After Step "+idStr+"-----------------------------------\n")
        fd.write("Success probability in offline mode monitor: Mission: "+ str(root.getChild(0).getLessThenTProb(0,T))+" \n")
        fd.write( "Average success time monitor: Mission: "+ str(root.getChild(0).getAverageSuccTime(0))+" \n")
	fd.write("SD success time monitor: Mission:"+ str(root.getChild(0).getSDSuccTime(0))+" \n")
	
    fd.close()
    root.treeToXml(__location__+"/output/plan_after_run.xml") 
    return(root.getChild(0).getLessThenTProb(0,T),root.getChild(0).getAverageSuccTime(0),root.getChild(0).getSDSuccTime(0))
  