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

def coputeSuccProb(fd,stat, filePlanName,filePlanAttribName, runtime=0, samples=0):
   #if flagCompute1.get()==0:
   
   node.parmetersInTheWorld = 1
   #root.treeToXml("output/testforGUI.xml")  
   node.debugMode = False 
   
   if stat[0]:
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       root.treeToXml("monitor/output/testforGUI.xml")  
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)       
       fd.write("Accurate: Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml("monitor/output/testGUI1.xml") 
     
   if stat[1]: 
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           for i in range(int(samples)):           
               root.runPlan(j)           
           fd.write("Sampeling: # samples: "+str(samples)+" Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml("monitor/output/testGUI2.xml") 
     
       
def coputeDur(fd,stat, filePlanName,filePlanAttribName, T, runtime=0, accuracy=0 ,samples=0):   
   node.parmetersInTheWorld = 1
   node.debugMode = False            
   
   if stat[0]:
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)
           tmp = root.getChild(0).getLessThenTProb(j,float(T))       
       fd.write("Accurate: "+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("monitor/output/testGUI1.xml")  
       
   if stat[1]:  
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           root.runPlanApproximate(j,float(accuracy),float(T))
           tmp = root.getChild(0).getLessThenTProb(j,float(T))
       
       fd.write("Approxite: "+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("monitor/output/testGUI2.xml") 
       
   if stat[2]: 
       tree = xmlTree(filePlanName, None,filePlanAttribName)
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           for i in range(int(samples)):           
               root.runPlan(j)
           tmp = root.getChild(0).getLessThenTProb(j,float(T))
       fd.write("Sampeling: # samples: "+str(samples)+"T:"+str(T)+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("monitor/output/testGUI3.xml") 