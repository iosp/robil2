# -*- coding: utf-8 -*-
"""
Created on Wed Aug 27 16:28:41 2014

@author: liat
"""


#from Compute import *
from monitor import Compute
import os

__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))


def config():
    fd=open(__location__+"/monitor/output/outputFile","w")
    fd.write("Success Probability\n")
    filePlanName=__location__+"/monitor/plan.xml"
    filePlanAttribName=__location__+"/monitor/plan_attrib.xml"
    statProb=[0,1]  
    statDur=[0,0,1]
    runtimeProb=0 
    samplesProb=100
    T=50
    runtimeDur=0 
    accuracyDur=0
    samplesDur=1000
    Compute.coputeSuccProb(fd,statProb,filePlanName,filePlanAttribName,runtimeProb,samplesProb)
    tree=Compute.coputeDur(fd,statDur,filePlanName,filePlanAttribName,T,runtimeDur,accuracyDur,samplesDur)
    print "done"
    return tree