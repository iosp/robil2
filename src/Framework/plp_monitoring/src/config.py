# -*- coding: utf-8 -*-
"""
Created on Wed Aug 27 16:28:41 2014

@author: liat
"""


#from Compute import *
from monitor import Compute

if __name__ == "__main__": 
    fd=open("monitor/output/outputFile","w")
    fd.write("Success Probability\n")
    filePlanName="monitor/plan.xml"
    filePlanAttribName="monitor/plan_attrib.xml"
    statProb=[1,1]  
    statDur=[0,0,1]
    runtimeProb=0 
    samplesProb=100
    T=50
    runtimeDur=0 
    accuracyDur=0
    samplesDur=100
    Compute.coputeSuccProb(fd,statProb,filePlanName,filePlanAttribName,runtimeProb,samplesProb)
    Compute.coputeDur(fd,statDur,filePlanName,filePlanAttribName,T,runtimeDur,accuracyDur,samplesDur)
    print "done"