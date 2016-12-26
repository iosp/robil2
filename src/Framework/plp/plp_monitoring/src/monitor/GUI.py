# -*- coding: utf-8 -*-
"""
Created on Sat Jun 21 15:25:14 2014

@author: liat
"""
from Tkinter import *
from tkFileDialog import askopenfilename
import tkMessageBox
from tree import xmlTree
from Node import node
from computeTree import *
import time
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import *


class GUI:
    
   def __init__(self,name,size,startLoc):
        self.master = Tk()
        self.master.title(name)
        self.master.geometry(size)
        self.startLoc=startLoc
                
   def addButton(self,name,call_back,locx, locy):
       Button(self.master, text=name, command=call_back).place(relx=locx, rely=self.startLoc+locy)
        
   def addLable(self,text,font,locx,locy):
       Label(self.master, text=text,font = font).place(relx=locx, rely=self.startLoc+locy)
   
   def addLableRes(self,text,font,locx,locy):
       Label(self.master, text=text,font = font).place(relx=locx, rely=self.startLoc+locy, width=60)
   
   
   def addLableRange(self,text,font,locx,locy):
       Label(self.master, text=text,font = font).place(relx=locx, rely=self.startLoc+locy, width=160)
  
   def addEntry(self,text,font,var,width,defultText,locx,locy,offset):
       self.addLable(text,font,locx,locy)
       entry=Entry(self.master, textvariable=var, width=width)
       entry.pack()
       entry.insert(0,defultText)
       entry.config(state=DISABLED)
       entry.place(relx=locx+offset, rely=self.startLoc+locy)       
       return entry
       
   def addCheckBox(self,text,var,call_back,locx,locy):
       cb=Checkbutton(self.master, text=text, variable = var, command=call_back).place(relx=locx, rely=self.startLoc+locy)  
       return cb   
    
   def addText(self,text,font, height,width,locx,locy,offset):
       self.addLable(text,font,locx,locy)
       t=Text(self.master, height=height, width=width)
       t.pack()
       t.place(relx=locx+offset, rely=self.startLoc+locy) 
       return t        
       
   def runGUI(self):
       self.master.mainloop()
       
#------------------------------AND CLASS--------------------------------------------

def loadCallback():
    textPlanFileName.delete("1.0",END)
    textAttribFileName.delete("1.0",END)
    filePlanName.set(askopenfilename()) 
    textPlanFileName.insert(INSERT, filePlanName.get())
    
    filePlanAttribName.set(askopenfilename()) 
    textAttribFileName.insert(INSERT, filePlanAttribName.get())
    
    if exportVarSuccProb.get() or exportVarDur.get():
        fd.write(filePlanName.get()+"\n"+filePlanAttribName.get()+"\n")
    

def enableEntryAcc():
    if CheckVar1.get():
        entryRTSuccProb.configure(state=NORMAL)
        entryRTSuccProb.update()
    else:
        entryRTSuccProb.configure(state=DISABLED)
        entryRTSuccProb.update() 
        
def enableEntrySamp():
    if CheckVar2.get():
        entrySuccProb.configure(state=NORMAL)
        entrySuccProb.update()
    else:
        entrySuccProb.configure(state=DISABLED)
        entrySuccProb.update() 
        
def coputeButtSuccProb():
   #if flagCompute1.get()==0:
   
   node.parmetersInTheWorld = 1
   #root.treeToXml("output/testforGUI.xml")  
   node.debugMode = False 
   
   if CheckVar1.get():
       tree = xmlTree(filePlanName.get(), None,filePlanAttribName.get())
       root = tree.getRoot()
       root.treeToXml("output/testforGUI.xml")  
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)
           gui.addLableRes(root.getChild(0).getProbAtIndex(j), None,0.5, 0.25)    
       if exportVarSuccProb.get():
          fd.write("Accurate: Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml("output/testGUI1.xml") 
     
   if CheckVar2.get(): 
       tree = xmlTree(filePlanName.get(), None,filePlanAttribName.get())
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           for i in range(int(varSamplSuccProb.get())):           
               root.runPlan(j)
           gui.addLableRes(root.getChild(0).getProbAtIndex(j), None,0.5, 0.35)    
           if exportVarSuccProb.get():
              fd.write("Sampeling: # samples: "+varSamplSuccProb.get()+" Result: "+str(root.getChild(0).getProbAtIndex(j))+" \n")                              
       root.treeToXml("output/testGUI2.xml") 
     
       
def coputeButtDur():   
   node.parmetersInTheWorld = 1
   node.debugMode = False            
   
   if CheckVar12.get():
       tree = xmlTree(filePlanName.get(), None,filePlanAttribName.get())
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           root.runPlanAccurate(j)
           tmp = root.getChild(0).getLessThenTProb(j,float(varT.get()))
           gui.addLableRes(root.getChild(0).getLessThenTProb(j,float(varT.get())), None,0.5, 0.62)    
       if exportVarSuccProb.get():
          fd.write("Accurate: "+"T:"+varT.get()+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("output/testGUI1.xml")  
       
   if CheckVar22.get():  
       tree = xmlTree(filePlanName.get(), None,filePlanAttribName.get())
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           root.runPlanApproximate(j,float(varAppParam.get()),float(varT.get()))
           tmp = root.getChild(0).getLessThenTProb(j,float(varT.get()))
           gui.addLableRes(root.getChild(0).getLessThenTProb(j,float(varT.get())), None,0.5, 0.72)    
           gui.addLable("Error Range:[",None, 0.6, 0.72)
           gui.addLableRange(str(root.getChild(0).getLBound(j))+","+str(root.getChild(0).getUBound(j))+"]",None,0.71,0.72)                                
       if exportVarSuccProb.get():
          fd.write("Approxite: "+"T:"+varT.get()+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("output/testGUI2.xml") 
       
   if CheckVar32.get(): 
       tree = xmlTree(filePlanName.get(), None,filePlanAttribName.get())
       root = tree.getRoot()
       for j in range(node.parmetersInTheWorld):
           for i in range(int(varSamplDur.get())):           
               root.runPlan(j)
           gui.addLableRes(root.getChild(0).getLessThenTProb(j,float(varT.get())), None,0.5, 0.82)               
           tmp = root.getChild(0).getLessThenTProb(j,float(varT.get()))
           if exportVarSuccProb.get():
              fd.write("Sampeling: # samples: "+varSamplSuccProb.get()+"T:"+varT.get()+" Result: "+str(tmp)+" \n")                              
       root.treeToXml("output/testGUI3.xml") 
# check box
def enableEntryAccDur():
    if CheckVar12.get():
        entryAccDur.configure(state=NORMAL)
        entryAccDur.update()
    else:
        entryAccDur.configure(state=DISABLED)
        entryAccDur.update() 

def enableEntryAppDur():
    if CheckVar22.get():
        entryAppDur.configure(state=NORMAL)
        entryAppDur.update()
    else:
        entryAppDur.configure(state=DISABLED)
        entryAppDur.update()
        
def enableEntrySampDur():
    if CheckVar32.get():
        entryDur.configure(state=NORMAL)
        entryDur.update()
    else:
        entryDur.configure(state=DISABLED)
        entryDur.update() 
        
if __name__ == "__main__":        
    #print sys.argv[0], len(sys.argv)    
    gui=GUI("Robil2 Plan Analysis", "800x600",0) 
    filePlanName=StringVar()
    filePlanAttribName=StringVar()
    fd=open("output/outputFile","w")
    fd.write("Success Probability\n")
    flagCompute1=IntVar()
    flagCompute1.set(0)
    flagCompute2=IntVar()
    flagCompute2.set(0)
    # Plan file name    
    textPlanFileName = gui.addText("Plan:",None,1,70,0.26,0.05,0.09) 
    
    # Attrib file name
    textAttribFileName = gui.addText("Attributes:",None,1,70,0.26,0.1,0.09) 
    
    # load button
    gui.addButton("Load Plan and \n Attributes", loadCallback,0.1,0.055)
    
    # lable first section success probability
    gui.addLable("Plan Success Probability:","bold", 0.1,0.15)
    
    # check box accurate
    CheckVar1 = IntVar()
    gui.addCheckBox("Accurate", CheckVar1, enableEntryAcc,0.1,0.2)
    
    # smpling parameter for Succ Prob
    varAccSuccProb=StringVar()
    entryRTSuccProb=gui.addEntry("Max run-time (sec):",None,varAccSuccProb, 20,"1",
                                 0.4,0.2,0.16)
    # result for Succ Prob accurate
    labelAccResultSuccProb = gui.addLable("Success Probability of the given Plan:",None,
                                          0.1,0.25)
    # check box sampling
    CheckVar2 = IntVar()
    gui.addCheckBox("Approximate - Sampling",CheckVar2, enableEntrySamp,0.1,0.3)
    
    # smpling parameter for Succ Prob
    varSamplSuccProb=StringVar()
    entrySuccProb=gui.addEntry("# Samples:",None,varSamplSuccProb,20,"100",0.4,0.3,0.1)
    
    # result for Succ Prob accurate
    labelAppResultSuccProb = gui.addLable("Approximated Success Probability of the given Plan:",
                                          None,0.1,0.35)
    # check compute button      
    gui.addButton('Compute', coputeButtSuccProb,0.1, 0.40)
      
    #export to file suuc prob
    exportVarSuccProb = IntVar()
    gui.addCheckBox("Export to file",exportVarSuccProb,None,0.3,0.4)
    
    # lable second section Duration Less then T
    gui.addLable("Probability that Plan Duration Takes Less then T time units:","bold",0.1,0.47)
    
    #get number of samples
    varT=StringVar()
    entryT = gui.addEntry("Please enter T (sec):",None,varT,None,"",0.1,0.52,0.17)
    entryT.config(state=NORMAL)
    
    #check box
    CheckVar12 = IntVar()
    gui.addCheckBox("Accurate",CheckVar12, enableEntryAccDur,0.1,0.57)
    
    # smpling parameter for Succ Prob
    varAccDur=StringVar()
    entryAccDur=gui.addEntry("Max run-time (sec):",None,varAccDur,20,"1",0.4,0.57,0.16)
    
    # result for Succ Prob accurate
    labelAccResultDur = gui.addLable("Accurete Probability:",None,0.1,0.62)
    
    #check box
    CheckVar22 = IntVar()
    gui.addCheckBox("Approximate",CheckVar22,enableEntryAppDur,0.1,0.67)
    #get accuracy paremeter
    varAppParam=StringVar()
    entryAppDur=gui.addEntry("Accuracy Parameter (0,1):", None,varAppParam,20,"0.1",0.4,0.67,0.21)
        
    # result for Succ Prob accurate
    labelAppResultDur=gui.addLable("Approximated Probability:",None,0.1,0.72)
    
    #check box
    CheckVar32 = IntVar()
    gui.addCheckBox("Approximate - Sampling",CheckVar32, enableEntrySampDur,0.1,0.77)
    
    #get number of samples
    varSamplDur=StringVar()
    entryDur=gui.addEntry("# Samples:", None,varSamplDur,20,"100",0.4,0.77,0.1)
    
    # result for Succ Prob accurate
    labelSampResultDur = gui.addLable("Approximated by Sampling Probability:",None,0.1,0.82)
    
    # check compute button
    gui.addButton('Compute',coputeButtDur,0.1,0.87)
    
    #export to file duration
    exportVarDur = IntVar()
    gui.addCheckBox("Export to file",exportVarDur,None,0.3,0.87)
    
    #QUIT
    gui.addButton('Quit', gui.master.quit,0.90, 0.90)
    
    gui.runGUI()  
    

    
        