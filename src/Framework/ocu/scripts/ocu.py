#!/usr/bin/python

from Tkinter import *
import tkMessageBox
import threading
import time
from tkFileDialog   import askopenfilename
import roslib;
from robil_msgs.msg import AssignMission,AssignManipulatorTask,AssignNavTask,MissionAcceptance
import std_msgs
import rospy, yaml
import genpy
import ttk

def parseYAML(filename):
   yamlfile=None
   if filename:
       try:
           yamlfile = yaml.load(file(filename, 'r'))
       except:
           tkMessageBox.showwarning("Error", "file cannot be parsed as yaml")
   return yamlfile

def messageFromYAML(msg_class,yamlfile):
     if yamlfile and len(yamlfile)>0:
           return parseFile(msg_class,yamlfile)
             
    
def parseFile(msg_class,myFile):
    try:           
        msg=msg_class()
        genpy.message.fill_message_args(msg,myFile)
        return msg
    except:
        tkMessageBox.showwarning("Error", "yaml and message do not contain the same fields")



class GuiHandler(object):

    def setMessageList(self,parent,title,list_row,list_column,msg_class):
        ListLabel = LabelFrame(parent, width=40, height=8,  text=title)
        ListLabel.grid(row=list_row,column=list_column)
        List = Listbox(ListLabel,selectmode=SINGLE,width=40)
        List.grid(row=1,columnspan=3)
        Button(ListLabel, text="Delete",command=lambda lb=List: lb.delete(ANCHOR)).grid(row=0,column=2)
        Button(ListLabel, text="AddFromYaml",command=self.genAddToList(List,msg_class)).grid(row=0,column=1)
        Button(ListLabel, text="Assign",command=self.genAssignMethod(List,msg_class)).grid(row=0,column=0)
        
    def __init__(self):
        
        self.TaskPublishers = dict()
        self.TaskPublishers[AssignNavTask]=rospy.Publisher('/OCU/SMME/NavigationTask', AssignNavTask)
        self.TaskPublishers[AssignMission]=rospy.Publisher('/OCU/SMME/MissionPlan', AssignMission)
        self.TaskPublishers[AssignManipulatorTask]=rospy.Publisher('/OCU/SMME/ManipulationTask', AssignManipulatorTask)
        self.decision_making_publisher=rospy.Publisher('/decision_making/events',std_msgs.msg.String)
        
        self.mainWindow = Tk()
        Label(self.mainWindow,text="OCU test",font=("Helvetica",20)).grid(row=0,columnspan=3)

        
        #control label
        controlLabel = LabelFrame(self.mainWindow, text="SMME_Control")
        controlLabel.grid(row=2,column=0)
        self.log =Text(controlLabel, state='disabled', width=40, height=6, wrap='none',font=("Purisa",12))
        self.log.grid(row=1,columnspan=6)
        self.numOfLines=0
        i=0
        for name in ["Start","Pause","Abort","Resume","Complete","Delete"]:
             Button(controlLabel, text =name, command = self.genCommand(name)).grid(row=0,column=i)
             i=i+1
        
        #task label
        TaskListLabel = LabelFrame(self.mainWindow, text="Tasks")
        TaskListLabel.grid(row=2,column=1)
        self.setMessageList(TaskListLabel,"NavTasks",1,1,AssignNavTask)
        self.setMessageList(TaskListLabel,"ManipulatorTasks",1,0,AssignManipulatorTask)
        self.setMessageList(TaskListLabel,"Missions",0,0,AssignMission)
        
        #mission status label
        self.missionStatus = StringVar()
        self.missionStatus.set("unknown status")
        statusLabel = LabelFrame(TaskListLabel, text="Status")
        statusLabel.grid(row=0,column=1)
        Label(statusLabel,text="MissionStatus:",font=("Helvetica",11),width=20).grid(row=1,column=0)
        Label(statusLabel,textvariable=self.missionStatus,font=("Helvetica",11)).grid(row=1,column=1,columnspan=2)

   
    def genAddToList(self,target,msg_class):
        def unknownAddToList():
            filename=askopenfilename()
            yamlfile=parseYAML(filename)
            if not yamlfile: return
            if messageFromYAML(msg_class,yamlfile):
                target.insert(END,filename)
        return unknownAddToList
        
    
    def genAssignMethod(self,listTarget,msg_class):
        def assignMethod():
            for sel in listTarget.curselection():
                yamlfile=parseYAML(listTarget.get(sel))
                msg=messageFromYAML(msg_class,yamlfile)
                self.TaskPublishers[msg_class].publish(msg)
            if not len(listTarget.curselection()):
                tkMessageBox.showinfo("Not assignd", "please choose a value form the list before assigning")
        return assignMethod
        
    def writeToLog(self,msg):
        self.numOfLines = self.numOfLines+1
        self.log['state'] = 'normal'
        if self.numOfLines > 6 :
             self.numOfLines= self.numOfLines -1
             self.log.delete("1.0", "2.0")
        if self.log.index('end-1c')!='1.0':
             self.log.insert('end', '\n')
        self.log.insert('end', msg)
        self.log['state'] = 'disabled'

    def writeToStatusLabel(self,msg):
        self.missionStatus.set(msg)
       
    def genCommand(self,name):
        def commandForName():
            msg=std_msgs.msg.String()
            msg.data="/SMME/"+name
            self.decision_making_publisher.publish(msg)
            self.writeToLog(name+" sent")
        return commandForName

class rosSubscriberThread (threading.Thread):
    
    def missionAcceptanceCallback(self,msg):
        self.gui.writeToStatusLabel("MissionAcceptance id:"+msg.mission_id + ", status:" + str(msg.status) + " , at:"+msg.mission_assign_stamp.__str__())
            
    def __init__(self,gui):
        threading.Thread.__init__(self)
        #place ros stuff here
        self.threadID = 0
        self.gui=gui
        self.statusSubscriber=rospy.Subscriber("/SMME/OCU/MissionAcceptance",MissionAcceptance, self.missionAcceptanceCallback)
        self.statusSubscriber=rospy.Subscriber("/SMME/OCU/Mission",MissionAcceptance, self.missionAcceptanceCallback)
        
    def run(self):
        print "Starting "
        rospy.spin()
        print "Exiting "
        
    def stop(self):
		rospy.signal_shutdown("die!")
		exit(0)


rospy.init_node("ocu_gui")
top = GuiHandler()

# Create new threads
thread1 = rosSubscriberThread(top)

# Start new Threads
thread1.start()


top.mainWindow.mainloop()
thread1.stop()
exit(0)
