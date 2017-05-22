#!/usr/bin/python

from Tkinter import *
import tkSimpleDialog
import tkMessageBox
import threading
import time
from tkFileDialog   import askopenfilename
import roslib;
from robil_msgs.msg import AssignMission,AssignManipulatorTask,AssignNavTask,MissionAcceptance ,IEDLocation,Map
from robil_msgs.srv import MissionState
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

import std_msgs
import rospy, yaml
import genpy
import ttk
from tf import transformations
from PIL import Image, ImageTk
from subprocess import call

from inspect import getsourcefile
import os
import re,sys

import math
import cv2
import numpy as np

def calcDistance(p1,p2):
    R = 6371.0
    lat1 = p1[1]*math.pi/180.0
    lat2 = p2[1]*math.pi/180.0
    dLat = lat2 - lat1
    dLon = (p2[2] - p1[2])*math.pi/180.0
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.sin(dLon/2) * math.sin(dLon/2) * math.cos(lat1) * math.cos(lat2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R*c
    print p1
    print p2
    return d*1000;

def calcBearing(p1,p2):

    lat1 = p1[1]*math.pi/180.0
    lat2 = p2[1]*math.pi/180.0
    dLat = lat2 - lat1
    dLon = (p2[2] - p1[2])*math.pi/180.0
    y = math.sin(dLon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon);
    return math.atan2(y, x);

def read_fileAux():
    file = open(os.getenv("HOME")+"/.ros/gps_init.txt", "r")
    return [float(re.split(" ", line)[1][:-1]) for line in file.readlines()[1:]]
    
def geo2xy(point,bearing=0):
    init_coor = read_fileAux()
    d = calcDistance(point,init_coor)
    theta = calcBearing(init_coor,point)
    return [d * math.cos(theta),d * math.sin(theta),point[0] - init_coor[0]]

def rotateXY(x,y,yaw):
    dx=(x*math.cos(yaw)+y*math.sin(yaw))
    dy=(y*math.cos(yaw)-x*math.sin(yaw))
    return dx,dy


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
        ButtonLabel = LabelFrame(ListLabel)
        Button(ButtonLabel, text="Delete",command=lambda lb=List: lb.delete(ANCHOR)).pack(side=RIGHT)
        Button(ButtonLabel, text="AddFromYaml",command=self.genAddToList(List,msg_class)).pack(side=RIGHT)
        Button(ButtonLabel, text="Assign",command=self.genAssignMethod(List,msg_class)).pack(side=RIGHT)
        Button(ButtonLabel, text="Edit",command=lambda lb=List: call(["gedit", lb.get(ANCHOR)]) if len(lb.curselection())>0 else None ).pack(side=LEFT)
            
        scrolbar =Scrollbar(ListLabel,orient=HORIZONTAL)
        scrolbar.config(command=List.xview)
        List.config(xscrollcommand=scrolbar.set)
        scrolbar.pack(side=BOTTOM,fill=X)
        List.pack(side=BOTTOM)
        ButtonLabel.pack(side=TOP , fill=X)
        self.lists[title]=List

        
    def __init__(self):
        
        self.TaskPublishers = dict()
        self.TaskPublishers[AssignNavTask]=rospy.Publisher('/OCU/SMME/NavigationTask', AssignNavTask,queue_size=1)
        self.TaskPublishers[AssignMission]=rospy.Publisher('/OCU/SMME/MissionPlan', AssignMission,queue_size=1)
        self.TaskPublishers[AssignManipulatorTask]=rospy.Publisher('/OCU/SMME/ManipulationTask', AssignManipulatorTask,queue_size=1)
        self.decision_making_publisher=rospy.Publisher('/decision_making/events',std_msgs.msg.String,queue_size=1)
        self.lists=dict()
        self.mapImage=0
        
        self.mainWindow = Tk()
        Label(self.mainWindow,text="OCU console",font=("Helvetica",20)).pack(side=TOP)

        leftWrapper=Label(self.mainWindow)
        rightWrapper=Label(self.mainWindow)

        leftWrapper.pack(side=LEFT)
        rightWrapper.pack(side=RIGHT)
       
        #status label
        statusLabel = LabelFrame(leftWrapper, text="Status",width=40,height=40)
        statusLabel.pack(side=TOP)
        
        self.missionStatus = StringVar()
        self.missionStatus.set("unknown status")
        Label(statusLabel,text="MissionStatus:",font=("Helvetica",11),width=20).grid(row=1,column=1)
        Label(statusLabel,textvariable=self.missionStatus,font=("Helvetica",11)).grid(row=1,column=2,columnspan=2)
        
        self.IEDStatus = StringVar()
        self.IEDStatus.set("unknown status")
        Label(statusLabel,text="IEDStatus:",font=("Helvetica",11),width=20).grid(row=2,column=1)
        Label(statusLabel,textvariable=self.IEDStatus,font=("Helvetica",11),height=8,width=20).grid(row=2,column=2,rowspan=6)
        
        Button(statusLabel, text="SetIED",command=lambda parent=self.mainWindow: IEDDialog(parent)).grid(row=2,column=0)
        
        #dignostics
        diagnosticsLabel = LabelFrame(leftWrapper, text="diagnostics")
        diagnosticsLabel.pack(side=TOP)
        scrolbar =Scrollbar(diagnosticsLabel)
        self.diaglog =Text(diagnosticsLabel, state='disabled', width=40, height=20, wrap='none',font=("Helvetica",11))
        Button(diagnosticsLabel, text="Clear",command=lambda parent=self: parent.clearDiagLog() ).pack(side=TOP)
        scrolbar.config(command=self.diaglog.yview)
        self.diaglog.config(yscrollcommand=scrolbar.set)
        scrolbar.pack(side=RIGHT, fill=Y)
        self.diaglog.pack(side=LEFT, fill=Y)
        self.numOfDiagLogLines=0
        
        
        #map image
        self.mapImage=Label(master=rightWrapper)
        self.mapImage.pack(side=BOTTOM)
        
        
        #task label
        robil2=os.environ.get('ROBIL2')
        print robil2
        if robil2 == None:
            init_dialog=False
        else:
            init_dialog=True
            fullpath=robil2+"/src/Framework/ocu/scripts/"
        
        TaskListLabel = LabelFrame(rightWrapper, text="Tasks")
        TaskListLabel.pack(side=BOTTOM)
        self.setMessageList(TaskListLabel,"NavTasks",1,1,AssignNavTask)
        
        if init_dialog :
            init_filename=fullpath+"Nav_11.yaml"
        #init_filename="/home/michele/robil2/src/Framework/ocu/scripts/Nav_11.yaml"
            yamlfile=parseYAML(init_filename)
            if not yamlfile: return
            if messageFromYAML(AssignNavTask,yamlfile):
                self.lists["NavTasks"].insert(END,init_filename)
        if init_dialog :
            init_filename=fullpath+"Nav_12.yaml"
        #init_filename="/home/michele/robil2/src/Framework/ocu/scripts/Nav_11.yaml"
            yamlfile=parseYAML(init_filename)
            if not yamlfile: return
            if messageFromYAML(AssignNavTask,yamlfile):
                self.lists["NavTasks"].insert(END,init_filename)
                
        self.setMessageList(TaskListLabel,"ManipulatorTasks",1,0,AssignManipulatorTask)
        #init_filename="/home/michele/robil2/src/Framework/ocu/scripts/WSM_23.yaml"
        if init_dialog:
            init_filename=fullpath+"WSM_23.yaml"
            yamlfile=parseYAML(init_filename)
            if not yamlfile: return
            if messageFromYAML(AssignManipulatorTask,yamlfile):
                self.lists["ManipulatorTasks"].insert(END,init_filename)
                
        self.setMessageList(TaskListLabel,"Missions",0,0,AssignMission)
        #init_filename="/home/michele/robil2/src/Framework/ocu/scripts/Mission3.yaml"
        if init_dialog:
            init_filename=fullpath+"Mission3.yaml"
            yamlfile=parseYAML(init_filename)
            if not yamlfile: return
            if messageFromYAML(AssignMission,yamlfile):
                self.lists["Missions"].insert(END,init_filename)
        
        #control label
        controlLabel = LabelFrame(TaskListLabel, text="Mission_Control")
        controlLabel.grid(row=0,column=1)
        self.log =Text(controlLabel, state='disabled', width=40, height=6, wrap='none',font=("Purisa",12))
        self.log.grid(row=1,columnspan=7)
        self.numOfLogLines=0
        i=0
        for name in ["Start","Complete","Pause","Abort","Resume","Clear","Delete"]:
             Button(controlLabel, text =name, command = self.genCommand(name,self.lists["Missions"],AssignMission)).grid(row=0,column=i)
             i=i+1


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
        self.numOfLogLines = self.numOfLogLines+1
        self.log['state'] = 'normal'
        if self.numOfLogLines > 6 :
             self.numOfLogLines= self.numOfLogLines -1
             self.log.delete("1.0", "2.0")
        if self.log.index('end-1c')!='1.0':
             self.log.insert('end', '\n')
        self.log.insert('end', msg)
        self.log['state'] = 'disabled'
        
    def writeToDiagLog(self,msg):
        self.numOfDiagLogLines = self.numOfDiagLogLines+1
        self.diaglog['state'] = 'normal'
        if self.numOfDiagLogLines > 1 :
             self.numOfDiagLogLines= self.numOfDiagLogLines -1
             self.diaglog.delete("1.0", "end")
        if self.diaglog.index('end-1c')!='1.0':
             self.diaglog.insert('end', '\n')
        self.diaglog.insert('end', msg)
        self.diaglog['state'] = 'disabled'
        
    def clearDiagLog(self):
        self.diaglog['state'] = 'normal'
        self.diaglog.delete(1.0, 'end')
        self.diaglog['state'] = 'disabled'

    def writeToStatusLabel(self,msg):
        self.missionStatus.set(msg)
    
    def writeToIEDStatusLabel(self,msg):
        self.IEDStatus.set(msg)
        
    def updateMapImage(self,img): 
        b = ImageTk.PhotoImage(image=Image.fromarray( cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)))
        self.mapImage.configure(image=b)
        self.mapImage._image_cache = b 
        self.mapImage.update()
        
    def genCommand(self,name,listTarget,msg_class):
        def commandForName():
            for sel in listTarget.curselection():
                yamlfile=parseYAML(listTarget.get(sel))
                missionMsg=messageFromYAML(msg_class,yamlfile)
                msg=std_msgs.msg.String()
                msg.data="/mission/"+missionMsg.mission_id+"/"+name+"Mission"
                self.decision_making_publisher.publish(msg)
                self.writeToLog(name+" sent")
            if not len(listTarget.curselection()):
                tkMessageBox.showinfo("Not assignd", "please choose a value form the list before assigning")
        return commandForName

        

class IEDDialog(tkSimpleDialog.Dialog):
    
    def body(self, master): 
        self.ied_publisher=rospy.Publisher('/OCU/IED/Location',IEDLocation)
        Label(master, text="latitude/x:").grid(row=0)
        Label(master, text="longitude/y:").grid(row=1)
        Label(master, text="elevation/z:").grid(row=2)

        self.cordSys=IntVar()
        Radiobutton(master,text= "xyz", variable=self.cordSys, value="1").grid(row=3,column=1)
        radio=Radiobutton(master,text= "geo", variable=self.cordSys, value="0")
        radio.select()
        radio.grid(row=3,column=0)
        self.e1 = Entry(master)
        self.e2 = Entry(master)
        self.e3 = Entry(master)
        
        self.e1.grid(row=0, column=1)
        self.e2.grid(row=1, column=1)
        self.e3.grid(row=2, column=1)
        
        return self.e1 # initial focus

    def validate(self):
        try:
            choice=self.cordSys.get()
            self.result = choice,float(self.e1.get()) , float(self.e2.get()) ,float(self.e3.get())
            return 1
        except ValueError:
            tkMessageBox.showwarning(
                "Bad input",
                "Illegal values, please try again"
            )
            return 0

    def apply(self):
        msg = IEDLocation()
        print self.result
        if self.result[0]:
            msg.location.x=self.result[1]
            msg.location.y=self.result[2]
            msg.location.z=self.result[3]
        else:
            x,y,z=geo2xy([self.result[3],self.result[1],self.result[2]])
            msg.location.x=x
            msg.location.y=y
            msg.location.z=z
        print msg
        self.ied_publisher.publish(msg)

class rosSubscriberThread (threading.Thread):

    def missionAcceptanceCallback(self,msg):
        self.gui.writeToStatusLabel("id:"+msg.mission_id + ", status:" + str(msg.status) + " , at:"+msg.mission_assign_stamp.__str__())
            
    def diagnosticsCallback(self,msg):
         self.gui.writeToDiagLog(msg.__str__())
         
    def iedLocationCallback(self,msg):
         self.gui.writeToIEDStatusLabel(msg.__str__())   
     
    def mapCallback(self,msg):
        multiplier=2.0
        height=int(multiplier*msg.info.height)
        width=int(multiplier*msg.info.width)
        #draw map
        for row in range(0,height):
            for col in range(0,width):
                if msg.data[int(int(row/multiplier)*msg.info.width+int(col/multiplier))].height<=-3:
                    self.image[col,row]=[0,0,100]
                else:
                    c=((msg.data[int(int(row/multiplier)*msg.info.width+int(col/multiplier))].height+3)/6)
                    self.image[col,row]=[120*(1-c),240,240]
        self.image= cv2.cvtColor(self.image, cv2.COLOR_HSV2BGR)
        #draw obstacles
        for row in range(0,height):
            for col in range(0,width):
                if msg.data[int(int(row/multiplier)*msg.info.width+int(col/multiplier))].type == 2:
                     self.image[col,row]=[255,0,0]
        #draw plan
        yaw,pitch,roll=transformations.euler_from_quaternion([msg.info.origin.orientation.w,msg.info.origin.orientation.x,msg.info.origin.orientation.y,msg.info.origin.orientation.z])
        yaw=yaw+math.pi/2
        index=0
        #print msg.info.origin.position.x ,msg.info.origin.position.y
        mapCenterX,mapCenterY=rotateXY(75*msg.info.resolution,75*msg.info.resolution,yaw)
        
        mapCenterX=msg.info.origin.position.x-mapCenterX
        mapCenterY=msg.info.origin.position.y-mapCenterY
        
        #testNotROt =np.zeros((150*2,150*2,3), np.uint8)
        #test =np.zeros((150*2,150*2,3), np.uint8)
        
        for pose in self.plan.poses:
            index+=1
            #rotX,rotY=rotateXY(pose.pose.position.x,pose.pose.position.y,-yaw)
            dy=pose.pose.position.x - msg.info.origin.position.x
            dx=pose.pose.position.y - msg.info.origin.position.y
            '''dx=(pose.pose.position.x*math.cos(yaw)+pose.pose.position.y*math.sin(yaw))
            dy=(pose.pose.position.y*math.cos(yaw)-pose.pose.position.x*math.sin(yaw))
            dx=dx - msg.info.origin.position.x
            dy=dy - msg.info.origin.position.y'''
            #Xpos=dx/msg.info.resolution*multiplier+height/2
            #Ypos=dy/msg.info.resolution*multiplier+height/2
            
            Xpos=dx/msg.info.resolution*multiplier+height/2
            Ypos=dy/msg.info.resolution*multiplier+height/2
            #print yaw
            dx1,dy1=rotateXY(dx,dy,yaw)
#           print msg.info.origin.position.x, msg.info.origin.position.y
#           print Xpos, Ypos
            #cv2.circle(testNotROt,(int(Ypos),int(Xpos)),2,[255,0,float(index)/len(self.plan.poses)*255],-1)
            
            Xpos=dx1/msg.info.resolution*multiplier+height/2
            Ypos=dy1/msg.info.resolution*multiplier+height/2
            
            #fix y orientation
            Ypos=height-Ypos
            #cv2.circle(test,(int(Ypos),int(Xpos)),2,[255,0,float(index)/len(self.plan.poses)*255],-1)
            '''
            
            Xpos=dx/0.2*multiplier+height/2#int(((dx*math.cos(yaw)+dy*math.sin(yaw))/0.2)*multiplier)+height/2
            Ypos=dy/0.2*multiplier+width/2#int(((dy*math.cos(yaw)-dx*math.sin(yaw))/0.2)*multiplier)+width/2'''
            if Xpos < height and Xpos >= 0 and Ypos < width and Ypos >= 0 :
                cv2.circle(self.image,(int(Ypos),int(Xpos+50)),2,[0,0,float(index)/len(self.plan.poses)*255],-1)
        
        #cv2.imshow('Map',test)
        #cv2.imshow('MapOrig',testNotROt)
        #cv2.waitKey(8)
        self.gui.updateMapImage(self.image)
    
    def drawCurrentPlan(self,image,height,width,multiplier):
        pass
        '''for pose in self.currentPlan:
           newP=(height*3.0/4.0 (pose[0]-first[0])*5*multiplier,(pose[1]-first[1])*5*multiplier+width/2)
           if newP[0] > height or newP[0] <0 or newP[1] >width or newP[1] <0:
              continue
           image[newP[0],newP[1]]=[0,0,0]'''
           
    def planCallback(self,msg):
        self.plan=msg
        '''basePose=0
        for pose in msg.poses:
            yaw,pitch,roll=transformations.euler_from_quaternion([self.pose.pose.pose.orientation.w,self.pose.pose.pose.orientation.x,self.pose.pose.pose.orientation.y,self.pose.pose.pose.orientation.z])
            
            dx=pose.pose.position.x - self.pose.pose.pose.position.x
            dy=pose.pose.position.y - self.pose.pose.pose.position.y
            self.currentPlan.append( (dx*math.cos(yaw)+dy*math.sin(yaw),dy*math.cos(yaw)-dx*math.sin(yaw)))'''

    def __init__(self,gui):
        threading.Thread.__init__(self)
        #place ros stuff here
        self.threadID = 0
        self.currentPlan = []
        self.gui=gui
        self.image=np.zeros((150*2,150*2,3), np.uint8)
        self.plan=Path()
        self.statusSubscriber=rospy.Subscriber("/SMME/OCU/MissionAcceptance",MissionAcceptance, self.missionAcceptanceCallback)
        self.statusSubscriber=rospy.Subscriber("/SMME/OCU/Mission",MissionAcceptance, self.missionAcceptanceCallback)
        #self.statusSubscriber=rospy.Subscriber("/diagnostics",DiagnosticArray, self.diagnosticsCallback)
        self.statusSubscriber=rospy.Subscriber("/IED/Location",IEDLocation, self.iedLocationCallback)
        rospy.Subscriber("/PER/Map",Map, self.mapCallback)
        rospy.Subscriber("/move_base/NavfnROS/plan",Path, self.planCallback)
 
    def run(self):
        print "Starting "
        rospy.spin()
        print "Exiting "
        
    def stop(self):
        rospy.signal_shutdown("die!")
        exit(0)

class rosServiceClientThread (threading.Thread):

    def diagnosticsCallback(self,msg):
         self.gui.writeToDiagLog(msg.__str__())

    def __init__(self,gui):
        threading.Thread.__init__(self)
        #place ros stuff here
        self.threadID = 1
        self.gui=gui
 
    def run(self):
        rospy.wait_for_service('mission_state')
        try:
            add_two_ints = rospy.ServiceProxy('mission_state', MissionState)
            while True:
                resp1 = add_two_ints()
                self.diagnosticsCallback(resp1)
                rospy.sleep(1)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def stop(self):
        rospy.signal_shutdown("die!")
        exit(0)
    

rospy.init_node("ocu_gui")

top = GuiHandler()

# Create new threads
thread1 = rosSubscriberThread(top)
thread2 = rosServiceClientThread(top)
# Start new Threads
thread1.start()
thread2.start()

top.mainWindow.mainloop()
thread1.stop()
exit(0)

