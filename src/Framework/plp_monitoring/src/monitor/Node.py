# -*- coding: utf-8 -*-
"""
node class. this class is the base class for all the node type in the tree.
global variable- parmetersInTheWorld- represent the amount of parm we have
               - debugMode- represent the run mode 
this class maintain the updates between the calculated arguments such as probability, distributions and debug,
and update the etree in order to print it to xml file.
as well read from xml file to etree and then wrap the etree in order to calculate diff arguments.
         
"""
import random
import math
import re
from lxml import etree
from copy import deepcopy
#DISTRIBUTIONS
from distributions.computed import Computed
from distributions.normal import Normal
from distributions.uniform import Uniform
import SumRandomVariables

class node:
    
    #global variable - represents the amount of different parmeters of the world.
    #it is a class attribute and can accessed via class node.parameterInTheWorld
    #can be set from everywhere that import node
    parmetersInTheWorld = 1
    debugMode = False
    
    #constractur- treeInstance-node in the etree, the etree itself, and prep-type(seq,plan etc.)
    def __init__(self,treeInstance = None,mytree = None,prep="plan",parent=None):
        # you can't have multiple __init__ functions in Python so we use mytree = None
        if mytree == None :
	  #create a new tree instance with plan node as root
            self.treeInst =  etree.Element("plan")
            from tree import xmlTree
            #tree instance new with plan as root
            self.myTree = xmlTree(None,self.treeInst)
        else:
            self.myTree = mytree
            self.treeInst = treeInstance

        self.parent = parent 
        # monitor - boolean property, default-True
        self.monitor = True
        #node child list
        self.childList = []
        #node probebility table
        self.probTable = []
        # node distribution table for success and failure
        #distribution table - each entry points to a distribution
        self.distTableSucc = self._createDistTable("Successdistribution")
        self.distTableFail = self._createDistTable("Failuredistribution")
        #update probability table
        probString = self.getAttrib("probability")
        if probString !=None:
           # self.probTable= self._parseString(probString)
            self.probTable = self._createProbTable(probString)
        else:
            self.probTable= None
        
        #node debuge child property

        #DEBUGchild -  boolean value if we have a debug node in the sub-tree which was already debug- default- False
        self.DEBUGchild= False   
        self._updateChildDebug()
        # DEBUG - list of two parameters first elem is boolean, second parm- is float        
        self.DEBUG = self._setDebugFromXmlFile()
	
	#flag that indicates if this node was updated after debug
        self.reset = False
        self.upperBound = [0]*node.parmetersInTheWorld
        self.lowerBound = [0]*node.parmetersInTheWorld    
    #parseString by whiteSpace
    def _parseString(self, string):
        words = re.split('\s+',string)
	   #return a list of words seperate by whiteSpace
        return words
    #create probtalbe- parse string to list of float
    def _createProbTable(self,stringProbList):
        probList = self._parseString(stringProbList)        
       # for index in range(len(probList)):
       #     probList[index] =float(probList[index])
        return probList
        
        
    #return parent. if it's the root- return None   
    def getParent(self):
        return self.parent

    #get branch-factor        
    def getBF(self):
        return (len(self.treeInst))
    
    #create a new node. append it as a child to the self.treeInst and return a node 
    def createNode(self,tag):
         node = self._createChildByTag(etree.SubElement(self.treeInst,tag))
         return node
         
    #input:string-tagtype, create a new node with tag-type and add it to the node direct children
    #append the new child to the node children list
    #output - return the newNode
    def addNode(self,tag):
        node = self.createNode(tag)
        self.childList.append(node)        
        return node
    
            
    #input: parmeter and his value, add parm and set value or just set value  
    def setAttrib(self,parm,value):
        self.treeInst.attrib[parm] = str(value)
        
    #input: paramter name. output: return the value as a string or None   
    def getAttrib(self,parm):
        return self.treeInst.get(parm)
        
    #input: node, output: boolean if this node is monitore        
    def isMonitored (self):
       # return (self.treeInst.tag == "monitor")
       return (self.monitor == True)
    
    #input- tag, output if this node is this tag type- return True, else- False
    def boolWhoAmI (self, tag):
        return (self.treeInst.tag == tag)

    #return list of the node children
    def getChildren (self):
        #call _createChildList which create a wrap for the etree node chilren - 
        return self._createChildList()
                
        
    #create the wrap for child list and return a list    
    def _createChildList(self):
        if len(self.childList) !=0:
            return self.childList
        for element in list(self.treeInst):
            self.childList.append(self._createChildByTag(element))
        return self.childList
    
    
    #input: child num in the list , output: a new child node- not a deepcopy    
    def getChild(self,index):
        #return none if index given is bigger then the list length
        if index >= len(self.childList):
            return None
        else:
            #if child list is not empty - return pointer to the node
            if len(self.childList) > 0:
                return self.childList[index]
            else:
                #create child list and return the child at index
                self._createChildList()
                return self.childList[index]

    #input xml tree elem, create the node wrap    
    def _createChildByTag(self,elem):
        #return none if element given from etree is none.
        if elem == None:
            return None
        #create the new node according to type
        if elem.tag == "seq":
            from seqnode import SeqNode
            return SeqNode(elem,self.myTree,self)
        #tsk type child
        if elem.tag == "tsk":
            from tsknode import TskNode
            return TskNode(elem,self.myTree,self)
        #decorstor - L is for loop according to cogniteam code        
        if elem.tag == "dec":
            #createDecNodeFromName will append the right child to self
            return self._CreatDecoratorNodesFromName(elem)
        #loop child type
        if elem.tag == "loop":
            from loopnode import LoopNode
            return LoopNode(elem,self.myTree,self)
            #need to continue implementing the rest..
        if elem.tag == "not":
           from notnode import NotNode
           return NotNode(elem,self.myTree,self)
        #parallel child type    
        if elem.tag =="par": 
            from parallelnode import ParallelNode 
            return ParallelNode(elem,self.myTree,self)
        #selector child type
        if elem.tag =="sel": 
            from selectnode import SelectNode 
            return SelectNode(elem,self.myTree,self)
        #bool node
        if elem.tag == "bool":
            from boolean import BooleanNode
            return BooleanNode(elem,self.myTree,self)
                
                        
    #print the tree to xml- can be done from every node in the tree.       
    def treeToXml(self,fileName):
       #call treeToXml.
       self.myTree.treeToXml(fileName)
         
     #set monitor boolean property    
    def setMonitor(self,boolSet):
        self.monitor = boolSet
    
    #this func compare the node by there instance ID given by python func id.
    def comparTo(self,nodeToCompare):
        return id(self)==id(nodeToCompare)
    
    #this func remove the elem from the original xml tree. r
    def _removeSubElement(self,elem):
        #remove method compares elements based on identity, not on tag value or contents.
        self.treeInst.remove(elem)
        
    def __getitem__(self):
        return self

    
    #input - EtreeInst- element which it's tag is dec - decorator
    #output new node- loop/not with childen- example- for dec "!L!" crete not - loop - not      
    def _CreatDecoratorNodesFromName(self, element):
        name = element.get("name")
        #update Indentation
        ident = element.text
        identTail = element.tail
        newChild = None
        newEtreeInst = deepcopy(element)
        parent = element.getparent()
        lastChild = None
        #itertating over name char and creating node child as necessary
        for char in name:        
                #new child is the first child that replace decorator
                if newChild == None:
                        #if char is T/F create bool node
                        if char == "T" or char == "F":
                            from boolean import BooleanNode
                            return BooleanNode(element,self.myTree,self)
                        #if char is "L"- create loop node
                        if char == "L" :
                            #addNode func- create the node by tag and appand it to self.childList
                            newChild = self.createNode("loop")   
                        #if char is "!" - create not node
                        else:
                            if char == "!":
                                    newChild = self.createNode("not")
                        if newChild!= None:
                            newChild.treeInst.text = ident
                            newChild.treeInst.tail = identTail
                #after we create newChild we'll appand it all the other- by newChild.addNode func.
                else:
                    if lastChild == None:
                            #this is the only child of newChild
                            if char == "L" :
                                lastChild = newChild.addNode("loop")
                                
                            if char == "!":
                                lastChild = newChild.addNode("not")
                            if lastChild!= None:
                                #treeInst.text update- make the xml file readable with indentation
                                lastChild.treeInst.text = ident
                                #indentation of head and tail
                                lastChild.treeInst.tail = identTail
                    else:
                            if char == "L" :
                                lastChild = lastChild.addNode("loop")
                                
                            if char == "!":
                                lastChild = lastChild.addNode("not")  
                                
                            lastChild.treeInst.text = ident
                            lastChild.treeInst.tail = identTail
                #update Indentation
                ident += "\t"
               
        #if we succeded to create newChild and hid children we will give the last node all decorator attributes by deepcopy dec-treeInst                
        if lastChild !=None :
            lastChildParent =  lastChild.treeInst.getparent()
            #assigning the new tag for dec attributes not/loop- in element-etree
            if lastChild.treeInst.tag == "not":
                newEtreeInst.tag="not"
            if lastChild.treeInst.tag == "loop":
                newEtreeInst.tag="loop"
            #maintain the pointers with the etree and node tree to point the updated nodes.
            #remove lastChild.tree inst from his parent
            lastChildParent.remove(lastChild.treeInst)
            #give lastChild a new Tree inst- so he holds all the dec attributes from the xmltree
            lastChild.treeInst = newEtreeInst
            #append the treeInst back to his parent child list
            lastChildParent.append(lastChild.treeInst)
            
        #if we didn't create newChild any other children- exmple- <dec name ="L /dec>
        #we create only new child as loop- we'll give it decorator attributes.
        else:
            if newChild != None:
                #assigning the new tag for dec attributes not/loop- in element-etree
                if newChild.treeInst.tag == "not":
                    newEtreeInst.tag="not"
                if newChild.treeInst.tag == "loop":
                    newEtreeInst.tag="loop"
                #remove newChild.tree inst from his parent
                (parent).remove(newChild.treeInst)
                #give newChild a new Tree inst- so he holds all the dec attributes from the xmltree
                newChild.treeInst = newEtreeInst
                #append the treeInst back to his parent child list updated
                (parent).append(newChild.treeInst)
            
       #after reading it name and creating nodes as necessary we want to replace this subElement with the updated tree and update the xml tree(used to be decorator)
       #replace(self, old_element, new_element)
        parent.replace(element, newChild.treeInst)
        self._updateChildForDec(newChild , len(name))
        #return the newChild created.- return the root of the list/sub-tree
        return newChild
        
    #update the childs that we create for decorator property   
    def _updateChildForDec(self,newChild,size):      
      childToCheck = newChild
      #size- is the size of name string- the amount of childs we created.
      for i in range(size):
          #for each child we want to update his property
        	if childToCheck != None:
                 #update child debug
                  childToCheck._updateChildDebug()
                  #update distributions tables
                  childToCheck.distTableSucc = self._createDistTable("Successdistribution")
                  childToCheck.distTableFail = self._createDistTable("Failuredistribution")
                  #get the next child
                  childToCheck = childToCheck.getChild(0)
             
      
     #this func update the etree in order to print the new value in the xml file
    def _updateEtreeToPrintXmlFile(self,updateNode):
            if updateNode == None :
                return None
            #turn distribution table to a string that we know how to read from xml file
            if updateNode.distTableSucc != [] :
                updateNode.setAttrib("Successdistribution",updateNode._distTableToString(updateNode.distTableSucc))
            if updateNode.distTableFail != [] :  
                updateNode.setAttrib("Failuredistribution",updateNode._distTableToString(updateNode.distTableFail))
            #get child list
            childList = updateNode.getChildren()
            #iterate over child list with recursive call (list of lists)  
            if childList != None :
                for child in childList :
                    #update everyChild in the tree
                    self._updateEtreeToPrintXmlFile(child)                    
            #update Debug attributes in the xml file.        
            updateNode._updateDebugAttribToXmlFile()
            #update probability attributes in the xml file- to the etree
            updateNode._updateProbTableToXmlFile()
     
               
   #this func update the attribute in the xml file for debug - turn DEBUG- into a string and set etree attribute
    def _updateDebugAttribToXmlFile(self):
            if self.DEBUG != None:
                updateString =""
                if self.DEBUG[0]== True or self.DEBUG[0]=="True":
                    updateString+="True"+" "
                else :
                    updateString+="False"+" "
                updateString+= str(self.DEBUG[1])
                self.setAttrib("DEBUG",updateString) 
                
    
    # this func read attribute "DEBUG" from xml. and parse it by whiteSpace
    def _setDebugFromXmlFile(self):
        #get string from xml - "True 0.1" for example.	
          debug = self.getAttrib("DEBUG")
          if debug != None :
	      #return debug
              self.DEBUG =[]
              #print(debug)
              #parse the string by whiteSpace and returns a list
              debug = self._parseString(debug)
              #print(debug)
              #first element in the list should be boolen- success
              if debug[0]!=None and debug[0] == "True":
                  debug[0] = True
              else:
                 debug[0] = False   
             # second element in the list should be time - float number
              if debug[1]!=None:
                  debug[1]=float(debug[1])
              else :
                  debug = None
                  
              return debug  


    
        
    def _updateChildDebug(self):
        #iterate on all the element sub-tree which are from type-tree.Element
        for element in self.treeInst.iter(tag=etree.Element):
            if element.get("DEBUG") != None:
                self.DEBUGchild = True
                break
   
    #return true/false if the node has a debug child         
    def hasDebugChild(self):
        return self.DEBUGchild
    #append a new distribution to the succ table    
    def addDistToSuccTable(self, dist):
        self.distTableSucc.append(dist)
    #append a new distribution to the fail table
    def addDistToFailTable(self, dist):
        self.distTableFail.append(dist)
        
        
    #debug getter
    def getDebug(self):
        return self.DEBUG
        
        
    #get a table-distributions list and translate it back to string that we know how to read from xml file       
    def _distTableToString(self,table):
        if table == None:
            return None
        string =""
        #iterate all over the table len
        for index in range(0,len(table)) :
            #each dist has toString func- that we appand to string
            string += ((table[index]).toString())
            #we don't want whitSpace at the end of the string so we appand it only if we didn't reach the last index in the table            
            if index < (len(table)-1):
                string+=" "
        #return the table as string- for empty table we return empty string.
        return (string)        
        

    def getRandomProb(self, index):
        x = random.random()
        #print "getRandomProb", self.getAttrib("name"),self.getProbAtIndex(index)
        p = float(self.getProbAtIndex(index))
        if p==None:
            return None
        return (x <= p)
   
   #set prob table- set the probtable property- input- list of float
   #update the attribute in the etree
    def setProbTable(self, probtable):
        self.probTable = probtable
        self.setAttrib("probability",probtable)
        
    #set distribution success table with distTable- list of pointers to distributions.
    #update the attribute in the etree        
    def setDistTableSucc(self, distTable):
        self.distTableSucc = distTable
        self.setAttrib("Successdistribution",self._distTableToString(self.distTableSucc))
    
    #set distribution fail table with distTable- list of pointers to distributions.
    #update the attribute in the etree              
    def setDistTableFail(self, distTable):
        self.distTableFail = distTable
        self.setAttrib("Failuredistribution",self._distTableToString(self.distTableFail))    
      
     #update prob table at index with index and val given   
    def updateProbTableAtIndex(self, index, val, prob=0):
        if (self.probTable==None or len(self.probTable)==0 ):
            a = []
            #if the prob table is empty- create a probe table at the size of 2^parmetersInTheWorld
            for i in range(int(math.pow(2,node.parmetersInTheWorld))):
                a.append([0,0])
            #update probe table
            self.setProbTable(a)
        if prob!=0:
            self.probTable[index]=prob
        else:    
            if val:
                #if val is set to True- update another success . probTable[0]- count succ-numerator,probTable[1]-Counter of time tried-Denominator.
                #print(type(self.probTable[index]) , type(self.probTable[index][0]))
                self.probTable[index][0] = self.probTable[index][0]+1
                self.probTable[index][1] = self.probTable[index][1]+1
                #update the new probtable in etree
                self._updateProbTableToXmlFile()            
            else:
                #val is false
                # probTable[0]- count succ-numerator,probTable[1]-Counter of time tried-Denominator.
                self.probTable[index][1] = self.probTable[index][1]+1
                #update the new probtable in etree
                self._updateProbTableToXmlFile() 
            
    #update the etree in order to print the calculated value to xml file of probtable        
    def _updateProbTableToXmlFile(self):
        if (self.probTable==None or len(self.probTable)==0 ):
            return
        #turn probtable from list of flost to a string that we can read back from xml file
        probTableString = ""
        for index in range(len(self.probTable)) :
             probTableString+=str(self.getProbAtIndex(index))    
            #string concatenation- white space between the values
             probTableString +=' '
        #set probability attribute in etree
        self.setAttrib("probability",probTableString)
            
    def  updateProbTableAtIndexComputed(self,index,prob):
        self.probTable[index]=prob
        
    #getter for probIndex return flosat
    def getProbAtIndex(self,index):
        if self.probTable!=None and len(self.probTable) > index:
            #check if the index is a probability numer- float
            if (isinstance(self.probTable[index],float) or isinstance(self.probTable[index],int) or 
                (isinstance(self.probTable[index],str))):  
                return float(self.probTable[index])
            else:
                #this entry is a list of two parms- # probTable[0]- count succ-numerator,probTable[1]-Counter of time tried-Denominator.
                if float(self.probTable[index][1]) !=0 :   
                    #return the caculated value
                    return (float(self.probTable[index][0])/float(self.probTable[index][1]))
                return 0
        return None
           
     #set distributaion success table at index with time   
    def setDistTableSuccAtIndex(self, index, time, distTable=[],mapDistTable={}):
        if (self.distTableSucc==[]):
            a = []
            for i in range(int(math.pow(2,node.parmetersInTheWorld))):
                dist = Computed({})
                a.append(dist)
            self.setDistTableSucc(a)
       
        if  ((distTable==[]) and (mapDistTable=={})):   
            self.distTableSucc[index].setValueToTime(time, self.distTableSucc[index].getCountByTime(time)+1)
            #self.setAttrib("Successdistribution",self._distTableToString(self.distTableSucc))
        else:
            if  mapDistTable!={}:
                self.distTableSucc[index]=Computed(mapDistTable)
            else:    
                self.distTableSucc[index].setComputedDistTable(distTable)
        self.setAttrib("Successdistribution",self._distTableToString(self.distTableSucc))
    
            
       #set distributaion fail table  at index with time 
    def setDistTableFailAtIndex(self, index, time, distTable=[],mapDistTable={}):
        if (self.distTableFail==[]):
            a = []          
            for i in range(int(math.pow(2,node.parmetersInTheWorld))):
                dist = Computed({})
                a.append(dist)
            self.setDistTableFail(a)
        if  ((distTable==[]) and (mapDistTable=={})): 
            self.distTableFail[index].setValueToTime(time, self.distTableFail[index].getCountByTime(time)+1)
        else:
             if  mapDistTable!={}:
                self.distTableFail[index]=Computed(mapDistTable)
             else:   
                self.distTableFail[index].setComputedDistTable(distTable)    
        self.setAttrib("Failuredistribution",self._distTableToString(self.distTableFail))
        
        
        
        
    #node- run func 
    def run(self, index):
        a = None
        if (node.debugMode):
            tmpIndex  = index
            a = self.DEBUG
            if (a!=None):
                if not(self.boolWhoAmI("tsk")): 
                    if (self.monitor):
                        if a[0]:
                            self.setDistTableSuccAtIndex(tmpIndex, a[1])
                        else:
                            self.setDistTableFailAtIndex(tmpIndex, a[1])          
                        self.updateProbTableAtIndex(tmpIndex, a[0])               
        return a
    
   
        
   #set debug - recive a string exmp. "True 100"
    def setDebug(self, succtime):
        self.DEBUG = self._parseString(succtime)    
        self.DEBUG[1] = float(self.DEBUG[1])
        #self.setAttrib("DEBUG", self.DEBUG )
        self._updateDebugAttribToXmlFile()
        
    #run as base case func
    def runAsBaseCase (self, index):
        debug = node.run(self, index)
        if (debug!=None):
            return debug  
        a = [True, 0]
        randP = self.getRandomProb(index)   
        if randP==None:
            return None
        a[0]= randP             
        if a[0]:
            a[1] = float(self.getDistSuccByIndex(index).calcProb())
        else:          
            a[1] = float(self.getDistFailByIndex(index).calcProb())

        return a
        
    #get the distributions from distribution table success by index    
    def getDistSuccByIndex(self,index):
        if len(self.distTableSucc) > index:
            return self.distTableSucc[index]
        return None
        

    #get the distributions from distribution table fail by index 
    def getDistFailByIndex(self,index):
        if len(self.distTableFail) > index:
            return self.distTableFail[index]
        return None 
    
#    def getSuccDistAtIndex(self,index):
#        if self.distTableSucc != None and len(self.distTableSucc) > index :
#            return self.distTableSucc[index]
#            
#    def getFailDistAtIndex(self,index):
#        if self.distTableFail != None and len(self.distTableFail) > index :
#            return self.distTableFail[index]
            
            
   #clear the node property    
    def clear (self):
       self.probTable = []
       self.distTableSucc = []
       self.distTableFail = [] 
       try:
           del self.treeInst.attrib["Successdistribution"]
       except:
           pass
       try:
           del self.treeInst.attrib["probability"]
       except:
           pass
       try:
           del self.treeInst.attrib["Failuredistribution"]
       except:
           pass
#       self.setAttrib("Successdistribution",[])
#       self.setAttrib("probability",[])
#       self.setAttrib("Failuredistribution",[])  
       self.reset = True
       
      #clear whole tree property
    def clearWholeTree(self):
        childlist = self.getChildren()
        if childlist !=None:
            for child in childlist:
                child.clear()
                child.clearWholeTree()
        
     #run plan  
    def runPlan(self, index):
      children = self.getChildren()
      children[0].run(index) 
   
    def runPlanAccurate(self,index):
        children = self.getChildren()
        children[0].runAccurate(index) 
        
    def runPlanApproximate(self,index,e,T):
        children = self.getChildren()
        children[0].runApproximate(index,e,T)
        
    
    
    #get average to success time
    def getAverageSuccTime(self, index):
        if self.getDistSuccByIndex(index) != None:
            return self.getDistSuccByIndex(index).calcAverageTime()
        else:
            return float('Inf')

    def getSDSuccTime(self, index):
        if self.getDistSuccByIndex(index) != None:
            return self.getDistSuccByIndex(index).SDTime()
        else:
            return float(0)
            
#compute probability for less then T time            
    def getLessThenTProb(self,index,T):
        if self.getDistSuccByIndex(index) != None:
            #return round(self.getProbAtIndex(index)*self.getDistSuccByIndex(index).LessThenT(T),4)
            return self.getDistSuccByIndex(index).LessThenT(T)
        else:
            return float(0)
                
 #table is the name of the table needed- attribute
    def _createDistTable(self,table):
        string = self.getAttrib(str(table))
        
        table =[]        
        if string != None:
            table = self._parseString(string)

        newDistTable =[]
        #loop over the table- range (0,table len-1)- specifying the step value as 2
        if table != None:        
            for index in range(len(table)):
                #computed dist   
                if (table[index][0] == 'C'):
                    newDistTable.append(self._createComputedDist(table[index]))
                #normal dist
                if(str(table[index][0]) =='N'):
                    newDistTable.append(self._createNormalDist(table[index]))
                #discrete dist
                if(table[index][0] == 'D'):
                    pass
                #iniform dist- create new instance and 
                if(table[index][0] == 'U'):
                    x=self._createUniformDist(table[index])
                    newDistTable.append(x)

        return newDistTable
            
    #create computed distribution
    def _createComputedDist(self,Sinput):
        ans =self._getDictOfNumPairFromString(Sinput)
        return Computed(ans)        
    #create normal distribution
    def _createNormalDist(self,Sinput):
      ans = self._getTwoNumFromString(Sinput)
      return Normal(ans[0],ans[1])
       
    #create uniform distribution  
    def _createUniformDist(self,Sinput):
       ans = self._getTwoNumFromString(Sinput)
       return Uniform(ans[0],ans[1])
    
    def setDEBUGnode(self,sSucc=None,sTime=None):
       pass

    #input- string "num,num" output: tauple [num,num]
    # we use this func to divide two numbers for distribution parmeters value
    #can only work for two numbers in the string
    def _getTwoNumFromString(self,Sinput):
      stringNumA = ""
      stringNumB = ""
      nextNum = False
      
      #loop over the string
      for index in range(0, len(Sinput)):  
          #check if the Sinput[index] is a number or "." - for float num.
          if (Sinput[index].isdigit() or Sinput[index]=='.' ) == True and (nextNum == False):
              stringNumA += str( Sinput[index] )
              continue
          if(str(Sinput[index]) ==','):
              nextNum= True
              continue
          if (Sinput[index].isdigit() or Sinput[index]=='.') == True and (nextNum == True):
              stringNumB+= str(Sinput[index] ) 
              continue
              
      #return a list of two str that represent float numbers
      return [str(stringNumA),str(stringNumB)]
      
      
      
    # Sinput should look like this - C[123,123],[123,1231],[54,23] 
    #input- the string above, output: dictionary of key and value
    #we use this func to create the map/dictionary for computed distribution
    def _getDictOfNumPairFromString(self,Sinput):
        openBracket = False
        stringPair=""
        #start pairList as empty dictionary
        PairList = {}
        #iter from index=0 to strint- Sinput size
        for index in range(0,len(Sinput)):
            if Sinput[index] == '[' and openBracket == False :
                openBracket = True
                continue
            if Sinput[index] == ']' and openBracket == True:
                #call getTwoNumFromString func with stringPair and appand to the PairList- to get a tauple[num,num]
                pair = self._getTwoNumFromString(stringPair)
                PairList[str(pair[0])]= str(pair[1])
                #update open bracket to close                
                openBracket = False
                #init the stringPair
                stringPair = ""
                continue
            if openBracket == True :
                stringPair += Sinput[index]
                continue
        #return distionry  
        return PairList
    
        
    def getTime(self):
        if self.DEBUG !=None and len (self.DEBUG) > 1:
            return self.DEBUG[1]
            
            
            
    def getDistData(self, index, succ):
        if (succ==1):
            if self.getDistSuccByIndex(index) != None:
                return self.getDistSuccByIndex(index).getData()
            else:
                return None      
        else:
            if self.getDistFailByIndex(index) != None:
                return self.getDistFailByIndex(index).getData()
            else:
                return None  
                
                
    def isRoot(self):
        return (self.getParent().getParent()== None)
        
            
                
    def setBounds(self,index,ubound,lbound):
        #self.upperBound[index] = round(self.getProbAtIndex(index)*ubound,4)
        #self.lowerBound[index] = round(self.getProbAtIndex(index)*lbound,4)
        self.upperBound[index] = ubound
        self.lowerBound[index] = lbound
         
    def getUBound(self,index):
        return round(self.upperBound[index],4)
        
    def getLBound(self,index):
        return round(self.lowerBound[index],4)   
           