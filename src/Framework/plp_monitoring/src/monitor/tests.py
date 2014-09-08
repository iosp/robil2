# -*- coding: utf-8 -*-

from tree import xmlTree
from Node import node
import time
import sys
from GUI import *

#create a tree from scratch
def test1():
    
    tree = node()
    root = tree
    #first child
    newChild = root.addNode("par")#parallel
    #notice that the childlist start from zero.
    child = root.getChild(0)
    #compare by instance id:
    if newChild.comparTo(child) == True :
        print("test 1: success!")
    else:
        print("test 1: failed :-(")

#provide a tree xml file      
def test2():
    tree = xmlTree("tests/test2.xml")
    root = tree.getRoot()
    # root is always a plan    
    newNode = root.getChild(0)
    ChildList = newNode.getChildren()
    #iterate over first node children:
    count = 0
    for childNode in ChildList:
        count += 1
    if count == 5:
        print("test 2: success!")
    else:
        print("test 2: failed :-(")
    
    
#this test check the node-type contratur and thir oo
def test3():
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("seq")
    if firstChild == None:
        print ("error creating seq node")
        print("test 3: failed :-(")
        return None
        
    tempN = firstChild.addNode("seq")
    if tempN == None:
        print ("error creating seq node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
        
    tempN = firstChild.addNode("seq")
    if tempN == None:
        print ("error creating seq node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
        
    tempN = firstChild.addNode("loop")
    if tempN == None:
        print ("error creating loop node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
        
    tempN = firstChild.addNode("par")
    if tempN == None:
         print ("error creating parallel node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
        
    tempN = firstChild.addNode("tsk")
    if tempN == None:
        print ("error creating tsk node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
        
    tempN = firstChild.addNode("sel")
    if tempN == None:
        print ("error creating selector node")
    else:
        tempN.setAttrib("probability","0.1 0.5")
    
    #iterate over firstChild children:
    firstChildList = firstChild.getChildren()
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 6:
        print("test 3: success! please check the file test3.xml - every tag need to have the same attrib.")
    else:
        print("test 3: failed :-(")
    
    #print the tree we built from scratch to xml file.
    #please check the file- every tag need to have the same attrib.
    root.treeToXml("tests/test3.xml")
    

#please run test 3 before test 4:    
def test4():
    tree = xmlTree("tests/test3.xml")
    #remember- root is alwayes type/tag- plan
    root = tree.getRoot()
    ans = []
    ans.append(root.boolWhoAmI("plan"))
    firstChild = root.getChild(0)
    ans.append(firstChild.boolWhoAmI("seq"))
    ans.append((firstChild.getChild(0)).boolWhoAmI("seq"))
    ans.append((firstChild.getChild(1)).boolWhoAmI("seq"))
    ans.append((firstChild.getChild(2)).boolWhoAmI("loop"))
    ans.append((firstChild.getChild(3)).boolWhoAmI("par"))
    ans.append((firstChild.getChild(4)).boolWhoAmI("tsk"))
    ans.append((firstChild.getChild(5)).boolWhoAmI("sel"))
    
    for index in range(0,7):
        if ans[index] == False:
            print("test 4: failed :-(")
            
    print("test 4: success!")
 
           
#please run test 3 before test 5: - check attrib func/method    
def test5():
    tree = xmlTree("tests/test3.xml")
    #remember- root is alwayes type/tag- plan
    root = tree.getRoot()
    ans = []
    ans.append(root.getAttrib("probability"))
    firstChild = root.getChild(0)
    ans.append(firstChild.getAttrib("probability"))
    ans.append((firstChild.getChild(0)).getAttrib("probability"))
    ans.append((firstChild.getChild(1)).getAttrib("probability"))
    ans.append((firstChild.getChild(2)).getAttrib("probability"))
    ans.append((firstChild.getChild(3)).getAttrib("probability"))
    ans.append((firstChild.getChild(4)).getAttrib("probability"))
    ans.append((firstChild.getChild(5)).getAttrib("probability"))
    
    #ans 1+2 dosn't have attribut- None
    if ans[0] !=None or ans[1] != None :
        print("test 5: failed :-(")
        
    for index in range(2,7):
        if ans[index] != "0.1 0.5":
            print("test 5: failed :-(")
            print (index)
            
    print("test 5: success!")

    
 #check the monitor set/get func/method   
def test6():
    tree = xmlTree("tests/test2.xml")
    root = tree.getRoot()
    firstChild = root.getChild(0)
    childList = firstChild.getChildren()
    for childNode in childList:
        childNode.setMonitor(False)
        
    for childNode in childList:
        boolVal = childNode.isMonitored()
        if boolVal != False :
            print("test 6: failed :-(")
            return None
    
    print("test 6: success!")
     
 #empty test - will be implemented- feeling creative? :-)    
def test7():
  
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating seq node")
        print("test 7: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    
    for j in range(3): 
      tempN = firstChild.addNode("seq")
      if tempN == None:
	  print ("error creating seq node")
	  
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setAttrib("time","1")
                          tempN2.setAttrib("succ","T")
                          #tempN2.setTime(0)
                          #tempN2.setSucc(False)
                          tempN2.setProbTable([0.8, 0.5])
                          for i in range(2):
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2.distTableSucc)
                          tempN2.setAttrib("Failuredistribution",tempN2.distTableFail)    
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setAttrib("time","1")
                  tempN1.setAttrib("succ","T")
                  #tempN1.setTime(0)
                  #tempN1.setSucc(False)
                  tempN1.setProbTable([0.7, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1.distTableSucc)
                  tempN1.setAttrib("Failuredistribution",tempN1.distTableFail) 
             
	    
        

    
    #iterate over firstChild children: 
    firstChildList = firstChild.getChildren()
    for i in range(5):
        firstChild.run(0)
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 3:
        print("test 7: success! please check the file output/test4.xml - every tag need to have the same attrib.")
    else:
        print("test 7: failed :-(")
    
    #print the tree we built from scratch to xml file.
    #please check the file- every tag need to have the same attrib.
    root.treeToXml("output/test4.xml")
    
    
def test8():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 8.1: success!")

   else:
        ("test 8.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 8.2: success!")
   else:
        ("test 8.2: failed :-( - check computed dist")
        
        
        
#this test read test9.xml and create distributaion as needed for tskNode 
def test9():
   tree = xmlTree("tests/test9.xml")
   #root it node type plan
   root = tree.getRoot()
   
   #this child is type- seq
   child = root.getChild(0)
   #this child is type- tsk
   tskChild = child.getChild(0)
   #get dist from the distTable
   distC = tskChild.getSuccDistAtIndex(2)
   distU = tskChild.getSuccDistAtIndex(1)
   distN = tskChild.getSuccDistAtIndex(0)
   

       
   if( distC.whoAmI() == "Computed" and float(distC.getCountByTime(0.1)) == 5 and float(distC.getCountByTime(257)) == 977):
       print ("test 9.1: success!")
   else:
       ("test 9: failed :-( - check computed dist")  

   if( distU.whoAmI() == "Uniform" and float(distU.parmA) == 0 and float(distU.parmB) == 5 ):
       print ("test 9.2: success!")
   else:
       ("test 9.2: failed :-( - check uniform dist")  


   if ( distN.whoAmI() == "Normal"):
       print ("test 9.3: success!")
   else:
       ("test 9.3: failed :-( - check normal dist")      
       
       
       
def test10():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1) 
   dist_fail1 = _createUniformDist(5, 8)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 10.1: success!")

   else:
        ("test 10.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   dist.printMe()
   print dist.calcProb()
   print "-----------"
   dist_succ.printMe()
   print dist_succ.calcProb()
   dist_fail1.printMe()
   print dist_fail1.calcProb()
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 10.2: success!")
   else:
        ("test 10.2: failed :-( - check computed dist")
        
#def test11():
#    #in AdiEvent2- I removed x,y and id attribue. so we can see easily the decorator not,loop (L!)
#    tree = xmlTree("AdiEvent2.xml")
#    root= tree.getRoot()
#    seqChild = root.getChild(0)
#    if seqChild == None:
#        print("test 11: failed :-( ")
#    else:
#        #check debug reading from the file
#        #seqChild. getDEBUGtime() = 5
#        if seqChild.getDEBUGsucc() == True :#and seqChild.DEBUG[1] == 5 :
#            print("test 11.1: success")
#        else:
#            print ("test 11.1: failed :-( ")
#        seqChild.setDEBUGresult("True 100")
#        if seqChild.getDEBUGsucc() == True :#and seqChild.DEBUG[1] == 5 :
#            print("test 11.2: success")
#        else:
#            print ("test 11.2: failed :-( ")            

def test12():
    tree = xmlTree("tests/event1.xml")
    tree.treeToXml("output/test12.xml")
    print("test 12: success- check output/test12.xml file")      



def test14():
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating seq node")
        print("test 14: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      if j==0:  
          tempN = firstChild.addNode("seq")
          if tempN == None:
              print ("error creating seq node")
      if j==1:  
          tempN = firstChild.addNode("sel")
          if tempN == None:
              print ("error creating seq node")
      if j==2:  
          tempN = firstChild.addNode("loop")
          if tempN == None:
              print ("error creating seq node")        
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.1, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug("True 100")
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.3, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
          if j==2:
              break
	    
        

    
    #iterate over firstChild children: 

    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test14a.xml") 
    print("test 14.1: success! please check the file test14a.xml - every tag need to have the same attrib.")
    print "phase 2"
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test14b.xml") 
    print("test 14.2: success! please check the file test14b.xml - every tag need to have the same attrib.")
        
   
     
 #empty test - will be implemented- feeling creative? :-)    
def test15():
  
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating seq node")
        print("test 15: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      tempN = firstChild.addNode("seq")
      if tempN == None:
	  print ("error creating seq node")
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.8, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug("True 100")
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.7, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
             
	    
        

    
    #iterate over firstChild children: 

    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test15a.xml") 
    print("test 15.1: success! please check the file test15a.xml - every tag need to have the same attrib.")
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test15b.xml") 
    print("test 15.2: success! please check the file test15b.xml - every tag need to have the same attrib.")
        
        
    
    #print the tree we built from scratch to xml file.
    #please check the file- every tag need to have the same attrib.

    
 
 #check the monitor set/get func/method   
def test16():
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating par node")
        print("test 16: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      if j==0:  
          tempN = firstChild.addNode("seq")
          if tempN == None:
              print ("error creating seq node")
      if j==1:  
          tempN = firstChild.addNode("sel")
          if tempN == None:
              print ("error creating sel node")
      if j==2:  
          tempN = firstChild.addNode("loop")
          if tempN == None:
              print ("error creating seq node")        
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.1, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug("True 100")
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.3, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
          if j==2:
              break
	    
        

    
    #iterate over firstChild children: 
    firstChildList = firstChild.getChildren()
    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test16a.xml") 
    
    print "phase 2"
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test16b.xml") 
    
        
        
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 3:
        print("test 16: success! please check the file test4.xml - every tag need to have the same attrib.")
    else:
        print("test 16: failed :-(")
    
     
 #empty test - will be implemented- feeling creative? :-)    
def test17():
  
    tree = node()
    root = tree
    #first child
    firstChild = root.addNode("par")
    if firstChild == None:
        print ("error creating parallel node")
        print("test 17: failed :-(")
        return None
    dist_succ = _createUniformDist(2,5)
    dist_fail = _createUniformDist(6,10)   
    
    firstChild.DEBUGchild = True 
    for j in range(3): 
      tempN = firstChild.addNode("seq")
      if tempN == None:
	  print ("error creating seq node")
      
      for i in range(5):
          if ((j==1) and (i==2)):
              tempN1 = tempN.addNode("seq")
              tempN.DEBUGchild = True
              tempN1.DEBUGchild = True
              if tempN1 == None:
                  print ("error creating seq node")
              else:                
                  for i in range(4):
                      tempN2 = tempN1.addNode("tsk")
                      if tempN2 == None:
                          print ("error creating seq node")
                      else:
                          tempN2.setProbTable([0.8, 0.5])
                          for i in range(2):
                              dist_fail = _createUniformDist(6,10-i)  
                              tempN2.addDistToSuccTable(dist_succ)
                              tempN2.addDistToFailTable(dist_fail)
                          tempN2.setAttrib("Successdistribution",tempN2._distTableToString(tempN2.distTableSucc))
                          tempN2.setAttrib("Failuredistribution",tempN2._distTableToString(tempN2.distTableFail))
                          tempN2.setDebug("True 100")
                              
          else:
              tempN1 = tempN.addNode("tsk")
              if tempN1 == None:
                  print ("error creating seq node")
              else:
                  tempN1.setProbTable([0.7, 0.5])
                  for i in range(2):
                      tempN1.addDistToSuccTable(dist_succ)
                      tempN1.addDistToFailTable(dist_fail)
                      
                  tempN1.setAttrib("Successdistribution",tempN1._distTableToString(tempN1.distTableSucc))
                  tempN1.setAttrib("Failuredistribution",tempN1._distTableToString(tempN1.distTableFail))
             
	    
        

    
    #iterate over firstChild children: 
    firstChildList = firstChild.getChildren()
    node.debugMode = False
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test17.xml") 
    
    print "phase 2"
    node.debugMode = True
    for i in range(5):
        firstChild.run(0)
    root.treeToXml("output/test17.xml") 
    
        
        
    count = 0
    for childNode in firstChildList:
        count += 1
    if count == 3:
        print("test 17: success! please check the file test4.xml - every tag need to have the same attrib.")
    else:
        print("test 17: failed :-(")
    
    #print the tree we built from scratch to xml file.
    #please check the file- every tag need to have the same attrib.
#    root.treeToXml("test4.xml")
    
    
def test18():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 18.1: success!")

   else:
        ("test 18.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 18.2: success!")
   else:
        ("test 18.2: failed :-( - check computed dist")
        
        
        
#this test read test9.xml and create distributaion as needed for tskNode 
def test19():
   tree = xmlTree("tests/test9.xml")
   #root it node type plan
   root = tree.getRoot()
   
   #this child is type- seq
   child = root.getChild(0)
   #this child is type- tsk
   tskChild = child.getChild(0)
   #get dist from the distTable
   distC = tskChild.getSuccDistAtIndex(2)
   distU = tskChild.getSuccDistAtIndex(1)
   distN = tskChild.getSuccDistAtIndex(0)
   

       
   if( distC.whoAmI() == "Computed" and float(distC.getCountByTime(0.1)) == 5 and float(distC.getCountByTime(257)) == 977):
       print ("test 19.1: success!")
   else:
       ("test 19: failed :-( - check computed dist")  

   if( distU.whoAmI() == "Uniform" and float(distU.parmA) == 0 and float(distU.parmB) == 5 ):
       print ("test 19.2: success!")
   else:
       ("test 19.2: failed :-( - check uniform dist")  


   if ( distN.whoAmI() == "Normal"):
       print ("test 19.3: success!")
   else:
       ("test 19.3: failed :-( - check normal dist")      
       
       
       
def test20():
   tree = xmlTree("tests/test3.xml")
   root = tree.getRoot()
   
   #this child is type- tsk
   child = root.getChild(0)
   
   ### create a new dist - and 
   dist_succ = _createNormalDist(5,2)
   dist_fail = _createNormalDist(4,1) 
   dist_fail1 = _createUniformDist(5, 8)   
   #add to succ table
   child.addDistToSuccTable(dist_succ)
   #add to fail table/
   child.addDistToFailTable(dist_fail)
   #get distribute from the node by it's index (p1,p2,p3..)
   dist_get_succ = child.getSuccDistAtIndex(0)
   dist_get_fail = child.getFailDistAtIndex(0)
   #check that it has the same parms 
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.   
   if (dist_get_succ != None and dist_get_succ.parmM == float(5) and dist_get_succ.parmG == float(2) and dist_get_fail != None and dist_get_fail.parmM == float(4) and dist_get_fail.parmG == float(1)):
       print ("test 20.1: success!")

   else:
        ("test 20.1: failed :-(")
        
    # try to create computed dist.
   #added by RAZ -- Adi, I made the tests a bit more complex, you should always have the tests as hard a possible, checking all possible cases.
   dist = _createComputedDist()
   dist.setValueToTime(0.1,1)
   dist.setValueToTime(0.1, dist.getCountByTime(0.1)+1 )
   dist.setValueToTime(0.2,1)   
   dist.setValueToTime(0.05,1)
   dist.printMe()
   print dist.calcProb()
   print "-----------"
   dist_succ.printMe()
   print dist_succ.calcProb()
   dist_fail1.printMe()
   print dist_fail1.calcProb()
   if (dist.getCountByTime(0.1) == 2 and dist.getCountByTime(0.2) == 1 and dist.getCountByTime(0.05) == 1):
       print ("test 20.2: success!")
   else:
        ("test 20.2: failed :-( - check computed dist")
          

#provide a tree xml file      
def test21():
    
    start = time.time()
    tree = xmlTree("tests/event1.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1
    root.treeToXml("output/testE211.xml")  
    print("test 21.1: success!, testE211.xml")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0)  
    for i in range(100):
        root.runPlan(1)    
    root.treeToXml("output/testE212.xml") 
    print("test 21.2: success!, testE212.xml")
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
    root.treeToXml("output/testE213.xml") 
    print("test 21.3: success!, testE213.xml")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    
    
def test22():
    start = time.time()
    tree = xmlTree("tests/event2.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/testE221.xml")  
    print("test 22.1: success!")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0) 
    for i in range(100):
        root.runPlan(1)
    root.treeToXml("output/testE222.xml") 
    print("test 22.2: success!")
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
    root.treeToXml("output/testE223.xml") 
    print("test 22.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    
    
def test23():
    start = time.time()
    tree = xmlTree("tests/event3.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/testE231.xml")  
    print("test 23.1: success!")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0)
    for i in range(100):
        root.runPlan(1)
    root.treeToXml("output/testE232.xml") 
    print("test 23.2: success!")
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
    root.treeToXml("output/testE233.xml") 
    print("test 23.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed

def test24():
    print "-------TEST 24-------"
    start = time.time()
    tree = xmlTree("tests/small_test.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/small_test_before_run.xml")  
    print("test 4.1: success!")
 
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)    
    for i in range(1000):
        root.runPlan(1)
    root.treeToXml("output/small_test_no_debug.xml") 
    print("test 4.2: success!")
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
    root.treeToXml("output/small_test_debug_mode.xml") 
    print("test 4.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    print "-----------------------"

#run offline monter-carlo and read from outputted file for debug mode.
def test25():
    print "-------TEST 25-------"
    start = time.time()
    tree = xmlTree("tests/small_test.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1
    
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)    
    for i in range(1000):
        root.runPlan(1)
    root.treeToXml("output/small_test_after_offline.xml") 
    print "Finished gathering offline statistics."
    print "-------Debug mode-------"
    node.debugMode = True    
    tree = xmlTree("output/small_test_after_offline.xml")
    root = tree.getRoot()
    for i in range(100):
        root.runPlan(0)    
    for i in range(100):
        root.runPlan(1)   
    root.treeToXml("output/small_test_debug_mode.xml") 
    print("test 4.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    print "-----------------------"
    

def test26():
    print "-------TEST 26-------"
    start = time.time()
    tree = xmlTree("tests/small_test_no_tsk_attrib.xml", None, "tests/small_test_tsk_attrib.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1
    
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)
    for i in range(1000):
        root.runPlan(1)
    root.treeToXml("output/small_test_after_offline_tsk.xml")
    print "Finished gathering offline statistics."
    print "-------Debug mode-------"
      
    #tree = xmlTree("output/small_test_after_offline_tsk.xml")
    node.debugMode = True
    root = tree.getRoot()
    for i in range(1000):
        root.runPlan(0)
    for i in range(1000):
        root.runPlan(1)
    root.treeToXml("output/small_test_debug_mode_tsk.xml")
    print("test 26: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    print "-----------------------"    
    
    
    
def test27():
    
    start = time.time()
    tree = xmlTree("tests/event1_no_tsk_attrib.xml",None ,"tests/event1_tsk_attrib.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1
    root.treeToXml("output/testE271.xml")  
    print("test 27.1: success!, testE271.xml")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0)  
    for i in range(100):
        root.runPlan(1)    
    root.treeToXml("output/testE272.xml") 
    print("test 21.2: success!, testE272.xml")
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
    root.treeToXml("output/testE273.xml") 
    print("test 27.3: success!, testE273.xml")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    
    
def test28():
    start = time.time()
    tree = xmlTree("tests/event2_no_tsk_attrib.xml",None,"tests/event2_tsk_attrib.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/testE281.xml")  
    print("test 28.1: success!")
 
    node.debugMode = False
    for i in range(100):
        root.runPlan(0) 
    for i in range(100):
        root.runPlan(1)
    root.treeToXml("output/testE282.xml") 
    print("test 28.2: success!")
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
    root.treeToXml("output/testE283.xml") 
    print("test 28.3: success!")
    print "Success probability in debug mode: Clear sky = %f, Cloudy = %f" %(root.getChild(0).getProbAtIndex(0),root.getChild(0).getProbAtIndex(1))
    print "Average success time in debug mode with clear sky = %f" %(root.getChild(0).getAverageSuccTime(0))
    print "Average success time in debug mode when Cloudy = %f" %(root.getChild(0).getAverageSuccTime(1))
    elapsed = (time.time() - start)
    elapsed = (time.time() - start)
    print "Time: %f" %elapsed
    
    
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
    tree = xmlTree("tests/skill4.xml",None,"tests/skill4_tsk_attribs.xml")
    root = tree.getRoot()
    node.parmetersInTheWorld = 1

    root.treeToXml("output/skill4.xml") 
    print("test 32.1: success!")
    node.debugMode = False
    for i in range(1000):
        root.runPlan(0)
    tree.createWrapperTreeMap("id")    
#    monitorID = "param=9b82d340-6893-4e68-a676-4d1658aae8d0"
#    print monitorID.split("=")[1]    
    monitordNode=tree.getWrappedNode("ae9c53ae-b42c-4f21-ba6a-7b1fa8741c2d")
    if monitordNode:
        print "Success probability in offline mode monitor: Mission: %f" % monitordNode.getProbAtIndex(0)
        print "Average success time monitor: Mission= %f" % monitordNode.getAverageSuccTime(0)
        print "SD success time monitor: Mission= %f" % monitordNode.getSDSuccTime(0)
    monitordNode=tree.getWrappedNode("b9e18714-4869-422c-bc38-cb2dca88c530")
    if monitordNode:
        print "Success probability in offline mode monitor: ExitFromCar: %f" % monitordNode.getProbAtIndex(0)
        print "Average success time monitor: ExitFromCar= %f" % monitordNode.getAverageSuccTime(0)
        print "SD success time monitor: Mission= %f" % monitordNode.getSDSuccTime(0)
    root.treeToXml("output/skill4_after_run.xml") 
    print("test 32.2: success!")
    print "Success probability in offline mode - root: %f" % root.getChild(0).getProbAtIndex(0)
    print "Average success time - root= %f" % root.getChild(0).getAverageSuccTime(0)

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
    #run the 10 tests
    test32()
#    
#    if len(sys.argv) == 2 and sys.argv[1] == "all":
#	test1()
#	test2()
#	test3()
#	test4()
#	test5()
#	test6()
#	test7()
#	test8()
#	test9()
#	test10()
##    test11()
#	test12()
#	test14()
#	test15()
#	test16()
#	test17()
#	test18()
#	test19()
#	test20()
#	test21()
#	test22()
#	test23()
#	test24()
#	test25()
#	test26()
# 	test27()
#  	test28()
#	test29()
#	test31()
#      
#    elif len(sys.argv) == 2 and sys.argv[1] == "events":
#	test21()
#	test22()
#	test23()
#	test27()
#	test28()
#	test29()
#    elif len(sys.argv) == 2 and sys.argv[1] == "demo":
#	test24()
#	test25()
#    elif len(sys.argv) == 2 and sys.argv[1] == "integration":
#	test30()
#    else:
#	print "please provide one of the following command line arguments: [all,events,demo]"
     
