# -*- coding: utf-8 -*-
"""
SelectNode class inherits from node class.
type of node- sel
has it own run function
"""

from Node import node
import SumRandomVariables

class SelectNode (node):
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"sel",parent)
    
    #run-select
    def run (self, index):       
        tmpIndex = index
        
        if (node.debugMode):
            if not(self.hasDebugChild()):
                res = self.runAsBaseCase(index)
                if res!=None:
                    return res
            else:
                if not(self.reset):
                    self.clear()

        debug = node.run(self, index)
        if (debug!=None):
            return debug          
       
        a = [False, 0]
        for i in self.getChildren():                        
            b = i.run(index)           
            a[0] = a[0] or b[0]
            a[1] = a[1] + b[1]
            if b[0]:	  
                break
            

            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])       
            self.updateProbTableAtIndex(tmpIndex, a[0]) 
        return a    
        
        
        
        
        
        
        
    def runAccurate(self, index):
        distMatrixSucc = []
        distMatrixFail = []
        probTotal = 1
        probArr = []
        for child in self.getChildren():
            [prob,distS,distF] = child.runAccurate(index)
            distMatrixSucc.append(distS.toMatrix())
            distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*(1-prob)
            probArr.append(prob)
            
        distScc = SumRandomVariables.OrSumAccurateDescrete(distMatrixSucc,distMatrixFail,probArr)
        distFail = SumRandomVariables.SumAccurateDescrete(distMatrixFail)
        self.updateProbTableAtIndex(index, 0, round(1-probTotal,4))
        self.setDistTableSuccAtIndex(index,0,distScc)
        self.setDistTableFailAtIndex(index,0,distFail)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]

        
        
        
    def runApproximate(self, index,e,T):
        distMatrixSucc = []
        distMatrixFail = []
        probTotal = 1
        probArr = []
        numChildren = 0
        print self.isRoot()
        for child in self.getChildren():
            print child.isRoot()
            [prob,distS,distF] = child.runAccurate(index)
            distMatrixSucc.append(distS.toMatrix())
            distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*(1-prob)
            probArr.append(prob)
            numChildren+=1
        distScc = SumRandomVariables.OrSumAccurateDescrete(distMatrixSucc,distMatrixFail,probArr)
        distFail = SumRandomVariables.SumAccurateDescrete(distMatrixFail)
        self.updateProbTableAtIndex(index, 0, round(1-probTotal,4))
        self.setDistTableSuccAtIndex(index,0,distScc)
        self.setDistTableFailAtIndex(index,0,distFail)
        if self.isRoot():
            u= SumRandomVariables.OrComputeUpperBound(distMatrixSucc,distMatrixFail,probArr,T,e,numChildren)
            l= SumRandomVariables.OrComputeLowerBound(distMatrixSucc,distMatrixFail,probArr,T,e,numChildren,)
            self.setBounds(index,u,l)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]
    
    
            
