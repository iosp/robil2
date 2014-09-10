# -*- coding: utf-8 -*-
"""
SeqNode class inherits from node class.
type of node- seq
has it own run function
"""
from Node import node
import SumRandomVariables
from distributions.computed import Computed
from distributions.normal import Normal
from distributions.uniform import Uniform

class SeqNode (node):
    
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"seq",parent)
        
        
    
    #seq- run
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
        
        a = [True, 0]        
        for i in self.getChildren():                     
            b = i.run(index)  
            a[0] = a[0] and b[0]
            a[1] = a[1] + b[1]
            if not b[0]:	  
                break
            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])    
            self.updateProbTableAtIndex(tmpIndex, a[0])
        return a    
        
#----------------------------------------------------------------------------------------

    def runAccurate(self, index):
        distMatrixSucc = []
        distMatrixFail = []
        probTotal = 1
        probArr = []
        for child in self.getChildren():
            [prob,distS,distF] = child.runAccurate(index)
            if isinstance(self.getDistSuccByIndex(index), Computed):
                distMatrixSucc.append(distS.toMatrix())
            if isinstance(self.getDistFailByIndex(index), Computed):    
                distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*prob
            probArr.append(1-prob)
        distScc = SumRandomVariables.SumAccurateDescrete(distMatrixSucc)
        distFail = SumRandomVariables.OrSumAccurateDescrete(distMatrixFail,distMatrixSucc,probArr)
        self.updateProbTableAtIndex(index, 0, round(probTotal,4))
        self.setDistTableSuccAtIndex(index,0,distScc)
        self.setDistTableFailAtIndex(index,0,distFail)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]

            
            
            
    def runApproximate(self, index,e,T):
        distMatrixSucc = []
        distMatrixFail = []
        probTotal = 1
        probArr = []
        numChildren = 0
        for child in self.getChildren():
            [prob,distS,distF] = child.runAccurate(index)
            distMatrixSucc.append(distS.toMatrix())
            distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*prob
            probArr.append(1-prob)
            numChildren+=1
        distScc = SumRandomVariables.SumApproximateDescrete(distMatrixSucc,e,numChildren)
        distFail = SumRandomVariables.OrSumApproximateDescrete(distMatrixFail,distMatrixSucc,probArr,e,numChildren)
        self.updateProbTableAtIndex(index, 0, round(probTotal,4))
        self.setDistTableSuccAtIndex(index,0,distScc)
        self.setDistTableFailAtIndex(index,0,distFail)
        if self.isRoot():
            u= SumRandomVariables.computeUpperBound(T,e,numChildren,distMatrixSucc)
            l= SumRandomVariables.computeLowerBound(T,e,numChildren,distMatrixSucc)
            self.setBounds(index,u,l)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]


        
