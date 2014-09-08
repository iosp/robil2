# -*- coding: utf-8 -*-
"""
ParallelNode class inherits from node class.
type of node- parallel
has it own run function
"""

from Node import node
import SumRandomVariables


class ParallelNode (node):
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"par",parent)
    
    #run-parallel
    def run(self, index):
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
        
	#Changed to the following logic: Parallel runs untils one of its children finishes, and returns that child's result. (RAZ)
        a = [True, float("inf")]       
        for i in self.getChildren():
            b = i.run(index)
            if b[1]<a[1]:
                a[0]=b[0]  #was not indent
                a[1]=b[1]  #was not indent
            
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
        for child in self.getChildren():
            [prob,distS,distF] = child.runAccurate(index)
            distMatrixSucc.append(distS.toMatrix())
            distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*(1-prob)
        distSucc= SumRandomVariables.MaxAccurateDescrete(distMatrixSucc)
        distFail= SumRandomVariables.MaxAccurateDescrete(distMatrixFail)
        self.updateProbTableAtIndex(index, 0, round(1-probTotal,4))
        self.setDistTableSuccAtIndex(index,0,[],distSucc)
        self.setDistTableFailAtIndex(index,0,[],distFail)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]
        
        
        
        
    def runApproximate(self, index,e,T):
        distMatrixSucc = []
        distMatrixFail = []
        probTotal = 1
        for child in self.getChildren():
            [prob,distS,distF] = child.runApproximate(index,e,T)
            distMatrixSucc.append(distS.toMatrix())
            distMatrixFail.append(distF.toMatrix())
            probTotal=probTotal*(1-prob)
        probTotal = 1-probTotal
        distSucc= SumRandomVariables.MaxAccurateDescrete(distMatrixSucc)
        distFail= SumRandomVariables.MaxAccurateDescrete(distMatrixFail)
        self.updateProbTableAtIndex(index, 0, round(probTotal,4))
        self.setDistTableSuccAtIndex(index,0,[],distSucc)
        self.setDistTableFailAtIndex(index,0,[],distFail)
        return [self.getProbAtIndex(index),self.getDistSuccByIndex(index),self.getDistFailByIndex(index)]    