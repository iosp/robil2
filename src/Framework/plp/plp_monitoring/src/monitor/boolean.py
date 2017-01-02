# -*- coding: utf-8 -*-
"""
NotNode class inherits from node class.
type of node- not
has it own run function
"""


from Node import node

class BooleanNode (node):
    def __init__(self,treeInst,mytree,parent):
        #call to super constracture
        node.__init__(self,treeInst,mytree,"bool",parent)
        treeInst.tag = "bool"
        if treeInst.get("name") == "T":
            self.boolP = "True"
        else:
            self.boolP = "False"
            

    #run-bool
    def run (self, index):
        
        runResult = [self.boolP , 0]
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
            runResult[1] = debug[1]
            return runResult             
        
        a = [self.boolP, 0]
#        index = 1
        child = self.getChildren()
        b = child[0].run(index)
        
        a[1] = b[1]

            
        if (self.monitor):    
            if a[0]:
                self.setDistTableSuccAtIndex(tmpIndex, a[1])
            else:
                self.setDistTableFailAtIndex(tmpIndex, a[1])    
            self.updateProbTableAtIndex(tmpIndex, a[0])
        return a    
