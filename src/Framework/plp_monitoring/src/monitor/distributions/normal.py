# -*- coding: utf-8 -*-
"""

@author: polak
"""
import random
from distribution import Distribution

class Normal(Distribution):
    #constractur
    def __init__(self, parmM,parmG):
        Distribution.__init__(self)
        self.parmM = parmM
        self.parmG = parmG
        
        
    def calcProb(self):
        return random.normalvariate(float(self.parmM), float(self.parmG))
    
    #for debaug     
    def whoAmI(self):
        return "Normal"
        
    def printMe (self):
        print self.parmM ,self.parmG 
        
    def toString(self):
        return str("N"+"["+str(self.parmM)+","+str(self.parmG)+"]")    