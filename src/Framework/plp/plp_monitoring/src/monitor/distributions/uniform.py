# -*- coding: utf-8 -*-
"""

@author: polak
"""
import random
from distribution import Distribution

class Uniform(Distribution):
    #constractur
    def __init__(self, parmA,parmB):
        Distribution.__init__(self)
        self.parmA = parmA
        self.parmB = parmB
        
        
    def calcProb(self):
        return random.uniform(float(self.parmA), float(self.parmB))
    
    #for debaug     
    def whoAmI(self):
        return "Uniform"
        
        
    def printMe (self):
        print self.parmA ,self.parmB       
        
        
    def toString(self):
        return str("U"+"["+str(self.parmA)+","+str(self.parmB)+"]")    