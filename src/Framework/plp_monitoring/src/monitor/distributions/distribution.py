# -*- coding: utf-8 -*-
"""

@author: polak
"""

class Distribution:
    #constractur
    def __init__(self):
        pass
    
    def calcProb(self):
        raise NotImplementedError("Subclasses should implement this!") 
        
    #for debaug     
    def whoAmI(self):
        return "Distribytion"
        
  