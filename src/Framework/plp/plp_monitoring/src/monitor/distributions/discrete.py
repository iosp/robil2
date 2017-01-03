# -*- coding: utf-8 -*-
"""

@author: polak
"""
from distribution import Distribution

class Discrete(Distribution):
    #constractur
    def __init__(self,listOFparms):
        Distribution.__init__()
        
        
    def calcProb(self):
        raise NotImplementedError("calcProb-discrete")
    
    
    #for debaug     
    def whoAmI(self):
        return "Discrete"