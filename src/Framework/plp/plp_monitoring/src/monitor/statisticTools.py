# -*- coding: utf-8 -*-
"""
Created on Sat Mar  8 22:43:33 2014

@author: liat
"""

import math 

def expectation(dist):
    expec = 0.0
    for xi in dist:        
        expec = expec+xi[0]*xi[1]       
    return expec 
    
    
def variance(dist):
    E=expectation(dist)
    var = 0.0
    for xi in dist:        
        var = var+xi[1]*((xi[0]-E)**2)
    return var 
    
    
        