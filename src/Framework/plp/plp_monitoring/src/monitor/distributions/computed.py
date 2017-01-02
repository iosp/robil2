# -*- coding: utf-8 -*-
"""

@author: polak
"""
import math
import random
from distribution import Distribution
#import SumRandomVariables

class Computed(Distribution):
    #constractur
    def __init__(self,Map = {}):
        Distribution.__init__(self)
         #map is a python dictionary{[time,count]..}    
        self.map = Map
        
       
    def calcProb(self):
        x = random.random()
        return self.distNormalize(x)
    
    
    def distNormalize(self, x):
        s = 0.0
        a = 0
        for i in self.map.values():
            if i !='':
                s=s+float(i)
        for i in self.map:
            if  i!='':
                value = self.map.get(i)
                if value!='':
                    a = a + (float(value))/s
                    if (a >= x):
                        return float(i)
            
    #search for the time key in the dictionary- return 0  or count.- value
    def getCountByTime(self,time):
        ans = self.map.get(float(time))        
        if (ans != None):
            return ans
        else:
            ans = self.map.get(str(time))
        if (ans != None):
            return ans
        return 0
    #search for the time key in the dictionary- and update it. or create it with value.   
    #Changed by RAZ - removed casting value to str    
    def setValueToTime(self,time,value):
        if (self.map.get(float(time)) != None):
            self.map[float(time)]=value           
        else:
            self.map.setdefault(float(time) , value)
     
       
    def stringToDictionary(self,string):
        pass
    
   #for debaug     
    def whoAmI(self):
        return "Computed"
        
    def printMe (self):
        if len(self.map) == 0:
            print "[]"
        else:
            for i in self.map:
                print i, self.map.get(i)
                
                
 #   def toString(self):
  #      string ="C["
  #      i=0
  #      for index in self.map:
            #print i
  #          i=i+1
  #          string+= str("["+str(index)+","+str(self.map.get(index))+"]")
  #          #print string
  #      string+="]"
  #      return string   

                
    def toString(self):
        sumOfCounters=0
        #print self.map
        string ="C["
        
        sortKeyList = []
        for key in self.map :
            if key == '':
                break
            sortKeyList.append(float(key))
            sumOfCounters = sumOfCounters+float(self.getCountByTime(key))
        sortKeyList = sorted(sortKeyList)
        
        for key in sortKeyList :
        #for index in self.map:
            #print i
            string+= str("["+str(key)+","+str(round(float(self.getCountByTime(key))/float(sumOfCounters),4))+"]")
            
            #print string
        string +="]"
       
        return string  
        
    #prints the time and the amount of accurance (counter) not the distribution - first version Liat    
    def toStringNumValue(self):
        string ="C["
        
        sortKeyList = []
        for key in self.map :
            if key == '':
                break
            sortKeyList.append(float(key))
        sortKeyList = sorted(sortKeyList)
        
        for key in sortKeyList :
        #for index in self.map:
            #print i
            string+= str("["+str(key)+","+str(self.getCountByTime((key)))+"]")
            
            #print string
        string +="]"
       
        return string     
  
    def LessThenT(self,T):
        numOfValues = 0.0
        numLessT = 0.0
        timeValueMap = self.map
        for key in timeValueMap:
            numOfValues = numOfValues + float(timeValueMap.get(key))
            if (float(key)<=T):
                numLessT=numLessT + float(timeValueMap.get(key))
        if numOfValues==0:
            return float('Inf')        
        return round(numLessT / numOfValues,4)  
        
        
    def calcAverageTime(self):
        numOfValues = 0.0
        totalOfValues = 0.0
        timeValueMap = self.map
        for key in timeValueMap:
            numOfValues = numOfValues + float(timeValueMap.get(key))
            totalOfValues = totalOfValues + (float(key) * float(timeValueMap.get(key)))
        if  numOfValues==0:
            return float('Inf')
        return totalOfValues / numOfValues   
        
        
    def SDTime(self):
       avg = self.calcAverageTime()
       timeValueMap = self.map
       var = 0.0
       numOfValues =0.0
       for key in timeValueMap:
           numOfValues = numOfValues + float(timeValueMap.get(key))
           var = var + ((float(key) - avg)**2)*float(timeValueMap.get(key))
       if  numOfValues==0:
            return float(0)    
       return math.sqrt(var/numOfValues)    
           
    def getData(self):
       duration = [float(i) for i in self.map.keys()]
       times = [float(j) for j in self.map.values()]
       return (duration, times)

    def toMatrix(self):
       m=[]
       times=[float(j) for j in self.map.values()]  
       timeSum=sum(times)
       for key in self.map:
           #may be better not to round here round(float(self.map.get(key))/timeSum,4))
           m.append((float(key),float(self.map.get(key))/timeSum))
       return m
       
    def setComputedDistTable(self,distObj):
        theta=distObj.getTheta()
        dist=distObj.getDist()
        for i in range(len(dist)):
            if dist[i]!=0:
                self.map[i*theta]=dist[i]   
          
        

             
           
    
       
           
            
