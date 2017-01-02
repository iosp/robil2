# -*- coding: utf-8 -*-
"""
Created on Sun Jun 22 19:39:41 2014

@author: liat
"""

class distObject():
    def __init__(self,dist,theta=1):
        self.dist=dist
        self.theta=theta
        
    def setTheta(self,theta):
        self.theta = theta

    def getDist(self):
        return self.dist

    def getTheta(self):
        return self.theta        



def _MaxSum(data):
    sumell=0
    for dist in data:        
        sumell+=max(dist)[0]
    return int(sumell )

         
             
    
# compute distribution acuurate seqAnd
def SumAccurateDescrete(data):
    size=_MaxSum(data)+1
    M = [0]*(size)
    tempM = [1]
    for dist in data:
        for (val,prob) in dist:
            for i in range(len(tempM)):
                if tempM[i]!=0:
                    M[int(val)+i]=prob*tempM[i] + M[int(val)+i]
        tempM=M
        #print M
        M = [0]*size
    dist = distObject(tempM)    
    return dist


# compute distribution acuurate seqOr
def OrSumAccurateDescrete(dataSucc,dataFail,probSucc):
    size=_MaxSum(dataSucc)+_MaxSum(dataFail)+1
    M = [0]*(size)
    data=[]
    probFail = 1
    for i in range(len(dataSucc)):
        #print i
        for j in range(i):
            data.append([(ell, probEll*(1-probSucc[j])) for (ell,probEll) in dataFail[j]]) 
        data.append([(ell, probEll*probSucc[i]) for (ell,probEll) in dataSucc[i]]) 
        distPartial = SumAccurateDescrete(data)
        data=[]
        for k in range(len(distPartial.getDist())):
            M[k]+=distPartial.getDist()[k]            
        probFail*= (1-probSucc[i])    
    if  (1-probFail)!=0:   
        M= [prob*1/(1-probFail) for prob in M]    
    dist = distObject(M)    
    return dist    

#get data and returns (1) sorted data (2)a map with all values (3) sorted array af values
def _AllTimes(data):
    allTimes={}
    for i in range(len(data)):
        data[i]=sorted(data[i])
        for (val,prob) in data[i]:
            allTimes[val]=1
    sortedMapVal= sorted(allTimes)      
    #allTimes=sorted(allTimes) 
    return data, allTimes, sortedMapVal  


# compute the dist in a case of palAnd    
def MaxAccurateDescrete(data):
    (data,allTimes,sortedVals)=_AllTimes(data)
    sumi=0
    for ell in sortedVals:
        for dist in data:
            for (val,prob) in dist:
                if (val<=ell):
                    sumi+=prob
            
            allTimes[ell]*=sumi
            sumi=0
    return CDFToPDF(allTimes, sortedVals)


# compute the dist in a case of palOr
def MinAccurateDescrete(data):
    (data,allTimes,sortedVals)=_AllTimes(data)
    sumi=0    
    for ell in sortedVals:
        for dist in data:
            for (val,prob) in dist:
                if (val>ell):
                    sumi+=prob
            allTimes[ell]*=sumi
            sumi=0
        allTimes[ell]=1-allTimes[ell] 
    return CDFToPDF(allTimes, sortedVals)


#get data in a form of CDF and return PDF
def CDFToPDF(mapCdf,arrCdf):    
    arrCdf.reverse()
    for i in range(len(arrCdf)-1):
        mapCdf[arrCdf[i]]=mapCdf[arrCdf[i]]-mapCdf[arrCdf[i+1]]
        
    return  mapCdf  
    

#max element in dist table
def maxElement(m):
    maxE = 0
    for i in range(len(m)):
        for j in range(len(m[i])):
            if (m[i][j][0]>maxE):
              maxE = m[i][j][0]
    return maxE          


# round a float number up "u" or down "n"  or normal "n"    
def roundUpOrDown(direc,num,dig):
    n = round(num,dig)
    if(direc=="u"):        
        if (n - num)<0:
            n = n+ (10**(-dig))            
    elif (direc=="d"):
        if (n - num)>0:
            n = n - (10**(-dig))
    else:
        n=n        
    return n        

# compute the manipulated matrix original matrix devided by theta
def newMatrix(data,e,n,vmax,direc):
    newData=[]
    for i in range(len(data)):
        newData.append([])
        for j in range(len(data[i])):
            newData[i].append ((int(roundUpOrDown(direc,(data[i][j][0])/(vmax*e/n),0)), data[i][j][1]))     
    return newData

#NOT RELEVANT!!!----------------------------------------------------------------
# compute approximated distribution in a seqAnd node return distObject
def SumApproximateDescrete(data,e,n,direc="n"):
    vmax=maxElement(data)
    theta = (vmax*e/n)
    datan = newMatrix(data,e,n,vmax,direc)
    dist = SumAccurateDescrete(datan)
    #-----------------------------------------------idea--------------------
#    tempM=temp[1]
#    M=[0]*int(round(len(tempM)*theta)+1)
#    for i in range(len(tempM)):
#            if tempM[i]!=0:
#                M[int(round(i*theta))]=tempM[i]
    #-----------------------------------------------end idea-----------------
    #
    #(1,M)
    dist.setTheta(theta)
    return dist
#NOT RELEVANT!!!--------------------------------------------------------
# compute approximated distribution in a seqOr node return distObject
def OrSumApproximateDescrete(dataSucc,dataFail,probSucc,e,n,direc="n"):
    vmax=max(maxElement(dataSucc), maxElement(dataFail))
    theta = (vmax*e/n)    
    dataSuccn=newMatrix(dataSucc,e,n,vmax,direc)
    dataFailn=newMatrix(dataFail,e,n,vmax,direc)
    dist = OrSumAccurateDescrete(dataSuccn,dataFailn,probSucc)
    dist.setTheta(theta)
    return dist



# compute sum less then T
def sumArrayToMap(distObj,T):
    theta=distObj.getTheta()
    dist=distObj.getDist()
    sumi=0
    for i in range(len(dist)):
        if dist[i]!=0:
            if i*theta<=T:#i*theta<T:
                sumi+=dist[i]               
    return sumi            
            
def Test(distObj):
    mapi={}
    theta=distObj.getTheta()
    dist=distObj.getDist()
    for i in range(len(dist)):
        if dist[i]!=0:
            mapi[i*theta]=dist[i] 
    print mapi        

    
# compute the upper bound in case of approximation and seqAnd node    
def computeUpperBound(T,e,n,data): 
    distobj = SumApproximateDescrete(data,e,n,"n")
    print "normal"
    Test(distobj)
    distobj = SumApproximateDescrete(data,e,n,"d")
    print "down"
    Test(distobj)
    return sumArrayToMap(distobj,T)

# compute the lower bound in case of approximation and seqAnd node    
def computeLowerBound(T,e,n,data):
    distobj = SumApproximateDescrete(data,e,n,"u")
    print "up"
    Test(distobj)
    return sumArrayToMap(distobj,T)  

# compute the upper bound in case of approximation and seqOr node        
def OrComputeUpperBound(d1,d2,p,T,e,n): 
    distobj = OrSumApproximateDescrete(d1,d2,p,e,n,"d")
    return sumArrayToMap(distobj,T)

# compute the lower bound in case of approximation and seqOr node        
def OrComputeLowerBound(d1,d2,p,T,e,n):
    distobj = OrSumApproximateDescrete(d1,d2,p,e,n,"u")
    return sumArrayToMap(distobj,T)      

def gridyAlgo(data,error,n):
    e=error/n
    M={}
    temp=-1
    tempM = {0:1}
    for dist in data:
        for (val,prob) in dist:
            for i in tempM:
                if tempM[i]!=0:    
                    if M.has_key(int(val)+i):
                        M[int(val)+i]=prob*tempM[i] + M[int(val)+i] 
                    else:
                        M[int(val)+i]=prob*tempM[i]+0
                        
        if (len(M.keys())>int(1/e)+1):
            d= sorted(M)  
            for key in d:
                if (M.has_key(temp) and M[temp]+M[key]<e):                    
                    M[temp]=M[temp]+M[key]
                    M.pop(key)
                else:
                    temp=key                    
        temp=-1                           
        tempM=M
        M = {}
    dist = distObject(tempM) 
    #prob=sumMapLessThenT(dist,T)
    return dist
    
if __name__ == "__main__":
    data = [[(2, 0.5), (3, 0.5)], [(1, 0.5), (2, 0.5)], [(2.0,0.5),(1001.0,0.5)], [(1, 0.75), (4, 0.25)]]
#    data2 = [[(20, 0.5), (30, 0.5)], [(10, 0.5), (20, 0.5)], [(20.0,0.5),(1001.0,0.5)], [(10, 0.75), (40, 0.25)]]
    
#    prob=[0.8,0.6]
#    d1=[[(1, 0.5), (2, 0.5)],[(3, 0.5), (4, 0.5)]]
#    d2=[[(10, 0.5), (20, 0.5)],[(30, 0.5), (40, 0.5)]]
    print SumAccurateDescrete(data).getDist()
    print sumArrayToMap(SumAccurateDescrete(data),10)
    data = [[(2,1.0)], [(1, 0.5), (2, 0.5)], [(2.0,0.5),(1001.0,0.5)], [(1, 0.75), (4, 0.25)]]
    print sumArrayToMap(SumAccurateDescrete(data),10)
#    print _AllTimes(data)
#    print MinAccurateDescrete(data)
#    sumi=0
#    a=[[11.0, 0.0029], [12.0, 0.0176], [13.0, 0.0469], [14.0, 0.0771], [15.0, 0.0938], [16.0, 0.0918], [17.0, 0.0723], [18.0, 0.0469], [19.0, 0.0283], [20.0, 0.0156], [21.0, 0.0059], [22.0, 0.001], [1010.0, 0.0029], [1011.0, 0.0176], [1012.0, 0.0469], [1013.0, 0.0771], [1014.0, 0.0938], [1015.0, 0.0918], [1016.0, 0.0723], [1017.0, 0.0469], [1018.0, 0.0283], [1019.0, 0.0156], [1020.0, 0.0059], [1021.0, 0.001]]
#    for (a1,a2) in a:
#        sumi+=a2
#    print sumi    
#        



                    
       