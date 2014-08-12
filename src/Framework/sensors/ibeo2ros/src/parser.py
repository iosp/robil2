from header import *
import rospy

def findMagicWord(st,start=0):
    return st.find('affec0c2',start)

def getLittleEndian(st,idx,offset,size,value):
    try:
      if size == 1:
	  return value + int(st[idx+offset*2:idx+offset*2+2],16)
      beg = idx+(offset+size-1)*2
      en = idx+(offset+size-1)*2+2
      h = st[beg:en]
      return getLittleEndian(st,idx,offset,size-1,(value + int(h,16)) * int('0x100',16))
    except:
      return -1
          
def getBigEndian(st,idx,offset,size,as_hex=1):
  if as_hex:
    return st[idx+offset*2:idx+offset*2+size*2]
  else:
    return int(st[idx+offset*2:idx+offset*2+size*2],16)

def uint2int(num):
  return numpy.int16(numpy.uint16(num))

  
def parse(st,pub):
  
    idx = findMagicWord(st,0)
    
    if idx == -1:
	return -1
    if(idx == 0):
	size = int(getBigEndian(st,idx,8,4),16)
	#print 'size after thread: ',len(st)
	if not (size*2 + 48) == len(st):
	  #print 'size is not good'
	  return
	  
	step = getLittleEndian(st,idx,46,2,0)
	s = uint2int(getLittleEndian(st,idx,48,2,0))
	e = uint2int(getLittleEndian(st,idx,50,2,0))
	numOfPoints = getLittleEndian(st,idx,52,2,0)
	points = []
	for i in range((size-44)/10):
	  layerEcho = getLittleEndian(st,idx,24+44+i*10,1,0)
	  flag = getLittleEndian(st,idx,24+44+i*10 +1,1,0)
	  HoAngle = uint2int(getLittleEndian(st,idx,24+44+i*10 +2,2,0))
	  RadDist = getLittleEndian(st,idx,24+44+i*10 +4,2,0)
	  EchoPulseWidth = getLittleEndian(st,idx,24+44+i*10 +6,2,0)
	  if layerEcho == -1 or flag == -1 or HoAngle == -1 or RadDist == -1 or EchoPulseWidth == -1:
	    print 'message stopped. Should have been ',(size-44)/10,' are: ',len(points)
	    break
	  p = Point(layerEcho,flag,HoAngle,RadDist,EchoPulseWidth)
	  points.append(p)
	
	rosPoints = MeasureData()
	rosPoints.set_point_array_as_ROS(points,s,e,step)
	pub.publish(rosPoints._rosPointArray)
    
    else:
	print 'something is wrong: magic word is not at the begging'