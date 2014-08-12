from parser import *
import errno
import socket
import thread
import time
import curses
from to_console import *

SIZE = 1024*1024*1024

rospy.init_node('real_ibeo')
pub = rospy.Publisher('/SENSORS/IBEO/2',MultiLaserScan)

def sendToIBEO(prnt):
  '''
  This function manages the data sent to the IBEO sensor.
  There are 2 options:
  1. Reset request
  2. Status request.
  '''
  if prnt.error_occured:
    if prnt.error_occured and (not prnt.reset_sent):
      send_str = 'AFFEC0C200000000' + '00000004' + '0000' + '2010' + '0000000000000000' + '0000' + '0000'
      prnt.generic_message('Reset request is sent')
      decoded_send = send_str.decode('hex')
      sock.send(decoded_send)
      prnt.reset_sent = True
  elif (not prnt.sent_something) and prnt.reset_sent:
    send_str = 'AFFEC0C200000000' + '00000004' + '0000' + '2010' + '0000000000000000' + '0100' + '0000'
    decoded_send = send_str.decode('hex')
    sock.send(decoded_send)
    prnt.sent_something = (True,time.time())
  return prnt

def sendToParser(data):
  '''
  This function receives a string variable data which is the buffered data received from the IBEO.
  
  The data is then processed and checked and if a full '2202' message is received, is sent to the parser
  and a ROS message is generated and published.
  '''
  
  #Check if there is enough data in the buffer for a header.
  if( len(data) <= 48):
    return data
    
  #Check if a magic word exists in the buffer.
  idx = findMagicWord(data)
  if idx == -1:
    return data
  #Get the size of the message specified in the header.
  size = int(getBigEndian(data,idx,8,4),16)
  #If not all of the message's content is in the buffer return and wait for more data.
  if(size > (len(data)-idx)/2):
    return data
  #Check which data type is the data type of the message.
  dtype = getBigEndian(data,idx,14,2)
  prnt.status = dtype
  #If message type is 2202 (measurement) send message to parser in a new thread.
  if dtype == '2202':
    thread.start_new_thread( parse,(data[idx: idx+ 24*2+ size*2],pub,))
  #Return rest of buffer back to main function to catch more data from socket.
  return_data = data[idx+ 24*2+ size*2:]
  #Notify if buffer is not empty.
  if len(return_data) > 0:
    prnt.generic_message('buffer is too large. Discarding:'+str(len(return_data)/2000)+' kBytes')
    return_data = ''
  return return_data

#----Opening socket to IBEO sensor -----#
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(10)
server_address = ('192.168.0.6', 12002)
print 'Trying to open connection...'
sock.connect(server_address)
#sock.setblocking(1)
start_time = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
prnt = to_console(curses.initscr(),server_address,start_time)
rospy.sleep(0.5)
dat = ''
dat_size = SIZE
cnt = 0
exit_message = ''
try:
    while dat_size > 0:
      try:    
	inp = sock.recv(dat_size).encode('hex')
	cnt = 0
      except socket.error, e:
	if e.errno != errno.EINTR:
	  if cnt < 3:
	    prnt.generic_message('Trying to reset connection')
	    prnt.refresh()
	    sock.close()
	    rospy.sleep(1)
	    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	    sock.connect(server_address)
	    dat = ''
	    sock.settimeout(10)
	    cnt += 1
	    continue
	  else:
	    raise
      dat_size -= len(inp)
      prnt.size_of_data += len(inp)
      dat += inp
      dat = sendToParser(dat)
      prnt = sendToIBEO(prnt)
	
      prnt.refresh()
    
finally:
  curses.echo()
  curses.nocbreak()
  curses.endwin()
  print 'closing socket'
  sock.close()
  print 'Started to collect data at: ',start_time
  print 'Finished to collect data at: ',time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
  print 'Received ',(prnt.size_of_data /2) / 1000000.0,' MB.'
  print (exit_message)
  
  
  
  
  
  
  
  
  
  
  
  
  
  