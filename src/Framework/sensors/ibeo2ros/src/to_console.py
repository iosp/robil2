import curses,signal, sys, time
def signal_handler(signal, frame):
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

class to_console:
  def __init__(self,stdscr,server_address,start_time):
    self.reset_sent = False
    self.size_of_data = 0
    self.status = 0
    self.error_occured = False
    self.sent_something = (False,time.time())
    self.line_number = 0
    self.line_bias = 0
    self.stdscr = stdscr
    self.status_time = [time.time(),time.time(),time.time()]
    self.generic_message_time = [0,0,0,0,0,0,0]
    curses.noecho()
    curses.cbreak()
    curses.start_color()
    self.stdscr.addstr(0,0,"-----IBEO READER-----")
    self.stdscr.addstr(1,0,"=====================")
    self.stdscr.addstr(2,0,"Press ctrl+C to exit.")
    self.stdscr.addstr(3,0,"=====================")
    self.stdscr.addstr(4,0,"Connected to IBEO. ip: "+server_address[0]+" port: "+str(server_address[1]))
    self.stdscr.addstr(5,0,"Started at "+start_time)
    self.refresh()
    
  def linepp(self):
    self.line_number += 1
    return (self.line_number - 1)
  
  def set_time(self):
    self.stdscr.addstr(6,0,"Time now: "+time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime()))
    
  def refresh(self):
    self.set_time()
    self.send_status()
    self.size_of_data_processed()
    self.prnt_status()
    self.generic_message()
    self.stdscr.refresh()
  
  def size_of_data_processed(self):
    if self.size_of_data < 2000000:
      self.stdscr.addstr(7,0,"Processed till now "+str(self.size_of_data/2/1000.0)+" kBytes\n")
    else:
      self.stdscr.addstr(7,0,"Processed till now "+str(self.size_of_data/2/1000/1000.0)+" MBytes\n")
    
  def prnt_status(self):
    if self.status == 0:
      self.stdscr.addstr(9,0,"Status: Pending for data.")
    elif self.status == '2202':
      self.stdscr.addstr(9,0,"Status: Received data type 2202. Everything is OK.")
      self.stdscr.clrtoeol()
      self.status_time[0] = time.time()
      self.error_occured = False
    elif self.status == '2020':
      self.error_occured = True
      curses.init_pair(4, 4, curses.COLOR_BLACK)
      self.stdscr.addstr(10,0,"Status: IBEO answered status request.",curses.color_pair(4))
      self.stdscr.clrtoeol()
      self.status_time[1] = time.time()
      self.reset_sent = False
    elif self.status == '2030':
      self.error_occured = True
      curses.init_pair(1, 1, curses.COLOR_WHITE)
      self.stdscr.addstr(11,0,"Status: Received data type 2030. Warning/Error. Check connection and voltage level",curses.color_pair(1) | curses.A_BOLD)
      self.status_time[2] = time.time()
    else:
      self.stdscr.addstr(9,0,"Status: Received data type "+dtype,curses.color_pair(4))
      self.stdscr.clrtoeol()
    for i in range(3):
      if (self.status_time[i] - time.time()) > 1.5:
	self.clear_line(9+i)
      
  def send_status(self):
    if self.sent_something[0]:
      self.stdscr.addstr(12,0,'Sending status request to IBEO')
      if (time.time() - self.sent_something[1]) > 3:
	self.clear_line(12)
      if (time.time() - self.sent_something[1]) > 20:
	self.sent_something = (False,time.time())
  
  def clear_line(self,line_number):
    self.stdscr.addstr(line_number,0,'')
    self.stdscr.clrtoeol()
  
  def generic_message(self,data=''):
    self.stdscr.addstr(14,0,'Generic Message:')
    if data:
      self.generic_message_time[self.line_number] = time.time()
      num = 15 + self.line_number
      if self.line_number == 5:
	self.line_number = -1
      self.line_number += 1
      self.stdscr.addstr(num,0,data)
      self.stdscr.clrtoeol()
    for t in range(5):
      if (time.time() - self.generic_message_time[t]) > 3:
	self.clear_line(15+t)
	
    