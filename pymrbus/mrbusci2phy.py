"""this is based on the mrbus.py in the bootloader, but intended to be version two with a more separated event layer and imerative layer.  once this seems ok I'll separate it out into it's own project in the mrbus upstream then convert bootloader to use it.
"""
from __future__ import division
import serial
import time
from mrbuspacket import packet


class mrbusci2phy(object):
  def __init__(self, port, addr, logfile=None, logall=False, extra=False):

    if type(port)==str:
      port = serial.Serial(port, 115200, timeout=.1, rtscts=True)

    self.serial = port

    time.sleep(.1)
    while port.inWaiting():
      port.read(port.inWaiting())
    port.write(':CMD NS=00;\r')
    if extra:
      port.write(':CMD MM=00;\r')
    else:
      port.write(':CMD MM=01;\r')
  
    port.timeout=0

    self.pktlst=[]

    self.logfile=logfile
    self.logall=logall
    self.log(0, "instantiated mrbusSimple from %s"%port.name)

    self.addr=addr
#    self.buf=deque()

  def setTimeout(self, to):
    self.serial.timeout = to

  def time(self):
    return time.time()

  def sleep(self, t):
    time.sleep(t)


  def log(self, error, msg):
    if not self.logfile:
      return
    if not (error or self.logall):
      return
    if error:
      s="Error:"
    else:
      s="  log:"
    self.logfile.write(s+repr(msg)+'\n')

#needs timeout functionality
#  def readline(self)
#    while not self.linebuf():
#      r=self.serial.read(max(1, self.serial.inWaiting()))
#      while '\n' in r:
#        i = r.index('\n')
#        self.linebuf.append(list(self.linecbuf)+r[:i+1]        
#        self.linecbuf=deque()
#        r=r[i+1:]
#      if r:
#        self.linecbuf.extend(r)
#    return self.linebuf.leftpop()


  def getpkt(self):
    l = self.serial.readline()
#      self.readline()
    if not l:
      return None
    if l[-1] != '\n' and l[-1] != '\r':
      self.log(1, '<<<'+l)
      return False
    l2=l.strip()
    if l2 == 'Ok':
      self.log(0, '<<<'+l)
      return False
    if len(l2)<2 or l2[0]!='P' or l2[1]!=':':
      self.log(1, '<<<'+l)
      return False
    d=[int(v,16) for v in l2[2:].split()]
    if len(d)<6 or len(d)!=d[2]:
      self.log(1, '<<<'+l)
      return False
    self.log(0, '<<<'+l)
    return packet(d[0], d[1], d[5], d[6:])


  def sendpkt(self, dest, data, src=None):
    if src == None:
      src = self.addr
    s = ":%02X->%02X"%(src, dest)
    for d in data:
      if type(d) == str:
        d=ord(d)
      s+=" %02X"%(d&0xff)
    s+=";\r"
    self.log(0, '>>>'+s)
    self.serial.write(s)


def mrbusci2phy_ex(ser):
  addr=0
  phy = mrbusci2phy(ser, addr)
#  phy = mrbusci2phy(ser, logall=True, logfile=sys.stderr)
  t=phy.time()
  while phy.time()-t < 3:
    phy.sendpkt(0xff, ['A'])
    phy.sleep(.3)
    while 1:
      p = phy.getpkt()
      if not p:
        break
      if p.src!=addr and p.src!=0 and p.src!=0xff:
        print 'recieved reply from node:', p.src


if __name__ == '__main__':
  with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:

    mrbusci2phy_ex(ser)


