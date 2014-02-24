import serial
import time
from collections import deque
import sys

class packet(object):
  def __init__(self, dest, src, cmd, data):
    self.dest=dest
    self.src=src
    self.cmd=cmd
    self.data=data

  def __hash__(self):
    return hash(repr(self))

  def __eq__(self, other):
    return repr(self)==repr(other)

  def __repr__(self):
    return "mrbus.packet(0x%02x, 0x%02x, 0x%02x, %s)"%(self.dest, self.src, self.cmd, repr(self.data))

  def __str__(self):
    c='(%02xh'%self.cmd
    if self.cmd >= 32 and self.cmd <= 127:
      c+=" '%c')"%self.cmd
    else:
      c+="    )"
    return "packet(%02xh->%02xh) %s %2d:%s"%(self.src, self.dest, c, len(self.data), ["%02xh"%d for d in self.data])

class node(object):
  def __init__(self, mrb, addr):
    def handler(p):
      if p.src==self.addr and (p.dest==mrb.addr or p.dest==0xff):
        self.pkts.append(p)
      return True #eat packet

    self.mrb=mrb
    self.addr=addr
    self.pkts=deque()
    self.hint=mrb.install(handler, -1)


  def __dell__(self):
    self.mrb.remove(self.hint)

  def __str__(self):
    return "node(%02) %s"%(self.addr)

  def sendpkt(self, data):
    self.mrb.sendpkt(self.addr, data)

  def getpkt(self, timeout=None):
    if len(self.pkts) == 0:
      self.mrb.pump(timeout=timeout)
    if len(self.pkts) == 0:
      return None
    return self.pkts.popleft()



class mrbusSimple(object):
  def __init__(self, port, addr, logfile=None, logall=False):

    if type(port)==str:
      port = serial.Serial(port, 115200, timeout=0)

    self.serial = port

    self.pktlst=[]

    self.logfile=logfile
    self.logall=logall
    self.log(0, "instantiated mrbusSimple from %s"%port.name)

    self.addr=addr


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


  def getpkt(self):
    while 1:
      l = self.serial.readline()
      if not l:
        return None
      if l[-1] != '\n' and l[-1] != '\r':
        self.log(1, '<<<'+l)
        continue
      l2=l.strip()
      if len(l2)<2 or l2[0]!='P' or l2[1]!=':':
        self.log(1, '<<<'+l)
        continue
      d=[int(v,16) for v in l2[2:].split()]
      if len(d)<6 or len(d)!=d[2]:
        self.log(1, '<<<'+l)
        continue
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
    time.sleep(.05)        


class mrbus(object):
  def __init__(self, port, addr=None, logfile=None, logall=False):
    if type(port)==str:
      port = serial.Serial(port, 115200)

    self.mrbs = mrbusSimple(port, addr, logfile, logall)

    self.pktlst=[]
    self.handlern=0
    self.handlers=[]

    self.mrbs.log(0, "instantiated mrbus from %s"%port.name)

    #find an address to use
    if addr==None:
      self.mrbs.log(0, "finding address to use")
      for addr in xrange(254, 0, -1):
        found = self.testnode(addr, replyto=0xff)
        if not found:
          break
      if found:
        self.mrbs.log(1, "no available address found to use")
        raise Exception("no available address found to use")
     
    self.addr=addr
    self.mrbs.addr=addr

    self.mrbs.log(0, "using address %d"%addr)


  def sendpkt(self, addr, data, src=None):
    self.mrbs.sendpkt(addr, data, src)

  def getpkt(self):
    return self.mrbs.getpkt()

  def getnode(self, dest):
    return node(self, dest)

  def install(self, handler, where=-1):
    #interpret index differently than list.insert().  -1 is at end, 0 is at front
    self.mrbs.log(0, "install handler")
    if where<0:
      if where == -1:
        where = len(self.handlers)
      else:
        where+=1

    hint=self.handlern
    self.handlern+=1
    self.handlers.insert(where, (hint, handler))

  def remove(self, hint):
    self.mrbs.log(0, "remove handler")
    self.handlers = [h for h in self.handlers if h[0]!=hint]


  def pump(self, timeout=None):
    done=False
    to = self.mrbs.serial.timeout
    if timeout != None:
      timeout=max(0,timeout)
    self.mrbs.serial.timeout=timeout
    while not done:
      p = self.getpkt()
      if p:
        self.mrbs.serial.timeout=0
        for hint,h in self.handlers:
          r = h(p)
          if r:
            break
      else:
        done=True
    self.mrbs.serial.timeout=to

  def testnode(self, addr, replyto=None, wait=2):
    found=False

    def pingback(p):
      if p.src==addr:
        found=True
      if p.cmd=='a':
        return True #eat pings
      return False

    if replyto == None:
      replyto = self.addr

    hint = self.install(pingback, 0)

    t=time.time()
    n=0
    while time.time()-t < wait and not found:
      x=(time.time()-t)/.2
      if x > n:
        self.sendpkt(addr, ['A'], src=replyto)
        n+=1
      tn=time.time()
      to=min(wait+t-tn, n*.2+t-tn)
      self.pump(to)

    self.remove(hint)
    return found


        
  def scannodes(self, pkttype=ord('A'), rettype=ord('a'), wait=2):
    targets=set()

    def pingback(p):
      if p.src!=self.mrbs.addr and p.src!=0 and p.src!=0xff and p.cmd==rettype:
        targets.add(p.src)
      return False

    hint = self.install(pingback, 0)

    t=time.time()
    n=0
    while time.time()-t < wait:
      x=(time.time()-t)/.3
      if x > n:
        self.sendpkt(0xff, [pkttype])
        n+=1
      tn=time.time()
      to=min(wait+t-tn, n*.3+t-tn)
      self.pump(to)

    self.remove(hint)
    return sorted(targets)



###mrbus example use:
def mrbus_ex(ser):
  mrb = mrbus(ser)
#  mrb = mrbus(ser, logall=True, logfile=sys.stderr)
  nodes = mrb.scannodes()
  print 'nodes: '+', '.join(str(n) for n in nodes)


###node example use:
def node_ex(ser):
  mrb = mrbus(ser)
  nodes = mrb.scannodes()
  assert nodes

  n=mrb.getnode(nodes[0])

  n.sendpkt(['V'])
  p=n.getpkt(timeout=3)
  if p:
    print p
  else:
    print 'no packet returned'


###mrbusSimple example use:
def mrbussimple_ex(ser):
  addr=0
  mrbs = mrbusSimple(ser, addr)
#  mrbs = mrbusSimple(ser, logall=True, logfile=sys.stderr)
  t=time.time()
  while time.time()-t < 3:
    mrbs.sendpkt(0xff, ['A'])
    time.sleep(.3)
    while 1:
      p = mrbs.getpkt()
      if p==None:
        break
      if p.src!=addr and p.src!=0 and p.src!=0xff:
        print 'recieved reply from node:', p.src


if __name__ == '__main__':
  with serial.Serial('/dev/ttyUSB0', 115200, timeout=0) as ser:

#    mrbussimple_ex(ser)
#    mrbus_ex(ser)
    node_ex(ser)


