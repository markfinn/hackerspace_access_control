import sys
sys.path.insert(0, '../../mrbus_bootloader')
import mrbus
from collections import deque


class Evobj(object):
  def __init__(self, sim=None):
    self.e=[]
    if sim:
      sim.add(self)

  def getNextEventTime(self):
    if self.e:
      return self.e[0]

  def doEvent(self, n):
    assert self.e[0][0] == n
    self.e[0][1](self)
    self.e=self.e[0][1:]

  def setsim(self, sim):
    self.sim=sim



class Sim(object):
  def __init__(self):
    self.objs=[]
    self.t=0

  def add(self, o):
    self.objs.append(o)
    o.setsim(self)

  def run(self):
    while True:
      soonest=None
      for o in self.objs:
        n = o.getNextEventTime()
        if n != None and (soonest == None or n < soonest[0]):
          soonest = n, o
      if soonest:
        if self.t < soonest[0]:
          self.t = soonest[0]+.01
        else:
          soonest[0]+=.01
        soonest[1].doEvent(soonest[0])
      else:
        assert 0
      
        
  def time(self):
    self.t+=.0001
    return self.t

  def sleep(self, t):
    assert 0



class FakeBus(Evobj):
  def __init__(self, sim=None):
    Evobj.__init__(self, sim)
    self.nodes=[]

  def addnode(self, mrbs):
    self.nodes.append(mrbs)

  def time(self):
    return self.sim.time()

  def sleep(self, t):
    self.sim.sleep(t)

  def tx(self, dest, src, data):
    p = mrbus.packet(dest, src, data[0], data[1:])
    for n in self.nodes:
      n.pktlst.append(p)



class FakeMRBusSimple(Evobj):
  def __init__(self, fakeBus, addr=None, logfile=None, logall=False, extra=False):
    Evobj.__init__(self)


    self.timeout=.1

    self.fakeBus=fakeBus
    self.pktlst=deque()
    fakeBus.addnode(self)

    self.logfile=logfile
    self.logall=logall
    self.log(0, "instantiated FakeMRBusSimple from FAKE")

    self.addr=addr

  def name(self):
    return 'FAKE'

  def setTimeout(self, to):
    self.timeout=to

  def getpkt(self):
    if self.pktlst:
      return self.pktlst.popleft()

    if self.timeout == 0:
      return None

    assert 0

  def sendpkt(self, dest, data, src=None):
    if src == None:
      src = self.addr
    z=[]
    for d in data:
      if type(d) == str:
        d=ord(d)
      z.append(d)
#    self.log(0, '>>>'+s)
    self.fakeBus.tx(dest, src, data)




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

  def time(self):
    return self.fakeBus.time()

  def sleep(self, t):
    self.fakeBus.sleep(t)





class RandUser(Evobj):
  def __init__(self, send, recv):
    Evobj.__init__(self)
    self.send = send
    self.recv = recv










def test():


  sim = Sim()

  net = FakeBus(sim)

  p1= FakeMRBusSimple(net)
  p2= FakeMRBusSimple(net)

  mrb1 = mrbus.mrbus(p1, addr=1)#, logall=True, logfile=sys.stdout, extra=True)
  mrb2 = mrbus.mrbus(p2, addr=2)#, logall=True, logfile=sys.stdout, extra=True)

  n2 = mrb1.getnode(2)
  n1 = mrb2.getnode(1)

  r1=RDP(n1)
  r2=RDP(n2)


  r2.listen()
  r1.open(2)
 
  sim.run()

  rnd1=RandUser(n1, n2)
  sim.add(rnd1)
  rnd2=RandUser(n2, n1)
  sim.add(rnd2)



test()



