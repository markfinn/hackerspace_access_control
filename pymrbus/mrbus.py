"""this is based on the mrbus.py in the bootloader, but intended to be version two with a more separated event layer and imerative layer.  once this seems ok I'll separate it out into it's own project in the mrbus upstream then convert bootloader to use it.
"""
from __future__ import division
from mrbusci2phy import mrbusci2phy as mrbusphy
from mrbusnode import node



class mrbus(object):
  def __init__(self, phy, addr=None, logfile=None, logall=False, extra=False):

    if type(phy)==str:
      phy = serial.Serial(phy, 115200, rtscts=True)

    if type(phy)==serial.Serial:
      self.phy = mrbusphy(phy, addr, logfile, logall, extra)
    else:
      self.phy = phy

    self.pktlst=[]
    self.handlern=0
    self.handlers=[]
    self.timeHandlers = []

    self.kill=False

    self.phy.log(0, "instantiated mrbus from %s"%phy.name)

    self.pumpout()
    
    #find an address to use
    if addr==None:
      self.phy.log(0, "finding address to use")
      for addr in xrange(254, 0, -1):
        found = self.testnode(addr, replyto=0xff)
        if not found:
          break
      if found:
        self.phy.log(1, "no available address found to use")
        raise Exception("no available address found to use")
     
    self.addr=addr
    self.phy.addr=addr

    self.phy.log(0, "using address %d"%addr)


  def sendpkt(self, addr, data, src=None):
    self.phy.sendpkt(addr, data, src)

  def getnode(self, dest):
    return node(self, dest)

  def install(self, handler, where=-1):
    #interpret index differently than list.insert().  -1 is at end, 0 is at front
    if where<0:
      if where == -1:
        where = len(self.handlers)
      else:
        where+=1

    hint=self.handlern
    self.phy.log(0, "install handler %d:%s"%(hint,handler))
    self.handlern+=1
    self.handlers.insert(where, (hint, handler))
    return hint

  def remove(self, hint):
    self.phy.log(0, "remove handler %s"%hint)
    self.handlers = [h for h in self.handlers if h[0]!=hint]

  def installTimer(self, when, handler, absolute=False):
    if not absolute:
      when += self.phy.time()
    self.phy.log(0, "install timer for %s"%when)
    hint=self.handlern
    self.handlern+=1
    self.timeHandlers.append((when, hint, handler))
    self.timeHandlers.sort(reverse=True)
    return hint

  def removeTimer(self, hint):
    self.phy.log(0, "remove timer")
    self.timeHandlers = [h for h in self.timeHandlers if h[1]!=hint]

  def pump(self, duration=None, until=None, eager=False):
    start = self.phy.time()
    now = start
    while eager or not self.kill and (duration==None or duration+start > now) and (until==None or not until()):
      while self.timeHandlers and self.timeHandlers[-1][0] <= now:
        h = self.timeHandlers.pop()
        h[2]()
        now = self.phy.time()
      if self.timeHandlers:
        to = self.timeHandlers[-1][0] - now
      else:
        to = .1
      if duration != None:
        to=min(to,max(0,duration+start-now))
      self.phy.setTimeout(to)
      p = self.phy.getpkt()
      if p:
        for hint,h in self.handlers:
          if h(p):
            break
      elif p==None:
        eager=False
      now = self.phy.time()

  def pumpout(self):
    self.pump(duration=0, eager=True)


###imperative stuff below here

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

    t=self.phy.time()
    n=0
    while self.phy.time()-t < wait and not found:
      x=(self.phy.time()-t)/.2
      if x > n:
        self.sendpkt(addr, ['A'], src=replyto)
        n+=1
      tn=self.phy.time()
      to=min(wait+t-tn, n*.2+t-tn)
      self.pump(to)

    self.remove(hint)
    return found


        
  def scannodes(self, pkttype=ord('A'), rettype=None, wait=2):
    targets=set()

    if rettype==None:
      rettype=ord(chr(pkttype).lower())

    def pingback(p):
      if p.src!=self.phy.addr and p.src!=0 and p.src!=0xff and p.cmd==rettype:
        targets.add(p)
      return False

    hint = self.install(pingback, 0)

    t=self.phy.time()
    n=0
    while self.phy.time()-t < wait:
      x=(self.phy.time()-t)/.3
      if x > n:
        self.sendpkt(0xff, [pkttype])
        n+=1
      tn=self.phy.time()
      to=min(wait+t-tn, n*.3+t-tn)
      self.pump(to)

    self.remove(hint)
    return sorted(targets)


import serial
import sys

###mrbus example use:
def mrbus_ex(ser):
  mrb = mrbus(ser)
#  mrb = mrbus(ser, logall=True, logfile=sys.stderr)
  nodes = mrb.scannodes()
  print 'nodes: '+', '.join(str(n.src) for n in nodes)




if __name__ == '__main__':
  with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:

    mrbus_ex(ser)


