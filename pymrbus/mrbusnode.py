"""this is based on the mrbus.py in the bootloader, but intended to be version two with a more separated event layer and imerative layer.  once this seems ok I'll separate it out into it's own project in the mrbus upstream then convert bootloader to use it.
"""
from __future__ import division
from mrbuscmp import CMP


class node(object):
  def __init__(self, mrb, addr, enableCMP=True):
    def _handler(p):
      if p.src==self.addr and (p.dest==mrb.addr or p.dest==0xff):
        for hint,h in self.handlers:
          if h(p):
            break
        return True #eat packet

    self.mrb=mrb
    self.addr=addr
    self.hint=mrb.install(_handler)

    self.handlern=0
    self.handlers=[]

    self.cmp = CMP(self, enableCMP)

  def __dell__(self):
    self.mrb.remove(self.hint)

  def log(self, level, msg):
    self.mrb.phy.log(level, ('node %02Xh:'%self.addr)+msg)

  def install(self, handler, where=-1):
    #interpret index differently than list.insert().  -1 is at end, 0 is at front
    if where<0:
      if where == -1:
        where = len(self.handlers)
      else:
        where+=1

    hint=self.handlern
    self.log(0, "install handler %d:%s"%(hint,handler))
    self.handlern+=1
    self.handlers.insert(where, (hint, handler))
    return hint

  def remove(self, hint):
    self.log(0, "remove handler %d"%hint)
    self.handlers = [h for h in self.handlers if h[0]!=hint]

  def installTimer(self, when, handler, absolute=False):
    return self.mrb.installTimer(when, handler, absolute)

  def removeTimer(self, hint):
    self.mrb.removeTimer(hint)


  def __str__(self):
    return "node(%02) %s"%(self.addr)

  def sendpkt(self, data):
    self.mrb.sendpkt(self.addr, data)


  def pump(self, duration=None, until=None, eager=False):
    self.mrb.pump(duration, until, eager)

  def pumpout(self):
    self.mrb.pumpout()



###imperative stuff below here


  def getfilteredpkt(self, f, duration=None, until=None):
    pkt=[None]
    def h(p):
      if f(p):
        pkt[0]=p
        return True

    def u():
      return pkt[0]!=None or until != None and until()
      
    hint = self.install(h)
    self.pump(duration, u)
    self.remove(hint)
    return pkt[0]

  def gettypefilteredpktdata(self, t, duration=None):
    if type(t) == str:
      t=ord(t)
    p=self.getfilteredpkt(lambda p: p.cmd==t, duration=duration)
    if p:
      return p.data
    return None

  def doUntilReply(self, cmd, rep=None, delay=.5, timeout=5):
    if rep==None:
      rep=ord(cmd[0].lower())
    for i in xrange(1 + int(timeout//delay)):
      self.sendpkt(cmd)
      d = self.gettypefilteredpktdata(rep, duration=delay)
      if d != None:
        return d
    return None

import serial
import sys
import mrbus

###node example use:
def node_ex(ser):
  mrb = mrbus.mrbus(ser)
  nodes = mrb.scannodes()
  assert nodes

  n=mrb.getnode(nodes[0].src)

  n.pumpout()
  n.sendpkt(['V'])
  print n.gettypefilteredpktdata('v')
  #or this: print n.getfilteredpkt(lambda p: p.cmd==ord('v'), duration=2).data

  #or this: d = n.doUntilReply(['V'])



if __name__ == '__main__':
  with serial.Serial('/dev/ttyUSB0', 115200, timeout=0, rtscts=True) as ser:
    node_ex(ser)


