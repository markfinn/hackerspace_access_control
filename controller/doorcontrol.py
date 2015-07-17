#!/bin/python -tt

import sys
import argparse
import os
import struct

sys.path.insert(0, '../../mrbus_bootloader')
import mrbus


def readPins(fn):
  import re
  p={}
  with open(fn, 'r') as f:
    for l in f:
      if l[-1]=='\n':
        l=l[:-1]
      if not l.strip():
        continue
      m=re.match(r'\s*/\*\s*(\d+)\s*\*/\s*{\s*(\d+)\s*,\s*(\d+)\s*}\s*,?\s*(?://.*)?', l)
      if m:
        try:
          id,ln,pin = (int(x) for x in m.groups())
          if ln==0:
            continue
          if ln>=4:
            pinstr = ('%%0%uu'%ln)%pin
            p[id] = pinstr
            continue
        except:
          pass
      print 'pin import from %s dropped line "%s"'%(fn, l)
  return p

def PinLenDecode(pin):
  l=9
  while l and not(pin&1):
    l-=1
    pin>>=1
  pin>>=1;
  return ('%%0%uu'%l)%pin


def run(pintest):
  mrb = mrbus.mrbus('/dev/ttyUSB0', addr=0xfe)#, logall=True, logfile=sys.stdout, extra=True)

  def debughandler(p):
    if p.cmd==ord('*'):
      print 'debug:', p
      return True #eat packet
#    else:
#      print 'packet:', p
    return False #dont eat packet
  mrb.install(debughandler, 0)


  ns=mrb.getnode(0x10)
  nk=mrb.getnode(0x5C)
  openit=None



  def authreqhandler(p):
    if p.cmd==ord('O'):
      print 'authreq:', p

      id, pin = p.data[2], PinLenDecode(sum(p.data[3+i]<<(8*i) for i in xrange(4)))
      if pintest(id, pin):
        nk.sendpkt(['o', p.data[0], 1, 100])
        openit=p.data[1]
      else:
        nk.sendpkt(['o', p.data[0], 0, 100])

      return True #eat packet
    return False #dont eat packet
  nk.install(authreqhandler, 0)


  mrb.pumpout()
#  n.sendpkt()
  while 1:
    mrb.pump()
    if openit:
      print 'c'
      ns.doUntilReply(['C', 100, 1], rep=None, delay=.3, timeout=2)
      print 'd'
      openit=None

#    print n.gettypefilteredpktdata('S', 3)


#5c->* O [uinq], 10, id,pin4enc

#FE->5c o uinq,0-fail, 1-success, 100

#FE->10 ['C', 100, 1]



if __name__ == '__main__':
  PINS = readPins('../door_terminal2/userpins.h')
  run(lambda id,pin: id in PINS and PINS[id] == pin)
