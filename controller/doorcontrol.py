#!/bin/python -tt

import sys
import argparse
import os
import struct
import datetime

sys.path.insert(0, '../pymrbus')
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



  def authreqhandler(p):
    if p.cmd==ord('O'):
#      print 'authreq:', p

      uniq, strike, id, pin =  p.data[0], p.data[1], p.data[2], PinLenDecode(sum(p.data[3+i]<<(8*i) for i in xrange(4)))
      tm = datetime.datetime.now()
       
      if pintest(id, pin):
        #ignores requested strike...
        print '%s ID: %02u Unlock'%(tm.isoformat(sep=' '), id)
        sys.stdout.flush()
        nk.sendpkt(['o', uniq, 1, 100])
        ns.sendpkt(['C', 100, 1])

      else:
        print '%s ID: %02u Fail'%(tm.isoformat(sep=' '), id)
        sys.stdout.flush()
        nk.sendpkt(['o', uniq, 0, 100])
        #cache the result for a while and by addr/uniq
        #set up rety handlers
        #make doorterm.c monitor the strike status and flip to open (and stop the O send) state, also end open state when door done.

      return True #eat packet
    return False #dont eat packet


  def strikeStatushandler(p, staticarr=[None]):
    if p.cmd==ord('S'):
      tm = datetime.datetime.now()
      dooropen, dooropenhold, booted = (p.data[0]&(1<<i)!=0 for i in xrange(3))
      unlocked = p.data[1]!=0
      if (dooropenhold and not dooropen) or booted:
        ns.sendpkt(['Z'])
      if (unlocked, dooropen, dooropenhold, booted) != staticarr[0]:
        s='%s door status change:'%(tm.isoformat(sep=' '))
        if unlocked:
          s+='unlocked, '
        if dooropen:
          s+='door open, '
        if dooropenhold:
          s+='door was opened, '
        if booted:
          s+='strike rebooted'
        print s
        sys.stdout.flush()
        staticarr[0] = (unlocked, dooropen, dooropenhold, booted)
      return True #eat packet
    return False #dont eat packet



  mrb.pumpout()
  nk.install(authreqhandler, 0)
  ns.install(strikeStatushandler, 0)
  tm = datetime.datetime.now()
  print '%s start'%(tm.isoformat(sep=' '))
  sys.stdout.flush()
  while 1:
    mrb.pump()
  tm = datetime.datetime.now()
  print '%s exit'%(tm.isoformat(sep=' '))
  sys.stdout.flush()
 
#5c->* O [uinq], 10, id,pin4enc
#FE->5c o uinq,0-fail, 1-success, 100
#FE->10 ['C', 100, 1]
if __name__ == '__main__':
  PINS = readPins('../door_terminal2/userpins.h')
  run(lambda id,pin: id in PINS and PINS[id] == pin)

