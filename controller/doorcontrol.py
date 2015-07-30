#!/bin/python -tt

import sys
import argparse
import os
import struct
import datetime
import serial
import fcntl, termios
from daemon import runner
import logging

sys.path.insert(0, '/home/mark/hackerspace_access_control')
from pymrbus import mrbus


def readPins(fn):
  import re
  p={}
  with open(fn, 'r') as f:
    for l in f:
      if l[-1]=='\n':
        l=l[:-1]
      if not l.strip():
        continue
      if l.strip()=='};':
        continue
      if l.strip()[-1]=='{':
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
      logger.warn('pin import from %s dropped line "%s"'%(fn, l))
  return p

def PinLenDecode(pin):
  l=9
  while l and not(pin&1):
    l-=1
    pin>>=1
  pin>>=1;
  return ('%%0%uu'%l)%pin


def run(pintest):
  tty='/dev/ttyUSB0'
  try:
    ser = serial.Serial(tty, 115200, rtscts=True)
    try:
      fcntl.ioctl(ser.fileno(), termios.TIOCEXCL)
      fcntl.lockf(ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except IOError:
      logger.error('Serial port {0} is busy'.format(tty))
      return
  except serial.SerialException as ex:
    logger.error('Port {0} is unavailable: {1}'.format(tty, ex))
    return

  mrb = mrbus.mrbus(ser, addr=0xfe)#, logall=True, logfile=sys.stdout, extra=True)

  def debughandler(p):
    if p.cmd==ord('*'):
      logger.debug('debug pkt:'+str(p))
      return True #eat packet
    return False #dont eat packet
  mrb.install(debughandler, 0)


  ns=mrb.getnode(0x10)
  nk=mrb.getnode(0x5C)



  def authreqhandler(p):
    if p.cmd==ord('O'):

      uniq, strike, id, pin =  p.data[0], p.data[1], p.data[2], PinLenDecode(sum(p.data[3+i]<<(8*i) for i in xrange(4)))
       
      if pintest(id, pin):
        #ignores requested strike...
        logger.info('ID: %02u Unlock'%id)
        nk.sendpkt(['o', uniq, 1, 100])
        ns.sendpkt(['C', 100, 1])

      else:
        logger.info('ID: %02u Fail'%id)
        nk.sendpkt(['o', uniq, 0, 100])
        #cache the result for a while and by addr/uniq
        #set up rety handlers
        #make doorterm.c monitor the strike status and flip to open (and stop the O send) state, also end open state when door done.

      return True #eat packet
    return False #dont eat packet


  def strikeStatushandler(p, staticarr=[None]):
    if p.cmd==ord('S'):
      dooropen, dooropenhold, booted = (p.data[0]&(1<<i)!=0 for i in xrange(3))
      unlocked = p.data[1]!=0
      if (dooropenhold and not dooropen) or booted:
        ns.sendpkt(['Z'])
      if (unlocked, dooropen, dooropenhold, booted) != staticarr[0]:
        s='door status change:'
        if unlocked:
          s+='unlocked, '
        if dooropen:
          s+='door open, '
        if dooropenhold:
          s+='door was opened, '
        if booted:
          s+='strike rebooted'
        logger.info(s)
        staticarr[0] = (unlocked, dooropen, dooropenhold, booted)
      return True #eat packet
    return False #dont eat packet



  mrb.pumpout()
  nk.install(authreqhandler, 0)
  ns.install(strikeStatushandler, 0)
  logger.info('start')
  while 1:
    mrb.pump()
  logger.info('exit')
 

class App():
  def __init__(self):
    self.stdin_path = '/dev/null'
    self.stdout_path = '/dev/null'
    self.stderr_path = '/dev/null'
    self.pidfile_path =  '/var/run/doorcontrol.pid'
    self.pidfile_timeout = 5
  def run(self):
    run(lambda id,pin: id in PINS and PINS[id] == pin)

if __name__ == '__main__':
  app = App()

  logger = logging.getLogger("doorcontrol")
  logger.setLevel(logging.INFO)
  formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
  handler = logging.FileHandler("/var/log/doorcontrol.log")
  handler.setFormatter(formatter)
  logger.addHandler(handler)

  PINS = readPins('/home/mark/hackerspace_access_control/door_terminal2/userpins.h')

  daemon_runner = runner.DaemonRunner(app)
  #This ensures that the logger file handle does not get closed during daemonization
  daemon_runner.daemon_context.files_preserve=[handler.stream]
  daemon_runner.do_action()

  daemon_runner.do_action()


