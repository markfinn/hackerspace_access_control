#!/bin/python -tt

import sys
import argparse
import os
import struct

sys.path.insert(0, '../../mrbus_bootloader')
import mrbus


if __name__ == '__main__':


  mrb = mrbus.mrbus('/dev/ttyUSB0', addr=0xfe, logall=True, logfile=sys.stdout, extra=True)

  def debughandler(p):
    if p.cmd==ord('*'):
      print 'debug:', p
      return True #eat packet
    return False #dont eat packet
#  mrb.install(debughandler, 0)


  n=mrb.getnode(0x10)

  n.pumpout()
  n.sendpkt(['C', 200])
#  print n.gettypefilteredpktdata('v')

