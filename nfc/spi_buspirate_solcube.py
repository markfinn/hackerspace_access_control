#!/usr/bin/env python
# encoding: utf-8

import sys
#sys.path.insert(0, __file__+'/pyBusPirateLite')
sys.path.insert(0, './pyBusPirateLite')
from pyBusPirateLite.SPI import *



class cr95hf(object):
  def __init__(self, spi):
    self.spi=spi
    assert spi.BBmode()
    assert spi.enter_SPI()
    assert spi.cfg_pins(PinCfg.AUX)
    time.sleep(.015)
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.015)
    assert spi.cfg_pins(PinCfg.POWER)
    time.sleep(.015)
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.015)
    assert spi.set_speed(SPISpeed._250KHZ)
    assert spi.cfg_spi(0 | SPICfg.CLK_EDGE | SPICfg.OUT_TYPE)
    self.spi.CS_Low()
    self.bulk_trans(1)
    spi.CS_High()
    time.sleep(.015)
    assert spi.cfg_pins(PinCfg.POWER)
    time.sleep(.015)
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.015)

  def bulk_trans(self, *args):
    print 'send', args
    r  = self.spi.bulk_trans(len(args), args)
    r = [ord(x) for x in r]
    print 'returned', r
    return r

  def commandresp(self, c, opt=[]):
    self.spi.CS_Low()
    status = self.bulk_trans(3, 3)[1]
#    while status&8:
#      spi.CS_High()
#      time.sleep(.001)
#      self.spi.CS_Low()
#      self.bulk_trans(2, 0)
#      spi.CS_High()
#      time.sleep(.001)
#      self.spi.CS_Low()
#      status = self.bulk_trans(3, 3)[1]
    while not status&4:
      status = self.bulk_trans(3)[0]
    spi.CS_High()
    time.sleep(.001)

    dat = [0, c, len(opt)] + opt
#    dat=[0x55]
    self.spi.CS_Low()
    self.bulk_trans(*dat)
    self.spi.CS_High()
    time.sleep(.001)

    self.spi.CS_Low()
    status = self.bulk_trans(3,3)[1]
    while not status&8:
      status = self.bulk_trans(3)[0]
    self.spi.CS_High()
    time.sleep(.001)

    self.spi.CS_Low()
    resp = self.bulk_trans(2, 0, 0)[1:]
    resp2=[]
    if resp[1]:
      resp2 += self.bulk_trans(*([0]*resp[1]))
    self.spi.CS_High()
    time.sleep(.001)

    return resp[0], resp2


if __name__ == '__main__':
  spi = SPI("/dev/ttyUSB0", 115200)
  c = cr95hf(spi)
#  print c.commandresp(1, [])
  print c.commandresp(2, [2,0])
  print c.commandresp(4, [0x26, 0x07])

  spi.resetBP()

