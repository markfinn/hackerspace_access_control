#!/usr/bin/env python
# encoding: utf-8

from __future__ import division
import sys
#sys.path.insert(0, __file__+'/pyBusPirateLite')
sys.path.insert(0, '../nfc/pyBusPirateLite')
from pyBusPirateLite.I2C import *

#buspirate:
#[0x78 0x0 0xae] [0x78 0 0xd5 0x80] [0x78 0 0x3f] [0x78 0 0xd3 0] [0x78 0 0x40] [0x78 0 0x8d 0x14] [0x78 0 0x20 0] [0x78 0 0xA1] [0x78 0 0xC8] [0x78 0 0xda 0x12] [0x78 0 0x81 0xcf] [0x78 0 0xd9 0xf1] [0x78 0 0xdb 0x40] [0x78 0 0xa4] [0x78 0 0xa6]
#[0x78 0 0xaf][0x78 0 0x21, 0, 127][0x78 0 0x22, 0,   7]
#[0x78 0x40 0:1024]

class ssd1306(object):
  def __init__(self, isc):
    self.i2c=i2c
    assert i2c.BBmode()
    assert i2c.enter_I2C()
    assert i2c.cfg_pins(PinCfg.POWER)
    assert i2c.set_speed(I2CSpeed._100KHZ)

    self.reset()


  def reset(self):
asdf


  def write(data):
    i2c.send_start_bit()
    i2c.bulk_trans(len(data),data)
    i2c.send_stop_bit()



if __name__ == '__main__':
  i2c = I2C("/dev/ttyUSB0", 115200)
  oled = ssd1306(spi)

  i2c.resetBP()


