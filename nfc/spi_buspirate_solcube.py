#!/usr/bin/env python
# encoding: utf-8

from __future__ import division
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
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.10)
    assert spi.cfg_pins(PinCfg.POWER)
    time.sleep(.010)
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.010)
    assert spi.set_speed(SPISpeed._1MHZ)
    assert spi.cfg_spi(0 | SPICfg.CLK_EDGE | SPICfg.OUT_TYPE)

    self.reset()


  def reset(self):
    self.spi.CS_Low()
    self.bulk_trans(1)
    spi.CS_High()
    time.sleep(.01)
    assert spi.cfg_pins(PinCfg.POWER)
    time.sleep(.01)
    assert spi.cfg_pins(PinCfg.POWER | PinCfg.AUX)
    time.sleep(.01)

  @staticmethod
  def getfsdi():
    return 0x5
    #the maximum 14443-4 frame length this reader can support. 5 means 64 bytes.  This is poorly spec'ed in the data sheert, but the examples use 5.

  def bulk_trans(self, *args):
#    print 'send', args
    r=[]
    while args:
      a=args[:min(3, len(args))]
      args=args[len(a):]
      r  += self.spi.bulk_trans(len(a), a)

    r = [ord(x) for x in r]
#    print 'returned', r
    return r

  def commandresp(self, c, opt=[]):
    self.spi.CS_Low()
    status = self.bulk_trans(3, 3)[1]
    assert status&8 == 0
    while not status&4:
      status = self.bulk_trans(3)[0]
    spi.CS_High()
    time.sleep(.001)

    dat = [0, c, len(opt)] + opt
    self.spi.CS_Low()
    self.bulk_trans(*dat)
    self.spi.CS_High()
    self.spi.CS_Low()
    status = self.bulk_trans(3,3)[1]
    while not status&8:
      status = self.bulk_trans(3)[0]
    self.spi.CS_High()
    self.spi.CS_Low()
    resp = self.bulk_trans(2, 0, 0)[1:]
    resp2=[]
    if resp[1]:
      resp2 += self.bulk_trans(*([0]*resp[1]))
    self.spi.CS_High()
    return resp[0], resp2

  def protocolSelectOff(self):
    self.commandresp(2, [0,0])

  def protocolsetup14443A(self, tRate=0, rRate=0, frameWaitTime_uS=100):# I dont know from the spec what frameWaitTime_uS should be before rats gives me the right answer. 86 seems to work, but I'll go long for now
    x = (frameWaitTime_uS * 13.56) // 4096
    for pp in xrange(0x0f):
      mm = x//(2**pp)-1
      if mm<=0xfe:
        break
    else:
      assert 0
    mm=max(1, int(mm))
    res = self.commandresp(2, [2,((tRate&3)<<6)|((rRate&3)<<4), pp, mm])
    assert res == (0,[])


  def protocolSelect14443A(self, tRate=0, rRate=0, frameWaitTime_uS=100):
    self.protocolsetup14443A(tRate, rRate, frameWaitTime_uS)
    return iso14443A(self)


  def comTag(self, data, bits=0, doCrc=1, topaz=0, splitFrame=0, noparity=0, drop2forcrc=1):
    if bits==0:
      bits=len(data)*8
    assert (bits+7)//8 == len(data)
    print '>>>', [hex(a) for a in data], bits
    res = self.commandresp(4, data+[((topaz&1)<<7)|((splitFrame&1)<<6)|((doCrc&1)<<5)|((noparity&1)<<4)|((bits-1)%8+1)])
    assert res[0] == 0x80
    print '<<<', [hex(a) for a in res[1][:-3]], [hex(a) for a in res[1][-3:]]
    if drop2forcrc:
      return res[1][:-5], tuple(res[1][-3:])
    else:
      return res[1][:-3], tuple(res[1][-3:])

class iso14443A(object):
  def __init__(self, reader):
    self.reader=reader

  def comTag_anticol(self, cascade=1, bits=0, uuid=[0]*5):
    assert cascade >= 1 and cascade <=3
    bytes = bits//8
    r,x = self.reader.comTag([0x91+cascade*2, ((bytes+2) << 4)|(bits&7)]+uuid[:(bits+7)//8], bits=bits+16, doCrc=0, splitFrame=0 if bits%8==0 else 1, drop2forcrc=0)
    if x[0]&0x80:
      #collision
      assert x[2]&0xf<8
      return r, x[1]*8+(x[2]&0xf), x[0]&0xf
    assert x[0]&0x10 == 0
    return r, -1, x[0]&0xf

  def transcieve(self, bytes):
    return self.reader.comTag(bytes)

  def comTag_reqa(self):
    "wake up from idle. return atqa, which should be ignored due to possible collisions and no crc"
    return self.reader.comTag([0x26], bits=7, doCrc=0, drop2forcrc=0)

  def comTag_wupa(self):
    "wake up from idle or halt. return atqa, which should be ignored due to possible collisions and no crc"
    return self.reader.comTag([0x52], bits=7, doCrc=0, drop2forcrc=0)

  def comTag_halt(self):
    "idle halt a selected tag"
    try:#seems to not reply
      r, x = self.reader.comTag([0x50, 0])
    except:
      pass

  def comTag_select(self, uuid, cascade=1):
    assert cascade >= 1 and cascade <=3
    r,x = self.reader.comTag([0x91+cascade*2, 0x70]+uuid)
    assert x[0]&0xB0 == 0      #collision or error
    return r

  def selectOneCascade(self, cascade):
    def addbits(dest, src, bits):
      s=bits//8
      while bits < len(dest)*8:
    #    print bits, bits//8, 1<<(bits%8), bits//8 - s
        dest[bits//8] = (dest[bits//8]&~(1<<(bits%8))) | src[bits//8 - s]&(1<<(bits%8))
        bits+=1

    uuid=[0]*5
    bits=0

    while bits < 40:
      data, col, b = self.comTag_anticol(cascade=cascade, bits=bits, uuid=uuid)
      #print [hex(a) for a in data], col, b
      addbits(uuid, data, bits)
      if col>=0:
        bits += col+1
      else:
        bits += (len(data)-1)*8 + b
      assert uuid[0]^uuid[1]^uuid[2]^uuid[3]^uuid[4]==0
    sak=self.comTag_select(uuid, cascade=cascade)

    return uuid, sak[0]

  def selectOne(self):
    uuid=[]
    cascade=1
    while True:
      u, sak = self.selectOneCascade(cascade)
      if sak&0x4 == 0:
        uuid.extend(u[:-1])
        return uuid, sak
      assert u[0]==0x88
      uuid.extend(u[1:-1])
      cascade+=1


  def findAll(self):
    found = []
    while True:
      try:
        self.comTag_reqa()
      except:
        break
      try:
        uuid, sak = self.selectOne()
        print 'uuid:', [hex(a) for a in uuid]
        print 'sak:', hex(sak)
        found.append((uuid, sak))
        self.comTag_halt()
      except:
        pass



class iso14443_4(object):
  #doesn't do cid or nad
  def __init__(self, tag):
    self.tag=tag
    self.blockno=0

    #do rats
    cid=0
    fsdi = tag.reader.getfsdi()
    r, x = self.tag.transcieve([0xe0, (fsdi<<4)|cid])
    assert x[0]&0xB0 == 0      #collision or error
    assert r[0] == len(r)
    r = r[1:]
    self.rats = r
    r = r[1:]


    if self.rats[0]&0x10:
      self.rats_TA=r[0]
      r=r[1:]
    else:
      self.rats_TA=0

    if self.rats[0]&0x20:
      self.rats_TB=r[0]
      r=r[1:]
    else:
      self.rats_TB=0x40

    if self.rats[0]&0x40:
      self.rats_TC=r[0]
      r=r[1:]
    else:
      self.rats_TC=0x02

    self.rats_historicalbytes=r

    self.fsci = r[0]&0xf
    if self.fsci > 8:
      self.fsc = 256
    else:
      self.fsc = [16, 24, 32, 40, 48, 64, 96, 128, 256][self.fsci]

    self.fwi = self.rats_TB>>4
    self.fwt = 256*16/13.56*(2**self.fwi)
#  set fwt.
    self.tag.reader.protocolsetup14443A(frameWaitTime_uS=self.fwt)

    self.sfgi=self.rats_TB&0xf
    self.sfgt = 256*16/13.56*(2**self.sfgi)
#wait sfgt
    time.sleep(self.sfgt/1000000)


    


  def transcieve(self, bytes):
    #iblock
    r,x = self.tag.transcieve([0x2+self.blockno]+bytes)
    self.blockno=(1+self.blockno)%2
    assert x[0]&0xB0 == 0      #collision or error



#    send
#
#    while True:
#    #recv
#    check block n stuff?
#    if recvok:
#      if s fwt:
#        send s fwt 
#        change wait. unchange when?
#      elif s dsel:
#        done
#      elif rack
#        if it matches current block num
#          inc blockno
#          send next
#        else:
#          send last again
#      else:
#        add to buf
#        inc blockno
#        if chain:
#          ack
#        else:
#          return
#    else:#recv err
#      if input chaning:
#        send rnack
#        until when?
#      elif just sent a dselect:
#        send dselect
#        until when?
#      else
#        send rnack

#TODO need to keep track of picc block num, and test it for proto error
#TODO need to apply the "error revovery" rules
        

  
  def printats(self):
      rats0 = self.rats[0]
      rats = self.rats[1:]
      print 'max frame size:', [16, 24, 32, 40, 48, 64, 96, 128, 256][rats0&0xf]
      if rats0&0x10:
        bitrates=rats[0]
        rats = rats[1:]
        print "PCD to PICC bit rates: 106",
        for i,x in enumerate([212, 424, 847]):
          if bitrates&(1<<i):
            print ', ', x,
        print
        print "PICC to PCD bit rates: 106",
        for i,x in enumerate([212, 424, 847]):
          if bitrates&(16<<i):
            print ', ', x,
        print
        if bitrates&0x80:
          print 'must be symetrical'
        else:
          print 'asymetrical OK.'
      if rats0&0x20:
        waits=rats[0]
        rats = rats[1:]
        print 'frame waiting time:'
      print 'more rats decode here' 

    
if __name__ == '__main__':
  spi = SPI("/dev/ttyUSB0", 115200)
  chip = cr95hf(spi)
#  print c.commandresp(1, [])

  tech = chip.protocolSelect14443A()
  
#  tech.findAll()
#  sys.exit()

  while True:
    try:
      tech.comTag_reqa()
      break
    except:
      pass

  uuid, sak = tech.selectOne()
  print 'uuid:', [hex(a) for a in uuid]
  print 'sak:', hex(sak)
  if sak & 0x40:
    print 'iso 18092'
  if sak & 0x20:
    print 'iso 14443-4'

  if sak & 0x20:
    tag = iso14443_4(tech)


  tag.transcieve([0x60])
#  print c.comTag14443A([0x3, 0xaf])
#  print c.comTag14443A([0x2, 0xaf])

#    self.comTag_halt()

  spi.resetBP()




#todo #  set fwt

