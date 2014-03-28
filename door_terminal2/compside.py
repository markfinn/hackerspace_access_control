import sys
from Crypto.Cipher import AES
import argparse
import os
import aes_eax
import threading


sys.path.insert(0, '../../mrbus_bootloader')
import mrbus


    
def strfrombytes(b):
  s=''
  for bb in b:
    s+=str(chr(bb))
  return s


def intargparse(arg):
  if arg==None:
    return arg
  elif arg.startswith('0x') or arg.startswith('0X'):
    return int(arg[2:], 16)
  else:
    return int(arg)










class RDP(object):
  RDP_SEQ_N=16
  class RDP_Connection():
    #STATE_CLOSED     = 0  not used here. handled as connection dictionary entry for that address being None
    #STATE_LISTEN     = 1  not used here. handled in protocol, not connection
    STATE_SYN_SENT   = 2
    STATE_SYN_RCVD   = 3
    STATE_OPEN       = 4
    STATE_CLOSE_WAIT = 5

    def __init__(self, active=False):
      if active:
        self.state = self.STATE_SYN_SENT
      else:
        self.state = self.STATE_SYN_RCVD
    
  def __init__(self, node):

    def _handler(p):
      if p.cmd >= 0x80 and p.cmd < 0x80+self.RDP_SEQ_N+1:
        print p
        return True #eat packet
    def runner():
      while self.run:
        self.node.pump(1)


    self.node = node
    self.connections = {}
    self.hint=node.install(_handler)
    self.listening=False

    self.run=True
    self.thread = threading.Thread(group=None, target=runner, name='RDPPump')
    self.thread.start()
    self.sendbuf=[]

  def __dell__(self):
    self.node.remove(self.hint)
    self.run=False
    self.thread.join()


  def stop(self):
    self.run=False
    
  def listen(self):
    self.listening=True
    
  def open(self, dest):
    assert dest not in self.connections
    self.connections[dest] = self.RDP_Connection(True)
    self.node.sendpkt([self.RDP_SEQ_N-128, 0])

  def send(self, channel, data):
    assert channel < 16
    assert len(data) <= 0xfff
    self.sendbuf += [(channel<<12) | (len(data)>>8), len(data)&0xff] + data
    


class doorterm(RDP):
  def __init__(self, node):
    super(doorterm, self).__init__(node)

  def putScreen(self, s):
    self.send(1, [ord(c) for c in s])



if __name__ == '__main__':
  key='yourkeygoeshere\x00'
  parser = argparse.ArgumentParser(description='nfc door term prog')
  parser.add_argument('-p', '--port', type=str,help='port for mrbus CI2 interface. Will guess /dev/ttyUSB? if not specified')
  parser.add_argument('-a', '--addr-host', help='mrbus address to use for host.  Will scan for an unused address if not specified')
  parser.add_argument('-d', '--addr', default=None, help='mrbus address of node to program.  Will scan for a singular node in bootloader mode if not specified')
  args = parser.parse_args()

  args.addr_host = intargparse(args.addr_host)
  args.addr = intargparse(args.addr)


  if args.port == None:
    args.port = [d for d in os.listdir('/dev/') if d.startswith('ttyUSB')]
    if len(args.port) == 0:
      print 'no port specified, and can\'t find a default one'
      sys.exit(1)
    elif len(args.port) > 1:
      print 'no port specified, and there is more than one to guess from.  giving up.'
      sys.exit(1)
    args.port='/dev/'+args.port[0]
  
  mrb = mrbus.mrbus(args.port, addr=args.addr_host, logall=True, logfile=sys.stdout, extra=True)

  def debughandler(p):
    if p.cmd==ord('*'):
      print 'debug:', p
      return True #eat packet
    return False #dont eat packet
  mrb.install(debughandler, 0)


  if args.addr == None:
    nodes = mrb.scannodes(pkttype='V')
    nodes = [n.src for n in nodes if ''.join(map(chr, n.data[:7]))=='NfcDoor']
    if len(nodes) == 0:
      print 'no node found'
      sys.exit(1)
    if len(nodes) > 1:
      print 'found more than one node found. specify an address.'
      sys.exit(1)
    args.addr = nodes[0]

  node = mrb.getnode(args.addr)

  dt=doorterm(node)
  try:
    dt.open(2)
    dt.putScreen('hi')

    while 1:
      pass
  except:
    pass
  dt.stop()

#  print node.cmp.isSupported(timeout=200)



#############################
#AES TEST
#  t=[1]
#  enc = AES.new(key, AES.MODE_CBC, '\x00'*16)
#  print map(ord, enc.encrypt(strfrombytes(t + [0]*(16-len(t)))))

#  node.pumpout()
#  node.sendpkt(['Z']+t)
#  #print node.getfilteredpkt(lambda p: p.cmd==ord('z')).data
#  print node.gettypefilteredpktdata('z')
#############################


#############################
#CMAC TEST
#  t=283945720348972302934857
#  tag = aes_eax.OMAC(aes_eax.intfromstr(key), t, 14)
#  print hex(tag)

#  node.pumpout()
#  node.sendpkt(['1']+[s for s in aes_eax.strfromint(t,14)])
#  r = node.gettypefilteredpktdata('2')
#  print map(hex, r)
#############################


#############################
#EAX enc TEST
#
#  nonce=[1]
#  nl=len(nonce)
#  head=[1,2]
#  hl=len(head)
#  data=[2,3,4]
#  dl=len(data)
#
#  assert nl<16 and hl < 16 and dl < 16 and nl+hl+dl <= 13
#
#  print hex(aes_eax.intfrombytes(data))
#
#  node.pumpout()
#  node.sendpkt(['3']+[(hl<<4)|nl]+nonce+head+data)
#  r = node.gettypefilteredpktdata('4')
#  print map(hex, r)
#  print hex(aes_eax.aead_eax_aes_dec(aes_eax.intfromstr(key), aes_eax.intfrombytes(nonce), nl, aes_eax.intfrombytes(head), hl, aes_eax.intfrombytes(r), len(r), 3))
#
#############################





