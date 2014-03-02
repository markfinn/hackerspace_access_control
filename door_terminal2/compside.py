import sys
from Crypto.Cipher import AES
import argparse
import os
import aes_eax



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
  
  mrb = mrbus.mrbus(args.port, addr=args.addr_host)#, logall=True, logfile=sys.stdout, extra=True)

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



  print node.cmp.isSupported(timeout=200)

#  t=[1]
#  enc = AES.new(key, AES.MODE_CBC, '\x00'*16)
#  print map(ord, enc.encrypt(strfrombytes(t + [0]*(16-len(t)))))

#  node.pumpout()
#  node.sendpkt(['Z']+t)
#  #print node.getfilteredpkt(lambda p: p.cmd==ord('z')).data
#  print node.gettypefilteredpktdata('z')


  t=283945720348972302934857
  tag = aes_eax.OMAC(aes_eax.intfromstr(key), t, 14)
  print hex(tag)

  node.pumpout()
  node.sendpkt(['1']+[s for s in aes_eax.strfromint(t,14)])
  r = node.gettypefilteredpktdata('2')
  print map(hex, r)



