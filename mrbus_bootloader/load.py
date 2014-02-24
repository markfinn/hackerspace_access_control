import serial
import time
import sys
import mrbus
import intelhex
from Crypto.Cipher import AES
import argparse
import os

def strfrombytes(b):
  s=''
  for bb in b:
    s+=str(chr(bb))
  return s
    
def sign(m, key='MRBusBootLoader\x00'):
  # length prepended cbc mac aes
  enc = AES.new(key, AES.MODE_CBC, strfrombytes([0]*16))

  l=len(m)
  enc.encrypt(strfrombytes([l&0xff, (l>>8)&0xff, (l>>16)&0xff, (l>>24)&0xff] + [0]*12))

  while len(m)>=16:
    out = enc.encrypt(strfrombytes(m[:16]))
    m=m[16:]

  if m:
    out = enc.encrypt(strfrombytes(m+([0]*(16-len(m)))))

  return out   
    
    
    
def bootload(node, prog):
  def pollnode(node, pkttype, count=3, wait=1):
    reply = set()
    replycount=0
    t=time.time()
    n=0
    while time.time()-t < wait+count*.3:
      x=(time.time()-t)/.3
      if n < count and x > n:
        node.sendpkt([pkttype])
        n+=1
      tn=time.time()
      to=min(wait+count*.3+t-tn, n*.3+t-tn)
      p = node.getpkt(timeout=to)
      if p:
        reply.add(p)
        replycount+=1

    if replycount == 0:
      print >> sys.stderr, 'failed to find node %02xh with command %02xh'%(node.addr, ord(pkttype))
      return None
    if replycount > count:
      print >> sys.stderr, 'too many replies to find node %02xh with command %02xh. This might be due to bus dups, or there might be two nodes with the same address.  I\'m not risking it. Dying.'%(node.addr, ord(pkttype))
      return None
    if len(reply) > 1:
      print >> sys.stderr, 'too many unique replies to find node %02xh with command %02xh. This almost certainly means there are two nodes with the same address.  Dying.'%(node.addr, ord(pkttype))
      return None
    return reply.pop()


  #make sure the node replies, but only once.
  loaderstatus = pollnode(node, '!', wait=0)
  if None == loaderstatus: sys.exit(1)
  loaderversion = pollnode(node, 'V', wait=0)
  if None == loaderversion: sys.exit(1)
  loadersig = pollnode(node, 'S')
  if None == loadersig: sys.exit(1)

  #OK, we seem to be safe
  print loaderstatus
  print loaderversion
  print loadersig

  pagesize=loaderversion.data[2]|(loaderversion.data[3]<<8)


  def writepage(pageaddr, data):
    print'writepage', pageaddr
    def dountillreply(cmd, rep=None, to=5):
      if rep==None:
        rep=ord(cmd[0].lower())
      while True:
        node.sendpkt(cmd)
        p = node.getpkt(timeout=1)
        if p and p.cmd==rep:
          print p
          return
        elif p==None:
          print 'fail',
          to-=1
          if to == 0:
            print 'giving up'
            sys.exit(1)

    def requirestatus():
      while True:
        p = node.getpkt(timeout=1)
        if p and p.cmd==ord('@'):
          print p
          return p.data[2]|(p.data[3]<<8)
        elif p==None:
          print 'fail',
          node.sendpkt(['!'])

    def senddata(data):
      z=data[0]
      for x in data:
        if x!=z:
          break
      else:
        print 'sendf short', z
        dountillreply(['F', z])
        return

      print 'sendf'
      dountillreply(['F', 0])
          
      #D[12]xs
      #@ if s
      print 'sendd'
      tosend=set(xrange((pagesize+11)//12))
      while tosend:
        i=tosend.pop()
        stat=1 if len(tosend)==0 else 0
        d=[data[i*12+j] for j in xrange(12) if i*12+j < pagesize]
        d+=[0]*(12-len(d))
        node.sendpkt(['D']+d+[i, stat])
        if stat:
          p = node.getpkt(timeout=2)
          if p:
            print p
            tosend|=set((p.data[0]*8+k for k in xrange(8) if p.data[0]*8+k < (pagesize+11)//12 and p.data[1]&(1<<k)==0))
          else:
            tosend|=set([i])


    senddata(data)

  #W
  #w
    print 'sendw'
    dountillreply(['#', pageaddr, pageaddr>>8], rep=ord('$'))

  for page in xrange(0//pagesize, (len(prog)+pagesize-1)//pagesize):
    writepage(page*pagesize, prog[page*pagesize: (page+1)*pagesize])



def intargparse(arg):
  if arg==None:
    return arg
  elif arg.startswith('0x') or arg.startswith('0X'):
    return int(arg[2:], 16)
  else:
    return int(arg)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description='program an mrbus node via the bootloader')
  parser.add_argument('-p', '--port', type=str,help='port for mrbus CI2 interface. Will guess /dev/ttyUSB? if not specified')
  parser.add_argument('-a', '--addr-host', help='mrbus address to use for host.  Will scan for an unused address if not specified')
  parser.add_argument('-d', '--addr', default=None, help='mrbus address of node to program.  Will scan for a singular node in bootloader mode if not specified')
  parser.add_argument('-x', '--reset-to-bootloader', action='store_true', help='send the target node a reset (\'X\') command then to attempt to enter the bootloader. Implies -l')
  parser.add_argument('-l', '--listen-for-bootloader', action='store_true', help='wait for the node to send a bootloader-waiting packet, then halt the normal boot processs in bootloader mode')
  parser.add_argument('-r', '--reset-when-done', action='store_true', help='reset the target after we are finished')
#  parser.add_argument('-s', '--force-sign', action='store_true', help='sign the object even if it seems to have a signature')
#  parser.add_argument('-k', '--key-file', type=str, help='key file to use if signing with a proprietary shared key. reads the first 16 bytes from the file.')
  parser.add_argument('file', type=str,  help='file to load')
  args = parser.parse_args()

  args.addr_host = intargparse(args.addr_host)
  args.addr = intargparse(args.addr)

  if args.reset_to_bootloader and None == args.addr:
     print 'I need a dest address if you want me to reset something'
     print 'Well, I could use a ping scan after I figure out my own address, then if there is on;y one node, assume that\'s what you meant....  but no.'
     sys.exit(1)

  if args.port == None:
    args.port = [d for d in os.listdir('/dev/') if d.startswith('ttyUSB')]
    if len(args.port) == 0:
      print 'no port specified, and can\'t find a default one'
      sys.exit(1)
    elif len(args.port) > 1:
      print 'no port specified, and there is more than one to guess from.  giving up.'
      sys.exit(1)
    args.port='/dev/'+args.port[0]
  
  mrb = mrbus.mrbus(args.port, addr=args.addr_host, logall=True, logfile=sys.stderr)

  def debughandler(p):
    if p.cmd==ord('*'):
      print 'debug:', p
      return True #eat packet
    return False #dont eat packet
  mrb.install(debughandler, 0)


  if args.addr == None:
    nodes = mrb.scannodes(pkttype='!', rettype=0x40)
    if len(nodes) == 0:
      print 'no node found in bootloader mode.'
      sys.exit(1)
    if len(nodes) > 1:
      print 'found more than one node in bootloader mode. specify an address.'
      sys.exit(1)
    args.addr = nodes[0]



  print 'loading to node 0x%02X'%args.addr
  node = mrb.getnode(args.addr)



  if args.reset_to_bootloader:
    args.listen_for_bootloader=True
    print 'sending reset to get in bootloader mode'
    node.sendpkt(['X'])

  if args.listen_for_bootloader:
    print 'waiting for bootloader announce'
    t=time.time()
    p=None
    while time.time()-t < 100:
      tn=time.time()
      p = node.getpkt(timeout=100+t-time.time())
      if p and p.cmd==0x40:
        break
    else:
      print 'didn\'t see the node come up in bootloader mode'
      sys.exit(1)    



  ih = intelhex.IntelHex(args.file)
  print ih.minaddr(), ih.maxaddr()
  prog=[ih[ii] for ii in xrange(ih.maxaddr()+1)]
  if len(prog) > 0x7000-18:
    print 'program too long'
    sys.exit(1)

  prog=prog+([0xff]*(0x7000-18 - len(prog)))+[ord(s) for s in sign(prog)]+[len(prog)&0xff, (len(prog)>>8)&0xff]

  bootload(node, prog)

  node.sendpkt(['S'])
  while True:
    p = node.getpkt(timeout=1)
    if p and p.cmd==ord('s'):
      if p.data[0] != 0x21 or p.data[1] != 0:
        print 'signature doesn\'t verify.'
        print p
        sys.exit(1)
      break
    elif p==None:
      node.sendpkt(['S'])
  else:
    print 'cant get sig at end'
    sys.exit(1)    


  if args.reset_when_done:
    node.sendpkt(['X'])


