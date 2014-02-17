import serial
import time
import sys
import mrbus
import intelhex
from Crypto.Cipher import AES

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



if __name__ == '__main__':
  mrb = mrbus.mrbus('/dev/ttyUSB0', addr=0xfe)#, logall=True, logfile=sys.stderr)

  def debughandler(p):
    if p.cmd==ord('*'):
      print 'debug:', p
      return True #eat packet
    return False #dont eat packet
  mrb.install(debughandler, 0)

  nodes = mrb.scannodes(pkttype='!')
  if len(nodes) == 0:
    print 'no node found in bootloader node. improve this program to catch it on start up.'
    sys.exit(1)
  if len(nodes) > 1:
    print 'found more than one bootloader node. improve this program to deal with it.'
    sys.exit(1)

  dest = nodes[0]
  node = mrb.getnode(dest)

  ih = intelhex.IntelHex(sys.argv[1])
  print ih.minaddr()
  print ih.maxaddr()
  prog=[ih[ii] for ii in xrange(ih.maxaddr()+1)]
  if len(prog) > 0x7000-18:
    print 'too long'
    sys.exit(1)
  prog=prog+([0xff]*(0x7000-18 - len(prog)))+[ord(s) for s in sign(prog)]+[len(prog)&0xff, (len(prog)>>8)&0xff]
  print len(prog)
  bootload(node, prog)


