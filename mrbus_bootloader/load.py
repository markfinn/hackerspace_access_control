import serial
import time
import sys
import mrbus
import intelhex


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
  print ih.minaddr()
  print ih.maxaddr()
  print loaderstatus
  print loaderversion
  print loadersig

  pagesize=loaderversion.data[2]|(loaderversion.data[3]<<8)


  def writepage(pageaddr, data):
    def requirestatus():
      while True:
        p = node.getpkt(timeout=1)
        if p and p.cmd==ord('@'):
          print p
          return p.data[2]|(p.data[3]<<8)
        elif p==None:
          print 'fail',
          node.sendpkt(['!'])

  #Epp
  #e
    print 'sende', pageaddr
    node.sendpkt(['E', pageaddr, pageaddr>>8])
    requirestatus()

  #D[12]xs
  #@ if s
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
  #W
  #w
    print 'sendw'
    node.sendpkt(['W'])
    requirestatus()

  for page in xrange(ih.minaddr()//pagesize, (ih.maxaddr()+pagesize)//pagesize):
    writepage(page*pagesize, [ih[ii] for ii in xrange(page*pagesize, (page+1)*pagesize)])


def readhex(fname):
  pass

if __name__ == '__main__':
  mrb = mrbus.mrbus('/dev/ttyUSB0')#, logall=True, logfile=sys.stderr)

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
  bootload(node, ih) 


