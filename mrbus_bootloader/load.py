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
      print >> sys.stderr, 'failed to find node %02xh with command %02xh'%(node.addr, pkttype)
      return None
    if replycount > count:
      print >> sys.stderr, 'too many replies to find node %02xh with command %02xh. This might be due to bus dups, or there might be two nodes with the same address.  I\'m not risking it. Dying.'%(node.addr, ord(pkttype))
      return None
    if len(reply) > 1:
      print >> sys.stderr, 'too many unique replies to find node %02xh with command %02xh. This almost certainly means there are two nodes with the same address.  Dying.'%(node.addr, ord(pkttype))
      return None
    return reply.pop()

  #make sure the node replies, but only once.
  if None == pollnode(node, 'A'): sys.exit(1)
  loaderstatus = pollnode(node, '!')
  if None == loaderstatus: sys.exit(1)
  loaderversion = pollnode(node, 'V')
  if None == loaderversion: sys.exit(1)

  #OK, we seem to be safe
  print ih.minaddr()
  print ih.maxaddr()
  print loaderstatus
  print loaderversion


  

#  node.sendpkt(['V'])
#  p=node.getpkt(timeout=2)


def readhex(fname):
  pass

if __name__ == '__main__':
  mrb = mrbus.mrbus('/dev/ttyUSB0')#, logall=True, logfile=sys.stderr)
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


