import serial
import time
import sys
import mrbus
import intelhex
from Crypto.Cipher import AES
import argparse
import os
from collections import namedtuple

def strfrombytes(b):
  s=''
  for bb in b:
    s+=str(chr(bb))
  return s


def progload(f, maxsize=None):
  ih = intelhex.IntelHex(f)
  s=ih.maxaddr()+1
  if maxsize:
    s=min(maxsize, s)
  return [ih[ii] for ii in xrange(s)]


    
def sign(m, key):
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
    
    
    
def bootloadseek(node):
  def pollnode(node, pkttype, count=3, wait=1, delay=.2):
    reply = set()
    replycount=0
    t=time.time()
    n=0
    while time.time()-t < wait+count*delay:
      x=(time.time()-t)/delay
      if n < count and x > n:
        node.sendpkt([pkttype])
        n+=1
      tn=time.time()
      to=min(wait+count*delay+t-tn, n*delay+t-tn)
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
  loaderstatus = pollnode(node, '!', count=1, wait=0)
  if None == loaderstatus: sys.exit(1)
  loaderversion = pollnode(node, 'V', count=1, wait=0)
  if None == loaderversion: sys.exit(1)
  loadersig = pollnode(node, 'S', count=3, wait=.5, delay=.1)
  if None == loadersig: sys.exit(1)

  if loaderversion.data[0] != 0x21:
    print >> sys.stderr, 'version weirdness..  Dying.'
    sys.exit(1)
  version=loaderversion.data[1]
  if version <2:
    print >> sys.stderr, 'cowardly refusing to try to work with an unstable prerelease version of the bootloder API..  Dying.'
    sys.exit(1)
  pagesize=loaderversion.data[2]|(loaderversion.data[3]<<8)
  bootstart=loaderversion.data[4]|(loaderversion.data[5]<<8)
  avrsig=loaderversion.data[6:9]


  appsigok=loadersig.data[0]&0x80 == 0
  appfffill=loadersig.data[0]&0x10 == 0
  appsize=loadersig.data[1]|(loadersig.data[2]<<8)
  appclaimedsig=loadersig.data[3:3+8]

  App = namedtuple('BootloaderApp', ['sigok', 'fffill', 'size', 'claimedsig'])
  a = App(appsigok, appfffill, appsize, appclaimedsig)

  Client = namedtuple('BootloaderClient', ['node', 'rawloaderstatus', 'rawloaderversion', 'rawloadersig', 'version', 'pagesize', 'bootstart', 'avrsig', 'app', 'currentimg'])
  c=Client(node, loaderstatus, loaderversion, loadersig, version, pagesize, bootstart, avrsig, a, None)

  return c
  


def writepage(c, pageaddr, data, needStatusReset, prevdata=None):
  #print'writepage', pageaddr
  
  def dountillreply(cmd, rep=None, to=5):
    #print 'dountillreply', cmd, rep, to
    if rep==None:
      rep=ord(cmd[0].lower())
    c.node.sendpkt(cmd)
    start=time.time()
    sent=1
    now=start
    while True:
      p = c.node.getpkt(timeout=max(0, start+min(sent, to)-now))
      now=time.time()
      if p and p.cmd==rep:
        #print p
        return p
      elif start+to <= now:
        print 'giving up'
        sys.exit(1)
      elif start+sent-now > 0:
        print 'timeout->retry ',
        c.node.sendpkt(cmd)
        sent+=1


  def senddata(data):
    z=data[0]
    for x in data:
      if x!=z:
        break
    else:
      if z==0xff:
        print '-',
      else:
        print 'c',
      dountillreply(['F', z])
      return

    if needStatusReset:
      dountillreply(['F', 0])
        
    print 'd',
    tosend=set(xrange((c.pagesize+11)//12))
    while tosend:
      i=tosend.pop()
      stat=1 if len(tosend)==0 else 0
      d=[data[i*12+j] for j in xrange(12) if i*12+j < c.pagesize]
      d+=[0]*(12-len(d))
      c.node.sendpkt(['D']+d+[i, stat])
      if stat:
        p = c.node.getpkt(timeout=2)
        if p:
          failed=set((p.data[0]*8+k for k in xrange(8) if p.data[0]*8+k < (c.pagesize+11)//12 and p.data[1]&(1<<k)==0))
          tosend|=failed
          if failed:
            print 'failed, retry:', failed
        else:
          tosend|=set([i])

  if data == c.currentimg[pageaddr: pageaddr+c.pagesize]:
    print '*',
    return

  if prevdata==data:
    print 'r',
#  elif :
  else:
    senddata(data)

  dountillreply(['#', pageaddr, pageaddr>>8], rep=ord('$'))


  
def bootload(c, prog):

  oldData=None
  StatusIsZero = c.rawloaderstatus.data==[0,0]
  for page in xrange(0//c.pagesize, (len(prog)+c.pagesize-1)//c.pagesize):
    data=prog[page*c.pagesize: (page+1)*c.pagesize]
    writepage(c, page*c.pagesize, data, not StatusIsZero, oldData)
    oldData=data
    StatusIsZero=True #writing a page zeros the status
    sys.stdout.flush()

  print



def currentimagebuild(c, files, key):
  if c.app.sigok:
    #sort the files so that any that have a name with the right hash in it is done first
    s = ''.join('%02X'%a for a in c.app.claimedsig) 
    first = [f for f in files if s in f.upper()]
    second = list(files - set(first))
    files = first + second

    for f in files:
      try:
        d = progload(f, maxsize = c.bootstart+1)
        if len(d) == c.app.size:
          sig = sign(d, key)
          sighex = [ord(a) for a in sig[:8]]
          if sighex == c.app.claimedsig:
            r = d + ([0xff]*(c.bootstart-18-c.app.size)) + [ord(s) for s in sig]+[c.app.size&0xff, (c.app.size>>8)&0xff]
            return f, r
      except:
        pass
 
  if c.app.fffill:
    d = [None]*c.app.size 
    r = d + ([0xff]*(c.bootstart-18-c.app.size)) + ([None]*18)
    return '<none>', r

  return None
    



def intargparse(arg):
  if arg==None:
    return arg
  elif arg.startswith('0x') or arg.startswith('0X'):
    return int(arg[2:], 16)
  else:
    return int(arg)


if __name__ == '__main__':
  key='MRBusBootLoader\x00'
  parser = argparse.ArgumentParser(description='program an mrbus node via the bootloader')
  parser.add_argument('-p', '--port', type=str,help='port for mrbus CI2 interface. Will guess /dev/ttyUSB? if not specified')
  parser.add_argument('-a', '--addr-host', help='mrbus address to use for host.  Will scan for an unused address if not specified')
  parser.add_argument('-d', '--addr', default=None, help='mrbus address of node to program.  Will scan for a singular node in bootloader mode if not specified')
  parser.add_argument('-x', '--reset-to-bootloader', action='store_true', help='send the target node a reset (\'X\') command then to attempt to enter the bootloader. Implies -l')
  parser.add_argument('-t', '--test-run', action='store_true', help='test run, Don\'t actually program. still resets, commands, boots, waits, etc, just no actual program step.')
  parser.add_argument('-l', '--listen-for-bootloader', action='store_true', help='wait for the node to send a bootloader-waiting packet, then halt the normal boot processs in bootloader mode')
  parser.add_argument('-r', '--reset-when-done', action='store_true', help='reset the target after we are finished')
#  parser.add_argument('-s', '--force-sign', action='store_true', help='sign the object even if it seems to have a signature')
#  parser.add_argument('-k', '--key-file', type=str, help='key file to use if signing with a proprietary shared key. reads the first 16 bytes from the file.')
  parser.add_argument('-c', '--cached', nargs='*', action='append', help='cached files to use for speeding up the load. If specified with no teomplate, "mrboot_cache_*.hex" is used.')
  parser.add_argument('-s', '--save-cache', nargs='?', const='mrboot_cache_*.hex', help='save the written hex file to a cache with this name template. If specified with no teomplate, "mrboot_cache_*.hex" is used.')
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
  
  mrb = mrbus.mrbus(args.port, addr=args.addr_host)#, logall=True, logfile=sys.stdout, extra=True)

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

  print 'reading current bootloader info'
  c = bootloadseek(node)

  prog = progload(args.file, maxsize = c.bootstart+1)
  if len(prog) > c.bootstart - 18:
    print 'program too long.  it is %d bytes but I only have space for %d on this device'%(len(prog), c.bootstart-18)
    sys.exit(1)

  sig=sign(prog, key)
  progandsig=prog+([0xff]*(c.bootstart-18 - len(prog)))+[ord(s) for s in sig]+[len(prog)&0xff, (len(prog)>>8)&0xff]

  if args.cached:
    cached = set(reduce(lambda a,b: a+b, args.cached))
    if not cached:
      cached = set([f for f in os.listdir('.') if f.startswith('mrboot_cache_') and f.endswith('.hex')])
    cached = currentimagebuild(c, cached, key)
    if cached:
      print 'cached loading from ',cached[0]
      c = c._replace(currentimg = cached[1])
    else:
      print 'failed to find matching cached file.  loading full image.'

  if args.test_run:
    print 'test run, would normally program here.'
  else:
    print 'programming'
    bootload(c, progandsig)

  node.sendpkt(['S'])
  while True:
    p = node.getpkt(timeout=1)
    if p and p.cmd==ord('s'):
      if p.data[0] != 0:
        print 'signature doesn\'t verify.'
        print p
        sys.exit(1)
      break
    elif p==None:
      node.sendpkt(['S'])
  else:
    print 'cant get sig at end'
    sys.exit(1)    

  print 'success'

  if args.reset_when_done:
    print 'resetting to app'
    node.sendpkt(['X', 1])

  if args.save_cache != None:
    i=args.save_cache.index('*')
    sigstr = ''.join('%02X'%ord(a) for a in sig[:8]) 
    f=args.save_cache[:i]+sigstr+args.save_cache[i+1:]
    print 'writing hex file to cache as', f
    fl = open(f,'wb')
    ih=intelhex.IntelHex()
    for i,d in enumerate(prog):
      ih[i]=d
    ih.tofile(fl, format='hex')#doesn't save the sig if we made one. deal with that when I think about sig helpers
    fl.close
    

