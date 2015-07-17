

class CMP(object):
  def _startTimerHandler(self):
    if not self._supportsCMP:
      self.node.sendpkt([0xff, 0x00])#CMP capabilites request
      self.node.log(0, 'trying cmt start')
      self._tryStartHint = self.node.installTimer(self._tryStartDelay, lambda: self._startTimerHandler()) 
      self._tryStartDelay = min(10, self._tryStartDelay*1.5)



  def __init__(self, node, enableCMP):
    self.node=node
    self._supportsCMP = None #unsure at start
    if not enableCMP:
      self._supportsCMP = False
    
    self._tryStartDelay = .15

    if enableCMP:
      self.hint = node.install(lambda p: self._handler(p))
      self._startTimerHandler()
    else:
      self.hint = None

  def __dell__(self):
    if self.hint:
      self.node.remove(self.hint)

  def _handler(self, p):
    if p.cmd == 0xff or p.cmd == 0xfe:
      self.node.log(0, 'cmp pkt: %s'%p)
      self._supportsCMP = True
      if self._tryStartHint:
        self.node.removeTimer(self._tryStartHint)
      return True #eat packet

  def maxPktLen(self, timeout=0):
    if self._supportsCMP == False:
      return 20

    return 20
    
  def isSupported(self, timeout=0):
    #can't block for something that might NEVER return
    assert timeout != None
    if timeout == None:
      timeout = 2 

    #return answer now if we should or can
    if timeout == 0 or self._supportsCMP != None:
      return self._supportsCMP

    self.node.pump(until=lambda:self._supportsCMP, duration=timeout)

    return self._supportsCMP


