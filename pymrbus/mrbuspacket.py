"""this is based on the mrbus.py in the bootloader, but intended to be version two with a more separated event layer and imerative layer.  once this seems ok I'll separate it out into it's own project in the mrbus upstream then convert bootloader to use it.
"""
from __future__ import division


class packet(object):
  def __init__(self, dest, src, cmd, data):
    self.dest=dest
    self.src=src
    self.cmd=cmd
    self.data=data

  def __hash__(self):
    return hash(repr(self))

  def __eq__(self, other):
    if type(other)==packet:
      return repr(self)==repr(other)
    return list(other)==[self.cmd]+self.data

  def __repr__(self):
    return "mrbus.packet(0x%02x, 0x%02x, 0x%02x, %s)"%(self.dest, self.src, self.cmd, repr(self.data))

  def __str__(self):
    c='(%02xh'%self.cmd
    if self.cmd >= 32 and self.cmd <= 127:
      c+=" '%c')"%self.cmd
    else:
      c+="    )"
    return "packet(%02xh->%02xh) %s %2d:%s"%(self.src, self.dest, c, len(self.data), ["%02xh"%d for d in self.data])


