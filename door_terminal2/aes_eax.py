from Crypto.Cipher import AES
from Crypto.Util import Counter



def intfromstr(s):
  i=0
  for c in s:
    i=(i<<8)|ord(c)
  return i

def intfrombytes(l):
  i=0
  for c in l:
    i=(i<<8)|c
  return i


def strfromint(i, n):
  s=''
  while i:
    s=chr(i&0xff)+s
    i>>=8
  assert len(s) <= n
  return '\x00'*(n-len(s))+s

def cbc(K, M, m):
  enc = AES.new(strfromint(K, 16), AES.MODE_CBC, '\x00'*16)
  out = enc.encrypt(strfromint(M, m))
  return intfromstr(out[-16:])

def ctr(N, K, M, m):
  ctr = Counter.new(128, initial_value=N)
  enc = AES.new(strfromint(K, 16), AES.MODE_CTR, counter=ctr)
  m2 = (m+15)//16

  S = ''
  for i in xrange(m2):
    S += enc.encrypt('\x00'*16)
  S=intfromstr(S[:m])

  return M^S

def pad(M, m, B, P):
  if m and m%16 == 0:
    x=B
  else:
    x=P
    M=(M<<(8*(16-m%16))) | (0x80<<(8*(15-m%16)))
  return M^x

def L2(L):
  L<<=1
  if L&(1<<128):
    L^=0x87
  return L&((1<<128)-1)

def OMAC(K, M, m):#aka AES-CMAC aka OMAC1
  enc = AES.new(strfromint(K, 16), AES.MODE_ECB)
  L = intfromstr(enc.encrypt('\x00'*16))
  B = L2(L)
  P = L2(B)
  return cbc(K, pad(M, m, B, P), 16*((m+15)//16) if m>0 else 16)

def OMACt(t, K, M, m):
  return OMAC(K, (t<<(8*m))|M, m+16)


def aead_eax_aes(K, N, n, H, h, M, m, t):
  #print 'eax:', hex(K), hex(N), n, hex(H), h, hex(M), m, t
  Nx = OMACt(0, K, N, n)
  #print 'Nx:', hex(Nx)
  Hx = OMACt(1, K, H, h)
  #print 'Hx:', hex(Hx)
  C = ctr(Nx, K, M, m)
  #print 'C:', hex(C)
  Cx = OMACt(2, K, C, m)
  #print 'Cx:', hex(Cx)
  TAG = Nx^Cx^Hx
  #print 'TAG:', hex(TAG)
  T = TAG>>(8*(16-t))
  return C<<(8*t)|T


def aead_eax_aes_dec(K, N, n, H, h, C, c, t):
  if c<t:
    return None

  #split
  T=C&((1<<(8*t))-1)
  C=C>>(8*t)
  c-=t
	
  Nx = OMACt(0, K, N, n)
  Hx = OMACt(1, K, H, h)
  Cx = OMACt(2, K, C, c)
  TAGx = Nx^Cx^Hx
  Tx = TAGx>>(8*(16-t))

  if T != Tx:
    return None

  M = ctr(Nx, K, C, c)
  return M



if __name__ == '__main__':

  def r(s):
    if s == '':
      return 0
    return int(s, 16)

  assert L2(0x80) == 0x100
  assert L2(0x0) == 0x0
  assert L2(0x80<<(8*15)) == 0x87
  assert L2(0x41<<(8*15)) == (0x82<<(8*15))
  assert L2(0x81<<(8*15)) == (0x2<<(8*15))|0x87

  KEY=r('2b7e151628aed2a6abf7158809cf4f3c')
  MSG=r('6bc1bee22e409f96e93d7e117393172a')
  tag=r('070a16b46b4d4144f79bdd9dd04a287c')
  assert OMAC(KEY, MSG, 16) == tag

  KEY=r('2b7e151628aed2a6abf7158809cf4f3c')
  MSG=r('')
  tag=r('bb1d6929e95937287fa37d129b756746')
  assert OMAC(KEY, MSG, 0) == tag

  KEY=r('2b7e151628aed2a6abf7158809cf4f3c')
  MSG=r('6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411')
  tag=r('dfa66747de9ae63030ca32611497c827')
  assert OMAC(KEY, MSG, 40) == tag

  KEY=r('2b7e151628aed2a6abf7158809cf4f3c')
  MSG=r('6bc1bee22e409f96e93d7e117393172aae2d8a571e03ac9c9eb76fac45af8e5130c81c46a35ce411e5fbc1191a0a52eff69f2445df4f9b17ad2b417be66c3710')
  tag=r('51f0bebf7e3b9d92fc49741779363cfe')
  assert OMAC(KEY, MSG, 64) == tag



  MSG=r('')
  KEY=r('233952DEE4D5ED5F9B9C6D6FF80FF478')
  NONCE=r('62EC67F9C3A4A407FCB2A8C49031A8B3')
  HEADER=r('6BFB914FD07EAE6B')
  CIPHER=r('E037830E8389F27B025A2D6527E79D01')
  assert aead_eax_aes(KEY, NONCE, 16, HEADER, 8, MSG, 0, 16) == CIPHER
  assert aead_eax_aes_dec(KEY, NONCE, 16, HEADER, 8, CIPHER, 16, 16) == MSG

  MSG=r('F7FB')
  KEY=r('91945D3F4DCBEE0BF45EF52255F095A4')
  NONCE=r('BECAF043B0A23D843194BA972C66DEBD')
  HEADER=r('FA3BFD4806EB53FA')
  CIPHER=r('19DD5C4C9331049D0BDAB0277408F67967E5')
  assert aead_eax_aes(KEY, NONCE, 16, HEADER, 8, MSG, 2, 16) == CIPHER
  assert aead_eax_aes_dec(KEY, NONCE, 16, HEADER, 8, CIPHER, 18, 16) == MSG

  MSG=r('CA40D7446E545FFAED3BD12A740A659FFBBB3CEAB7')
  KEY=r('8395FCF1E95BEBD697BD010BC766AAC3')
  NONCE=r('22E7ADD93CFC6393C57EC0B3C17D6B44')
  HEADER=r('126735FCC320D25A')
  CIPHER=r('CB8920F87A6C75CFF39627B56E3ED197C552D295A7CFC46AFC253B4652B1AF3795B124AB6E')
  assert aead_eax_aes(KEY, NONCE, 16, HEADER, 8, MSG, 21, 16) == CIPHER
  assert aead_eax_aes_dec(KEY, NONCE, 16, HEADER, 8, CIPHER, 37, 16) == MSG

  #check a shorter tag too
  assert aead_eax_aes_dec(KEY, NONCE, 16, HEADER, 8, aead_eax_aes(KEY, NONCE, 16, HEADER, 8, MSG, 21, 7), 28, 7) == MSG


