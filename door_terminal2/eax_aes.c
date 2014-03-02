#include <string.h>

#include "mrbus_bootloader_builtins.h"
#include "cmac_aes.h"


static void omact(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t t, uint8_t *data, uint16_t sz)
{
	memset(out, 0, 15);
	out[15] = t;
	aes128_enc(out, cmac_ctx->key_ctx);
	cmac_aes_noclear(cmac_ctx, out, data, sz);
}


static void ctr_aes(aes128_ctx_t *aes_ctx, uint8_t*nOnce, uint8_t *out, uint8_t *data, uint16_t dataSz)
{
	uint8_t tmp[16];

	for(int i=0; i<dataSz; i++)
	{
		if ((i&0xf) ==0)
		{
			memcpy(tmp, nOnce, 16);
		  aes128_enc(tmp, aes_ctx);
			//inc nonce
		  uint8_t n=1;
		  for (uint8_t*p=nOnce+15;n!=0 && p>=nOnce;p--)
		  	n=*p=(*p)+1;
		}
		out[i] = data[i] ^ tmp[i&0xf];
	}
}

void eax_aes_enc(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t tagSz, uint8_t *nonce, uint8_t nonceSz, uint8_t * hData, uint16_t hDataSz, uint8_t *data, uint16_t dataSz)
{
	uint8_t tmp[16];
	
	omact(cmac_ctx, tmp, 0, nonce, nonceSz);
	memcpy(out+dataSz, tmp, tagSz);
  ctr_aes(cmac_ctx->key_ctx, tmp, out, data, dataSz);

	omact(cmac_ctx, tmp, 2, out, dataSz);
	out+=dataSz;
	for(int i=0;i<tagSz;i++)
		out[i] ^= tmp[i];

	omact(cmac_ctx, tmp, 1, hData, hDataSz);
	for(int i=0;i<tagSz;i++)
		out[i] ^= tmp[i];
}
/*
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


*/
