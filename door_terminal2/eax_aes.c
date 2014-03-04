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
//can be used in place. (out==data)
//modifies nOnce
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
//out must be dataSz + tagSz long.
//can be used in place. (out==data)
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

uint8_t eax_aes_dec(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t tagSz, uint8_t *nonce, uint8_t nonceSz, uint8_t * hData, uint16_t hDataSz, uint8_t *cipher, uint16_t cipherSz)
{
//out must be dataSz long, which is equivalent to cipherSz - tagSz.
//can be used in place. (out==cipher)
//returns a VALID flag
//only writes to output if it is a valid message
  if (cipherSz < tagSz)
    return 0;

  //split the ciphertext into Cipher and Tag
	cipherSz -= tagSz;
	uint8_t tag[16];
	memcpy(tag, cipher+cipherSz, tagSz);

	uint8_t tmp[16];
	omact(cmac_ctx, tmp, 2, cipher, cipherSz);
	for(int i=0;i<tagSz;i++)
		tag[i] ^= tmp[i];

	omact(cmac_ctx, tmp, 1, hData, hDataSz);
	for(int i=0;i<tagSz;i++)
		tag[i] ^= tmp[i];

	omact(cmac_ctx, tmp, 0, nonce, nonceSz);
	for(int i=0;i<tagSz;i++)
	{
		if((tag[i] ^ tmp[i]))
			return 0;
	}

  ctr_aes(cmac_ctx->key_ctx, tmp, out, cipher, cipherSz);
	return 1;
}

