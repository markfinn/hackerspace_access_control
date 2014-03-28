#include <string.h>
#include"cmac_aes.h"
#include "mrbus_bootloader_builtins.h"

static void cmac_double(uint8_t* dest, uint8_t* src)
{
	uint8_t i; 

	for (i=0;i<15;i++)
		dest[i] = (src[i]<<1)|(src[i+1]>>7);
	dest[15] = src[15]<<1;
	if(src[0]&0x80)
		dest[15]^=0x87;


/* work in progress, don't really know avr asm.
asm ("ld __tmp_reg__, %0\n\t"
"lsl __tmp_reg__\n\t" 
"st %0+, __tmp_reg__\n\t"

"ld __tmp_reg__, %0\n\t"
"rol __tmp_reg__\n\t" 
"st %0+, __tmp_reg__\n\t"

"ld __tmp_reg__, %0\n\t"
"rol __tmp_reg__\n\t" 
"st %0+, __tmp_reg__\n\t"

"ld __tmp_reg__, %0\n\t"
"rol __tmp_reg__\n\t" 
"st %0+, __tmp_reg__\n\t"
: "=e" (b) ::);
*/
}


void cmac_aes_init(aes128_ctx_t* key_ctx, cmac_aes_ctx_t* cmac_ctx)
{
	cmac_ctx->key_ctx = key_ctx;
	memset(cmac_ctx->k2, 0, 16);
	aes128_enc(cmac_ctx->k2, cmac_ctx->key_ctx);
	cmac_double(cmac_ctx->k1, cmac_ctx->k2);
	cmac_double(cmac_ctx->k2, cmac_ctx->k1);
}

void  cmac_aes_noclear(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t *data, uint16_t sz)
{
	uint8_t *x;
	int i;
	uint16_t blocks = (sz+15)/16;

	if (blocks==0)
	  blocks=1;
	sz -= (blocks-1)*16;

	while (--blocks > 0)
	{
		for(i=0;i<16;i++, data++)
			out[i]^=*data;
		aes128_enc(out, cmac_ctx->key_ctx);
	}
	for(i=0;i<sz;i++, data++)
		out[i]^=*data;
	if(i<16)
	{
		x = cmac_ctx->k2;
		out[i]^=0x80;
	}
	else
		x = cmac_ctx->k1;
	for(i=0;i<16;i++)
		out[i]^=x[i];
	aes128_enc(out, cmac_ctx->key_ctx);
}

void  cmac_aes(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t *data, uint16_t sz)
{
	memset(out, 0, 16);
	cmac_aes_noclear(cmac_ctx, out, data, sz);
}

