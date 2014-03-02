#ifndef CMAC_AES_H_
#define CMAC_AES_H_

#include"aes_types.h"

typedef struct{
	aes128_ctx_t* key_ctx;
	uint8_t k1[16];
	uint8_t k2[16];
} cmac_aes_ctx_t;

void cmac_aes_init(aes128_ctx_t* key_ctx, cmac_aes_ctx_t* cmac_ctx);
void  cmac_aes(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t *data, uint16_t sz);
void  cmac_aes_noclear(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t *data, uint16_t sz);

#endif /* CMAC_AES_H_ */

