#ifndef EAX_AES_H_
#define EAX_AES_H_

#include "cmac_aes.h"

void    eax_aes_enc(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t tagSz, uint8_t *nonce, uint8_t nonceSz, uint8_t * hData, uint16_t hDataSz, uint8_t *data,   uint16_t dataSz  );
uint8_t eax_aes_dec(cmac_aes_ctx_t* cmac_ctx, uint8_t *out, uint8_t tagSz, uint8_t *nonce, uint8_t nonceSz, uint8_t * hData, uint16_t hDataSz, uint8_t *cipher, uint16_t cipherSz);

#endif /* EAX_AES_H_ */

