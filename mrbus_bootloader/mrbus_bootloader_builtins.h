//#define BOOTSTART 0x7000

#ifndef MRBUS_BOOTLOADER_BUILTINS_H_
#define MRBUS_BOOTLOADER_BUILTINS_H_

#include <stdint.h>
#include "aes_types.h"

static __inline__ uint8_t getbootloaderver(void){ return ((uint8_t (*)(void))((BOOTSTART+0x2)/2))(); }

static __inline__ void lenpadcbcmacaes(uint8_t *data, uint8_t *key, uint16_t sz, uint8_t (*dataget)(uint16_t), uint16_t offset){ (void (*)((uint8_t *data, uint8_t *key, uint16_t sz, uint8_t (*dataget)(uint16_t), uint16_t offset))((BOOTSTART+0xc)/2))(); }

static __inline__ void aes128_init(const void* key, aes128_ctx_t* ctx){ ((void (*)(const void* key, aes128_ctx_t* ctx))((BOOTSTART+0x4)/2))(); }
static __inline__ void aes128_enc(void* buffer, aes128_ctx_t* ctx){ ((void (*)(void* buffer, aes128_ctx_t* ctx))((BOOTSTART+0x6)/2))(); }

static __inline__ void aes192_init(const void* key, aes192_ctx_t* ctx){ ((void (*)(const void* key, aes192_ctx_t* ctx))((BOOTSTART+0x8)/2))(); }
static __inline__ void aes192_enc(void* buffer, aes192_ctx_t* ctx){ ((void (*)(void* buffer, aes192_ctx_t* ctx))((BOOTSTART+0xa)/2))(); }

static __inline__ void aes256_init(const void* key, aes256_ctx_t* ctx){ ((void (*)(const void* key, aes256_ctx_t* ctx))((BOOTSTART+0x4a)/2))(); }
static __inline__ void aes256_enc(void* buffer, aes256_ctx_t* ctx){ ((void (*)(void* buffer, aes256_ctx_t* ctx))((BOOTSTART+0x4e)/2))(); }

static __inline__ void boot_program_page (uint32_t page, uint8_t *buf){ ((void (*)(uint32_t page, uint8_t *buf))((BOOTSTART+0x52)/2))(); }



#endif /* MRBUS_BOOTLOADER_BUILTINS_H_ */

