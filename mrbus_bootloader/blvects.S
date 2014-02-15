; Custom vector table that eliminates wasted space between the bare essential vestors.
;stuffs libc init code inside the table, and uses gaps as user code vectors

.section .bootvectreset,"ax",@progbits
 rjmp __init_here





.section .bootvectjmp,"ax",@progbits
user_vectors1:
 rjmp aes128_init ; vector
 rjmp aes128_enc ; vector
 rjmp aes192_init ; vector
 rjmp aes192_enc ; vector
 rjmp aes256_init ; vector
 rjmp aes256_enc ; vector





.section .bootvectint,"ax",@progbits
 rjmp __vector_18  ; 0x48 !used interrupt!
user_vectors2:
 rjmp lenpadcbcmacaes ; vector
 rjmp __vector_19  ; 0x4c !used interrupt!
user_vectors3:
 nop               ; 0x4e
 rjmp __vector_20  ; 0x50 !used interrupt!
user_vectors4:




