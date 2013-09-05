   list      p=16f54            ; list directive to define processor
   radix dec
 include p16f54.inc
 include helpers.inc

 __CONFIG _WDT_OFF & _RC_OSC

wait MACRO
 LOCAL l2
l2
 bfneql TMR0, 255, l2
 clrf TMR0
 ENDM



 org 0
start
 movlw 0
 tris PORTA
 movlw 0
 tris PORTB
 CLRWDT
 movlw b'000111'
 option


 clrf TMR0
loop
 wait
 bcf PORTB, 3
 bcf PORTB, 4
 wait
 bsf PORTB, 3
 bsf PORTB, 4
 goto loop



;reset vect
 org 0x1ff
 goto start
 END

