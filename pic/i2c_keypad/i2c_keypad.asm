   list      p=16f54            ; list directive to define processor
   radix dec
 include p16f54.inc
 include helpers.inc

DEV_ADDRESS EQU 0x37

;todo convert to wait using no prescal (periodh and periodl, plus not clearing TMR0
 __CONFIG _CP_OFF & _WDT_OFF & _RC_OSC

;rc@ 1.21 mips

databyte EQU 7
devaddress EQU 8
regaddress EQU 9
halfperiod EQU 10
entropy0 EQU 11
entropy1 EQU 12
entropy2 EQU 13
entropy3 EQU 14
temp0 EQU 15
temp1 EQU 16
temp2 EQU 17
temp3 EQU 18

;123
;456
;789
;e0

;ra 0-3 => rows 1-4 with 100k pull up

;rb 0-2 => cols 3-1
;   3 => keypad led
;   4 => status led (inverted)
;   5 => piezo
;   3 => clock
;   4 => data

pin_col1 EQU 0
pin_col2 EQU 1
pin_col3 EQU 2
pin_led_k EQU 3
pin_led_s EQU 4
pin_piezo EQU 5
pin_clk EQU 6
pin_dat EQU 7


LED_STATUS_OFF MACRO
 bsf PORTB, pin_led_s
 ENDM
LED_STATUS_ON MACRO
 bcf PORTB, pin_led_s
 ENDM
LED_KEYPAD_OFF MACRO
 bcf PORTB, pin_led_k
 ENDM
LED_KEYPAD_ON MACRO
 bsf PORTB, pin_led_k
 ENDM



IDLETRISA EQU b'00001111'
IDLETRISB EQU (0<<pin_col1)|(0<<pin_col2)|(0<<pin_col3)|(1<<pin_clk)|(1<<pin_dat)|(0<<pin_led_k)|(0<<pin_led_s)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 org 0x1ff
reset_vect
 goto start
 org 0



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
start_bit
  ;watch for start bit
  ; process background tasks while waiting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB
start_bit.wait_idle
 call background
 movf PORTB, w
 andlw (1<<pin_clk) | (1<<pin_dat)
 bwneql (1<<pin_clk) | (1<<pin_dat), start_bit.wait_idle

start_bit.wait_clklow
 call background
 movfw PORTB
 movwf temp0
 btfbc temp0, pin_clk, start_bit.wait_idle
 btfbs temp0, pin_dat, start_bit.wait_clklow

start_bit.wait_bothlow
 movfw PORTB
 movwf temp0
 btfbs temp0, pin_clk, start_bit.wait_bothlow

 retlw 0



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
read_byte
;reads a byte into databyte
;leaves the clock pulled lowand doesn't send ack (streched) .
  ; process background tasks while waiting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
c=0 
 WHILE c < 8
 clrc
read_byte.clk_wait_#v(c)
 call background
 btfbc PORTB, pin_clk, read_byte.clk_wait_#v(c)
 btfsc PORTB, pin_dat
 setc
 rlf databyte, f
c+=1
 ENDW

clk_wait_ack
 call background
 btfbs PORTB, pin_clk, clk_wait_ack
 trisb IDLETRISB & ~(1<<pin_clk)

 retlw 0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ack
  ; process background tasks while waiting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB & ~((1<<pin_dat)|(1<<pin_clk))
 nop
 trisb IDLETRISB & ~(1<<pin_dat)
 goto nack.clk_wait_ack1
 ;run through to nack!


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
nack
  ; process background tasks while waiting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB
nack.clk_wait_ack1
 call background
 btfbc PORTB, pin_clk, nack.clk_wait_ack1
 nop
nack.clk_wait_ack2
 call background
 btfbs PORTB, pin_clk, nack.clk_wait_ack2
 trisb IDLETRISB

 retlw 0


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
background
; process background tasks
  ;  watches keypad for status and entropy
  ;  runs beeper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
readkeypad
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;todo: this.


;todo bad idea?
 sfnz keypad
 retlw 0
 movfw TMR0
 call entropygen
 movfw keypad
 call entropygen
 retlw 0



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
entropygen
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 do
;jenkins_one_at_a_time_hash
hash += key[i];
hash += (hash << 10);
hash ^= (hash >> 6);
 retlw 0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
start
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 movlf 0, PORTA 
 movlf 0, PORTB 
 trisa IDLETRISA
 trisb IDLETRISB
 CLRWDT
 movlw b'000010'
 option
 LED_STATUS_OFF
 LED_KEYPAD_OFF

wait_start
 call start_bit
 call read_byte

 movfw TMR0
 call entropygen

 clrc
 rrf databyte, w
 bwneql DEV_ADDRESS, wait_start
 movff databyte, devaddress 
 call ack

 call read_byte
 movff databyte, regaddress 
 movfw TMR0
 call entropygen
 btfbs devaddress, 0, reading

writing
 bflneq regaddress, 0, notwrite1
 movlf 0, halfperiod
 call ack
 call read_byte
 movff databyte, halfperiod
 call ack
 goto wait_start

notwrite1
 bflneq regaddress, 1, i2caddrfail
 call ack
 call read_byte
 movbit databyte, 0, PORTB, pin_led_k
 movnbit databyte, 1, PORTB, pin_led_s
 call ack
 goto wait_start

i2caddrfail
 call nack
 goto wait_start

reading
 bflneq regaddress, 0, notread1
 mov keypad1, databyte
 call ack
 call writebyte
 mov keypad2, databyte
 call write2ready
 call writebyte
 goto wait_start

notread1
 bflneq regaddress, 1, i2caddrfail
; finish jenkins_one_at_a_time_hash
;  hash += (hash << 3);
 clrc
 rrf entropy0, w
 movwf temp0
 rrf entropy1, w
 movwf temp1
 rrf entropy2, w
 movwf temp2
 rrf entropy3, w
 movwf temp3

 clrc
 rrf temp0, f
 rrf temp1, f
 rrf temp2, f
 rrf temp3, f

 clrc
 rrf temp0, f
 rrf temp1, f
 rrf temp2, f
 rrf temp3, f

 addff32 entropy, temp

;todo  hash ^= (hash >> 11);
;todo hash += (hash << 15);

 mov entropy0, databyte
 call ack
 call writebyte
 mov entropy1, databyte
 call write2ready
 call writebyte
 mov entropy2, databyte
 call write2ready
 call writebyte
 mov entropy3, databyte
 call write2ready
 call writebyte
 goto wait_start


 END

