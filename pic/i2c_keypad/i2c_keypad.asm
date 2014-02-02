   list      p=16f54            ; list directive to define processor
   radix dec
 include p16f54.inc
 include helpers.inc

DEV_ADDRESS EQU 0x37

;todo convert to wait using no prescal (periodh and periodl, plus not clearing TMR0
 __CONFIG _CP_OFF & _WDT_OFF & _RC_OSC

;rc@ 1.21 mips

temp EQU 7
databyte EQU 8
devaddress EQU 9
regaddress EQU 10
halfperiod EQU 11
periods_cnt_low EQU 12
periods_cnt_high EQU 13
entropy0 EQU 14
entropy1 EQU 15
entropy2 EQU 16
entropy3 EQU 17

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
  ; also watches keypad for entropy changes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB
start_bit.wait_idle
 movf PORTB, w
 andlw (1<<pin_clk) | (1<<pin_dat)
 bwneql (1<<pin_clk) | (1<<pin_dat), start_bit.wait_idle

start_bit.wait_clklow
 movfw PORTB
 movwf temp
 btfbc temp, pin_clk, start_bit.wait_idle
 btfbs temp, pin_dat, start_bit.wait_clklow

start_bit.wait_bothlow
 movfw PORTB
 movwf temp
 btfbs temp, pin_clk, start_bit.wait_bothlow

 retlw 0



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
read_byte
;reads a byte into databyte
;leaves the clock pulled lowand doesn't send ack (streached) .
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
c=0 
 WHILE c < 8
 clrc
read_byte.clk_wait_#v(c)
 btfbc PORTB, pin_clk, read_byte.clk_wait_#v(c)
 btfsc PORTB, pin_dat
 setc
 rlf databyte, f
c+=1
 ENDW

clk_wait_ack
 btfbs PORTB, pin_clk, clk_wait_ack
 trisb IDLETRISB & ~(1<<pin_clk)

 retlw 0

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ack
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB & ~((1<<pin_dat)|(1<<pin_clk))
 nop
 trisb IDLETRISB & ~(1<<pin_dat)
 goto nack.clk_wait_ack1
 ;fall through to nack!


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
nack
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 trisb IDLETRISB
nack.clk_wait_ack1
 btfbc PORTB, pin_clk, nack.clk_wait_ack1
 nop
nack.clk_wait_ack2
 btfbs PORTB, pin_clk, nack.clk_wait_ack2
 trisb IDLETRISB

 retlw 0



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
wait MACRO time
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 LOCAL l2
l2
 cbfflt TMR0, time, l2
 clrf TMR0
 ENDM


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
beep
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 clrf TMR0
beep.loop
 wait halfperiod
 bcf PORTB, pin_piezo
 wait halfperiod
 bsf PORTB, pin_piezo
 dbnz periods_cnt_low, beep.loop
 dbnz periods_cnt_high, beep.loop

 retlw 0


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
 bflneq regaddress, 1, notwrite1
 call ack
 call read_byte
 movff databyte, halfperiod
 call ack
 call read_byte
 movff databyte, periods_cnt_high
 call ack
 call read_byte
 movff databyte, periods_cnt_low
 call beep
 call ack
 goto wait_start

notwrite1
 bflneq regaddress, 2, notwrite2
 call ack
 call read_byte
 movbit databyte, 0, PORTB, pin_led_k
 movnbit databyte, 1, PORTB, pin_led_s
 call ack

notwrite2
 call nack
 goto wait_start

reading
 bflneq regaddress, 1, notread1
 call readkeypad
 mov keypad1, databyte
 call ack
 call writebyte
 mov keypad2, databyte
 call write2ready
 call writebyte
 goto wait_start

notread1
 bflneq regaddress, 1, notread2
; finish jenkins_one_at_a_time_hash
;  hash += (hash << 3);
 clrc
 rrf hash, w
 movwf temp
 clrc
 rrf temp, f
 clrc
 rrf temp, w
 addwf hash, f

;  hash ^= (hash >> 11);
    hash += (hash << 15);
 mov entropy, databyte
 call ack
 call writebyte
 goto wait_start

notread2
 call nack
 goto wait_start




 goto wait_start





 END

