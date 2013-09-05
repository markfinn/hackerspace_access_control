
 radix dec

;skpc MACRO
; btfss STATUS, C
; ENDM

;skpnc MACRO
; btfsc STATUS, C
; ENDM

;skpz MACRO
; btfss STATUS, Z
; ENDM

;skpnz MACRO
; btfsc STATUS, Z
; ENDM



bfz MACRO file, target
 tstf file
 bz target
 ENDM

bfnz MACRO file, target
 tstf file
 bnz target
 ENDM
 
;bz MACRO target
; skpnz
; goto target
; ENDM

;bnz MACRO target
; skpz
; goto target
; ENDM

;bc MACRO target
; skpnc
; goto target
; ENDM

;bnc MACRO target
; skpc
; goto target
; ENDM

;clrc MACRO
; bcf STATUS, C
; ENDM

;setc MACRO
; bsf STATUS, C
; ENDM

;clrz MACRO
; bcf STATUS, Z
; ENDM

;setz MACRO
; bsf STATUS, Z
; ENDM

;movfw MACRO file
; movf file, w
; ENDM

;tstf MACRO file
; movf file, f
; ENDM


movlf MACRO value, target
 movlw value
 movwf target
 ENDM

trisa MACRO value
 movlw value
 tris PORTA
 ENDM

trisb MACRO value
 movlw value
 tris PORTB
 ENDM

bweql MACRO value, target
 xorlw value
 bz target
 ENDM

bwneql MACRO value, target
 xorlw value
 bnz target
 ENDM

bfeql MACRO file, value, target
 movfw file
 bweql value, target
 ENDM

bfneql MACRO file, value, target
 movfw file
 bwneql value, target
 ENDM

btfbc MACRO file, bit, target
 btfss file, bit
 goto target
 ENDM

btfbs MACRO file, bit, target
 btfsc file, bit
 goto target
 ENDM

movbit MACRO sfile, sbit, dfile, dbit
 bcf dfile,dbit
 btfsc sfile, sbit
 bsf dfile,dbit
 ENDM

movff MACRO src, dst
 movfw src
 movwf dst 
 ENDM

mov MACRO src, dst
 movff src, dst 
 ENDM

dbnz MACRO file, target
 decfsz file, f
 goto target
 ENDM


cbfflt MACRO f1, f2, target
 movfw f2
 subwf f1, w
 bnc target
 ENDM

cbffge MACRO f1, f2, target
 movfw f2
 subwf f1, w
 bc target
 ENDM

cbffgt MACRO f1, f2, target
 movfw f1
 subwf f2, w
 bnc target
 ENDM

cbffle MACRO f1, f2, target
 movfw f1
 subwf f2, w
 bc target
 ENDM

