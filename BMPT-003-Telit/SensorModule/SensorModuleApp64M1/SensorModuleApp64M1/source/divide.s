;********************************************************************************************
; divide a 32-bit unsigned int by a 16-bit unsigned int 
; 
; enter with 32-bit in BH:BL:AH:AL 
; enter with 16-bit in CH:CL 
; 
; exit with result in BH:BL:AH:AL 
; exit with remainder in "remainder4:remainder3:remainder2:remainder1". 

;.def remainder1 = r12 
;.def remainder2 = r13 
;.def remainder3 = r14 
;.def remainder4 = r15 
;AL = r16 
;AH = r17 
;BL = r18 
;BH = r19 
;CL = r20 
;CH = r21
;DL = r22
;DH = r23

divideU32byU16: 
  clr    r12      ;1   clear remainder 
  clr    r13      ;1
  clr    r14      ;1 
  clr    r15      ;1 

  clr    r23              ;1 

  ldi    r22,33           ;1   init loop counter 

divideU32byU16a: 
  sec                    ;1   set carry 

divideU32byU16b: 
  rol    r16              ;1   shift left dividend 
  rol    r17              ;1 
  rol    r18              ;1 
  rol    r19              ;1 

  dec    r22              ;1 
  brne   divideU32byU16c ;1/2 

  ret                     ;4 

divideU32byU16c: 
  rol    r12       ;1 
  rol    r13       ;1 
  rol    r14       ;1 
  rol    r15       ;1 

  sub    r12,r20    ;1   rem -= divisor 
  sbc    r13,r21    ;1 
  sbc    r14,r23    ;1 
  sbc    r15,r23    ;1 
  brcc   divideU32byU16a  ;1/2 if res < 0 

  add    r12,r20    ;1   restore remainder 
  adc    r13,r21    ;1 
  adc    r14,r23    ;1 
  adc    r15,r23    ;1 

  clc                     ;1   clear carry 

  rjmp   divideU32byU16b  ;2