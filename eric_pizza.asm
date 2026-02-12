$NOLIST
$MODMAX10
$LIST

CLK           EQU 33333333 ; Microcontroller system crystal frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.
TIMER2_RATE   EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/(12*TIMER2_RATE))))
BAND          EQU 2 ;for flat states

WINDOW      EQU 6     ; burst window length in seconds 

BAUD   EQU 57600
T1_LOAD EQU 256-(2*CLK) / (32*12*BAUD) ;Load 253 so it counts 3 counts before overflowing, which gives us a 57600 baud rate with a 33.333MHz clock
ABORT_TOKEN  EQU 0A5h

; ********* Buttons ***********
SELECT_BUTTON equ P3_4 ; middle
RESET_BUTTON  equ P3_2 ; left
START_BUTTON  equ P3_5 ; middle right
STOP_BUTTON   equ P3_7 ; right 
PARAM_BUTTON  equ P3_3 ; middle left

OVEN_PIN      equ p0.0
SOUND_OUT     equ P1_5 ; Speaker attached to this pin
UPDOWN        equ SWA_0
TENS          equ SWA_1
PRESET1_SW     equ SWA_7
PRESET2_SW     equ SWA_6
PRESET3_SW     equ SWA_5
PRESET4_SW     equ SWA_4
RED_LED       equ P1_0
GREEN_LED     equ P3_1

; Reset vector
org 0x0000
    ljmp MAIN

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
	
; Timer/Counter 2 overflow interrupt vector
org 0x002B
	ljmp Timer2_ISR


;--- DATA RAM ---
dseg at 0x30
STATE_VAR_1:     DS 1 
STATE_VAR_2:     DS 1
SECOND_COUNTER:  DS 1 
TEMP_HIGH_BYTE:  DS 1 
TEMP_LOW_BYTE:   DS 1 

;---------ADC RELATED-----;
ADCREFH:		 DS 1
ADCREFL:         DS 1
LM335H:          DS 1
LM335L:          DS 1
TCH:			 DS 1
TCL:			 DS 1
TH_H:            DS 1
TH_L:            DS 1
;----------------------
TEMP:            DS 5
TIME:            DS 2
POWER:           DS 2
DEGREES60:       DS 2
DEGREES150:      DS 2
DEGREES220:      DS 2
TARGET:          DS 2
TARGET_TIME:     DS 2
;*** Variables ***
SOAK_TEMP_set:       ds 2
SOAK_TIME_set:       ds 2
reflow_temp_set:     ds 2
REFLOW_TIME_set:     ds 2

soak_time:       ds 2
REFLOW_TIME:     ds 2

beep_count:      ds 1
T0_RH:           ds 1   ;Timer0 reload high bit
T0_RL:           ds 1   ;Timer0 reload low bit

; PWM variables
LOW_LIMIT:  ds 2
HIGH_LIMIT: ds 2
THRESHOLD:  ds 2
ON_SECS:     ds 1
PHASE:       ds 1    ;counter for seconds
LEAD:        ds 2 ;for ramp sates
ABORT_FLAG: ds 1

x:		ds	4 ;used for 32 bit math for temperature conversion
y:		ds	4 ;used for 32 bit math for temperature conversion
bcd:    ds  5 ; <--- ADD THIS: math32 needs 5 bytes for BCD conversions

;--ISR RELATED--;
Count1ms:     ds 2 ; Used to determine when half second has passed
BCD_counter:  ds 1 ; The BCD counter incrememted in the ISR and displayed in the main loop


; **** keypad variables ****
keypad_digit_count: ds 1

bseg
START_FLAG:         DBIT 1  ; Use DBIT for single bits in bseg
half_seconds_flag:  DBIT 1  ; half second flag
seconds_flag:       DBIT 1  ; can change later depending on how fast we want it
SELECT_BUTTON_FLAG: DBIT 1
PARAM_BUTTON_FLAG:  DBIT 1
KEYPAD_FLAG:        DBIT 1
mf:     dbit 1 ; <--- ADD THIS: math32 uses this as a status flag

cseg
; These 'equ' must match the hardware wiring
; None of these are implemented yet, we need to match these assignments to the wiring
ELCD_RS equ P1.7
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
ELCD_E equ  P1.1
ELCD_D4 equ P0.7
ELCD_D5 equ P0.5
ELCD_D6 equ P0.3
ELCD_D7 equ P0.1

;Keypad pin assignments
ROW1 EQU P1.2
ROW2 EQU P1.4
ROW3 EQU P1.6
ROW4 EQU P2.0
COL1 EQU P2.2
COL2 EQU P2.4
COL3 EQU P2.6
COL4 EQU P3.0

;                           1234567890123456
blank_row:              db '                ', 0
preset_message:         db 'Presets: 1 2 3 4', 0
stop_to_skip_message:   db 'SELECT to skip. ', 0
preset_param_message1:  db 'sTMP:xxx sTME:xx', 0
preset_param_message2:  db 'rTMP:xxx rTME:xx', 0
param_message:          db 'Select Parameter', 0
soak_temp_message:      db 'Soak Temp: xxx C', 0
soak_time_message:      db 'Soak Time: xx  s', 0
reflow_temp_message:    db 'Rflw Temp: xxx C', 0
reflow_time_message:    db 'Rflw Time: xx  s', 0
ready_message:          db 'Ready to Start! ', 0
state0_message:         db '-----State0-----', 0
state1_message:         db '==Ramp To Soak==', 0
state2_message:         db '======Soak======', 0
state3_message:         db '==Ramp To Peak==', 0
state4_message:         db '=====Reflow=====', 0
state5_message:         db '====Cooling!====', 0
abortcondition_message: db '*****ABORT!*****', 0
reflow_message:         db 'TIME:XXXTEMP:XXX', 0
reflowdone_message:     db 'Reflow Complete!', 0
restart_message:        db 'RST 2 Bake Again', 0

stop_message:           db 'Reflow Stopped! ', 0

TimeLabel: db 'T','I','M','E',':', 0
TempLabel: db 'T','E','M','P',':', 0

$NOLIST
$include(LCD_4bit_DE10Lite_no_RW.inc) ; A library of LCD related functions and utility macros
$LIST

$NOLIST
$include(math32.asm)
;$include(Read_keypad.asm)
$LIST

;-----------------------INTIALIZE SERIAL PORT FOR INPUT OUTUUT-----------------------;
;--Setting baud rate to 57600 with 33.33MHz clock--;
;-----------EXPLANATION------------
;Crystal oscillates at 33.33Mhz, the CV-8052 has a fixed prescaler of 12 for timers
;So the effective clock for timers is 33.33MHz/12 = 2.7775MHzl
;SMOD is set to 1 in PCON so using 1/16th the clock for baud rate generation
;That means the baud rate clock is 2.7775MHz/16 = 173.611kHz
;Since we have 253 out of 256 its three clicks 
;per bit, the baud rate is 173.611kHz/3 = 57.870kbps which is close enough to 57600bps
;-----------------------------------

InitSerialPort:
	; Configure serial port and baud rate
    clr TR1 ; Disable timer 1
    mov a, TMOD
    anl a, #0x0f ; Clear the bits for timer 1
    orl a, #0x20 ; Configure timer 1 as 8-bit autoreload
    mov TMOD, a ; Set timer 1 mode

    mov TH1, #T1_LOAD ; Load the timer value for the desired baud rate
    mov TL1, #T1_LOAD ;Doesnt matter what we load in TL1 because it is in autoreload mode, but we need to load it with something to prevent it from overflowing immediately
    ;Leave it as you found it, make SMOD = 1 for double baud rate
    mov a, PCON ; Set SMOD to 1
    orl a, #0x80
    mov PCON, a
    setb TR1 ; Enable timer 1
    mov SCON, #01010010B ; Mode 1, 8-bit UART, enable receiver
	ret

Display_Voltage_Serial:
	mov x+0, TEMP+0 ; reloads the temp into x which will be converted to bcd
    mov x+1, TEMP+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd ; standard math32.asm function
    
	mov a, #'T'
	lcall putchar
	mov a, #'='
	lcall putchar
	
	mov a, bcd+1
	anl a, #0FH
	orl a, #'0'
	lcall putchar

	mov a, bcd+0
	swap a
	anl a, #0FH
	orl a, #'0'
	lcall putchar
	
	mov a, bcd+0
	anl a, #0FH
	orl a, #'0'
	lcall putchar

	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	
	ret



; Function to stransmit accumulator value into the serial buffer register after previous completion
putchar:
    jnb TI, putchar ; TI is the transmit interrupt, it will loop until it is high and we know the previous bit is sent
    
    clr TI  ; Reset back to 0 to indicate we are transmitting 
    mov SBUF, a ; accumulator will have output chharacter already stored on it
    ret

SendString:
    clr A
    movc A, @A+DPTR
    jz SendStringDone
    lcall putchar
    inc DPTR
    sjmp SendString
SendStringDone:
    ret


; Send number **** new*****
SendNum:
	mov a, @R0
	add a, #'0'
	lcall putchar
	ret

    Send_BCD mac
	push ar0
	mov r0, %0
	lcall ?Send_BCD
	pop ar0
endmac

?Send_BCD:
	push acc
	; Write most significant digits
	mov a, r0
	swap a
	anl a, #0fh
	orl a, #30h
	lcall putchar
	
	; write least significant digits
	mov a, r0
	anl a, #0fh
	orl a, #30h
	lcall putchar
	pop acc
	ret
    
	
SendTemp:
	
	Send_BCD(bcd+1)
	
	Send_BCD(bcd+0)
	
	mov DPTR, #TempSuf
	lcall SendString
	
	ret
	;Sends 
 
TempSuf:
	db	'\r', '\n', 0


; ******************************* TIMER ISRS ************************************

Timer0_Init:
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a

    mov T0_RH, #high(TIMER0_RELOAD)
    mov T0_RL, #low(TIMER0_RELOAD)

	mov TH0, T0_RH
	mov TL0, T0_RL
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz square wave at pin P1.5 ;
;---------------------------------;
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	mov TH0, T0_RH ; Timer 0 doesn't have autoreload in the CV-8052
	mov TL0, T0_RL
	cpl SOUND_OUT ; Connect speaker to P1.5
	reti

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;
Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	mov RCAP2H, #high(TIMER2_RELOAD)
	mov RCAP2L, #low(TIMER2_RELOAD)
	; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Enable the timer and interrupts
    setb ET2  ; Enable timer 2 interrupt
    setb TR2  ; Enable timer 2
	ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in ISR
	
	push acc
	push psw
	push b
	push dpl
	push dph
	push 0
	push 1
	push 2
	push 3
	push 4
	push 5
	push 6
	push 7
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1 ;increment high 8-bits if low 8-bits overflowed

Inc_Done:
	; Check if full second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), Timer2_ISR_Midpoint ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_Midpoint

    sjmp Timer2_ISR_Bypass
    
    
; below this move to subroutine **************************************

;	mov ADC_C, #00000000b
	

;    mov x+3, #0
;	mov x+2, #0
;	mov x+1, ADC_H
;	mov x+0, ADC_L
    ; Convert ADC reading to temperature in Celsius
    ; Voltage = (ADC_value * 5000) / 4096
;    Load_y(5000)
;	lcall mul32
;	Load_y(4096)
;	lcall div32
    ; Result is in 'x'

;    Load_y(1000) ; convert to microvolts
 ;   lcall mul32
 ;   Load_y(12300) ; 41 * 300
 ;   lcall div32

 ;   Load_y(22) ; add cold junction temperature
 ;   lcall add32
    ;do your displays and stuff
    ;result is still in x
;    mov TEMP+0, x+0
;    mov TEMP+1, x+1
;    mov TEMP+2, #0
    
    ;lcall Display_Voltage_Serial
 ;   lcall hex2bcd
;    lcall SendTemp
;    lcall Display_BCD_7_seg
;
;    sjmp Timer2_ISR_Bypass


; above here keep in isr *********************************************

Timer2_ISR_Midpoint:
ljmp Timer2_ISR_done
Timer2_ISR_Bypass:

;---------------------------------------------------------------------;
	
	;1 second have passed.  Set a flag so the main program knows
	setb seconds_flag ; Let the main program know one second had passed
	; Toggle LEDR0 so it blinks
    inc TIME ; Increment the TIME Variable
	cpl LEDRA.0
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
    
;check for abort
lcall CheckAbort_1s

    mov A, ABORT_FLAG
    jz  ContinuePWM

    clr P0.0                 ; force off
    ljmp STOPOVEN            ; skip all PWM logic

ContinuePWM:
    ;Call PWM funcions
Checkforstate1:
    MOV A, STATE_VAR_1
    CJNE A, #1, NOT_STATE_1
    LCALL RampBurstPWM
    SJMP PWM_EXIT
NOT_STATE_1:
    CJNE A, #3, NOT_STATE_3
    LCALL RampBurstPWM
    SJMP PWM_EXIT
NOT_STATE_3:
    CJNE A, #2, NOT_STATE_2
    LCALL pwm_for_flatstates
    SJMP PWM_EXIT
NOT_STATE_2:
    CJNE A, #4, PWM_EXIT     ; If not 4, do nothing and exit
    LCALL pwm_for_flatstates
PWM_EXIT:


    
	; Increment the BCD counter
	mov a, BCD_counter
	jb UPDOWN, Timer2_ISR_decrement
	add a, #0x01; Increment the BCD counter
	mov a, BCD_counter
	jb UPDOWN, Timer2_ISR_decrement
	add a, #0x01
	sjmp Timer2_ISR_da
Timer2_ISR_decrement:
	add a, #0x99 ; Adding the 10-complement of -1 is like subtracting 1.
Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov BCD_counter, a

	
Timer2_ISR_done:
	pop 7
	pop 6
	pop 5
	pop 4
	pop 3
	pop 2
	pop 1
	pop 0
	pop dph
	pop dpl
	pop b
	pop psw
	pop acc
	reti

INITIALIZE:

	mov P0MOD, #10101011b ; P0.0(OVEN_PIN), P0.1, P0.3, P0.5, P0.7(LCD) are outputs. 
    mov P1MOD, #11110111b ; P1.7, P1.5, P1.1(LCD), 1.2, 1.4, 1.6(ROW) are outputs, P1.0
    mov P2MOD, #00000001b ; output: 2.0(ROW) input: 2.2, 2.4, 2.6(COL)
    mov P3MOD, #00000010b ; input: 3.0 (COL), 3.2, 3.3, 3.4, 3.5, 3.7 out: 3.1
    ; for keypad, (ROWS as output-1)1.2, 1.4, 1.6, 2.0 - (COLS as input-0) 2.2, 2.4, 2.6, 3.0
    mov ADC_C, #0x00      ; Select ADC Channel 0
    mov ADC_C, #10000000b ; ADC Enable = 1 test******
    ret                   ; Added RET so it doesn't crash after initializing

; ************************** FUNCTIONS ***********************************

ADC_Bible:
    push acc
	push psw
	push b
	push dpl
	push dph
	push 0
	push 1
	push 2
	push 3
	push 4
	push 5
	push 6
	push 7


    mov a, #00000010b ;get the voltage reference port 2
	
	mov ADC_C, a
	lcall Wait50ms
	lcall Wait25ms
	
	
	mov a, ADC_H
	anl a, #0x0F ;mask the lower 4 bits
	mov ADCREFH, a
	mov ADCREFL, ADC_L
	
	;values stored in ADCREFL now read lm335
	mov a, #0x01 ;port for lm335
	mov ADC_C, a
	
	lcall Wait50ms
	lcall Wait25ms

	
	
	mov a, ADC_H
	anl a, #0x0F ;mask the bottom 4 bits
	mov LM335H, a
	mov LM335L, ADC_L
	
	;now all in variables start calculations
	
	;start calculations
	
	mov x+3, #0
	mov x+2, #0
	mov x+1, LM335H
	mov x+0, LM335L
	
	Load_y(4116) ;initially assumed it was 4096 this is wrong 
	lcall mul32
	
	;vref * ADClm335
	
	
	;----------------------------
	mov y+3, #0
	mov y+2, #0
	mov y+1, ADCREFH
	mov y+0, ADCREFL
	
	lcall div32
	;ok now x = Vlm335 this is adc value
	
	
	
	Load_y(2730)
	lcall sub32
	
	Load_y(10)
	lcall div32
	
	mov TCH, x+1
	mov TCL, x+0
	;TC is done here
	
	;read from thermocouple/op amp
	;Th calculations
	mov a, #0x00
	mov ADC_C, a
	
	lcall Wait50ms
	lcall Wait25ms
	
	;mask the top bits
	mov a, ADC_H
	anl a, #0x0F
	
	mov x+3, #0
	mov x+2, #0
	mov x+1, a
	mov x+0, ADC_L
	;op amp exists in x

	Load_y(333)
	lcall mul32
	
	
	mov y+3, #0
	mov y+2, #0
	mov y+1, ADCREFH
	mov y+0, ADCREFL
	
	lcall div32
	
	mov TH_H, x+1
	mov TH_L, x+0
	;Th done here
	;wonderful now we add Th+Tc
	
	
	mov y+3, #0
	mov y+2, #0
	mov y+1, TCH
	mov y+0, TCL
	
	lcall add32
	
	mov TEMP+0, x+0
	mov TEMP+1, x+1
	
	mov x+3, #0
	mov x+2, #0
	
	
    lcall hex2bcd
    lcall SendTemp
    lcall Display_BCD_7_seg
	lcall Wait50ms

    pop 7
	pop 6
	pop 5
	pop 4
	pop 3
	pop 2
	pop 1
	pop 0
	pop dph
	pop dpl
	pop b
	pop psw
	pop acc

    clr seconds_flag
    ret

;--------------------------------------end of bible-----------



; above this is ADC *********************************************************

Wait50ms:
;33.33MHz, 1 clk per cycle: 0.03us
	mov R0, #30
Wait50ms_L3:
	mov R1, #74
Wait50ms_L2:
	mov R2, #250
Wait50ms_L1:
	djnz R2, Wait50ms_L1 ;3*250*0.03us=22.5us
    djnz R1, Wait50ms_L2 ;74*22.5us=1.665ms
    djnz R0, Wait50ms_L3 ;1.665ms*30=50ms
    ret


; **************************** KEYPAD *******************************

myLUT:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F

showBCD MAC
	; Display LSD
    mov A, %0
    anl a, #0fh
    movc A, @A+dptr
    mov %1, A

	; Display MSD
    mov A, %0
    swap a
    anl a, #0fh
    movc A, @A+dptr
    mov %2, A
ENDMAC

Display:
	mov dptr, #myLUT
	$MESSAGE TIP: If digits 10, 9, 8, and 7 are not zero, LEDR7: on

	mov a, bcd+3
	orl a, bcd+4
	jz Display_L1
	setb LEDRA.7 ; Non-zero digits alert
	sjmp Display_L2

Display_L1:
	clr LEDRA.7

Display_L2:
	$MESSAGE TIP: Pressing KEY3, displays the most significant digits of the 10-digit number

	jnb key.3, Display_high_digits
	showBCD(bcd+0, HEX0, HEX1)
	showBCD(bcd+1, HEX2, HEX3)
	showBCD(bcd+2, HEX4, HEX5)

	sjmp Display_end

Display_high_digits:
	showBCD(bcd+3, HEX0, HEX1)
	showBCD(bcd+4, HEX2, HEX3)
	mov HEX4, #0xff	
	mov HEX5, #0xff	

Display_end:
    ret


MYRLC MAC
	mov a, %0
	rlc a
	mov %0, a
ENDMAC

Shift_Digits_Left:
	mov R0, #4 ; shift left four bits

    mov a, STATE_VAR_2
    cjne a, #0, KCheck_StateC

    mov a, keypad_digit_count
    cjne a, #3, Shift_Digits_Left_L0

    ret

KCheck_StateC:
    cjne a, #2, KCheck_Time_States
    mov a, keypad_digit_count
    cjne a, #3, Shift_Digits_Left_L0
    ret

KCheck_Time_States:
    mov a, keypad_digit_count
    cjne a, #2, Shift_Digits_Left_L0
    ret

Shift_Digits_Left_L0:
	clr c
	MYRLC(bcd+0)
	MYRLC(bcd+1)
	MYRLC(bcd+2)
	MYRLC(bcd+3)
	MYRLC(bcd+4)

	djnz R0, Shift_Digits_Left_L0
	; R7 has the new bcd digit	
	mov a, R7
	orl a, bcd+0
	mov bcd+0, a

    inc keypad_digit_count
    setb c
    
Shift_Digits_left_exit:
	ret

	
MYRRC MAC
	mov a, %0
	rrc a
	mov %0, a
ENDMAC

Shift_Digits_Right:
	mov R0, #4 ; shift right four bits

Shift_Digits_Right_L0:
	clr c
	MYRRC(bcd+4)
	MYRRC(bcd+3)
	MYRRC(bcd+2)
	MYRRC(bcd+1)
	MYRRC(bcd+0)

	djnz R0, Shift_Digits_Right_L0

    mov a, keypad_digit_count
    jz Shift_Digits_Right_Ret
    dec keypad_digit_count

    setb c

Shift_Digits_Right_Ret:
	ret


Wait25ms:
;33.33MHz, 1 clk per cycle: 0.03us
	mov R0, #15
LL3: mov R1, #74
LL2: mov R2, #250
LL1: djnz R2, LL1 ;3*250*0.03us=22.5us
     djnz R1, LL2 ;74*22.5us=1.665ms
     djnz R0, LL3 ;1.665ms*15=25ms

    ret



CHECK_COLUMN MAC
	jb %0, CHECK_COL_%M
	mov R7, %1
	jnb %0, $ ; wait for key release
	setb c
	ret
CHECK_COL_%M:
ENDMAC

Configure_Keypad_Pins:
	; Configure the row pins as output and the column pins as inputs
	orl P1MOD, #0b_01010100 ; P1.6, P1.4, P1.2 output
	orl P2MOD, #0b_00000001 ; P2.0 output
	anl P2MOD, #0b_10101011 ; P2.6, P2.4, P2.2 input
	anl P3MOD, #0b_11111110 ; P3.0 input
	ret

; This subroutine scans a 4x4 keypad.  If a key is pressed sets the carry
; to one and returns the key code in register R7.
; It works with both a default keypad or a modified keypad with the labels
; rotated 90 deg ccw.  The type of keypad is determined by SW0, which is bit SWA.0

Keypad:
	; First check the backspace/correction pushbutton.  We use KEY1 for this function.
	$MESSAGE TIP: STOP_BUTTON is the erase key

	jb STOP_BUTTON, keypad_L0
	lcall Wait25ms ; debounce
	jb STOP_BUTTON, keypad_L0

	jnb STOP_BUTTON, $ ; The key was pressed, wait for release
	lcall Shift_Digits_Right
    lcall bcd2hex

    mov a, STATE_VAR_2
    cjne a, #0, check_delete_b

    mov soak_temp_set+0, x+0
    mov soak_temp_set+1, x+1
    sjmp delete_done

check_delete_b:
    cjne a, #1, check_delete_c

    mov soak_time_set+0, x+0
    mov soak_time_set+1, x+1
    sjmp delete_done

check_delete_c:
    cjne a, #2, check_delete_d

    mov reflow_temp_set+0, x+0
    mov reflow_temp_set+1, x+1
    sjmp delete_done

check_delete_d:
    cjne a, #3, delete_done

    mov reflow_time_set+0, x+0
    mov reflow_time_set+1, x+1
    sjmp delete_done

delete_done:

	clr c
	ret

keypad_L0:

	; Make all the rows zero.  If any column is zero then a key is pressed.
	clr ROW1
	clr ROW2
	clr ROW3
	clr ROW4

	mov c, COL1
	anl c, COL2
	anl c, COL3
	anl c, COL4

	jnc Keypad_Debounce
	clr c
	ret


Keypad_Debounce:
	; A key maybe pressed.  Wait and check again to discard bounces.
	lcall Wait25ms ; debounce

	mov c, COL1
	anl c, COL2
	anl c, COL3
	anl c, COL4

	jnc Keypad_Key_Code
	clr c
	ret
	

Keypad_Key_Code:	
	; A key is pressed.  Find out which one by checking each possible column and row combination.

	setb ROW1
	setb ROW2
	setb ROW3
	setb ROW4

	; This check section is for an un-modified keypad

keypad_default:	

	; Check row 1	
	clr ROW1
	CHECK_COLUMN(COL1, #01H)
	CHECK_COLUMN(COL2, #02H)
	CHECK_COLUMN(COL3, #03H)
	CHECK_COLUMN(COL4, #0AH)

	setb ROW1

	; Check row 2	

	clr ROW2
	CHECK_COLUMN(COL1, #04H)
	CHECK_COLUMN(COL2, #05H)
	CHECK_COLUMN(COL3, #06H)
	CHECK_COLUMN(COL4, #0BH)

	setb ROW2

	; Check row 3	

	clr ROW3
	CHECK_COLUMN(COL1, #07H)
	CHECK_COLUMN(COL2, #08H)
	CHECK_COLUMN(COL3, #09H)
	CHECK_COLUMN(COL4, #0CH)

	setb ROW3

	; Check row 4	

	clr ROW4
	CHECK_COLUMN(COL1, #0EH)
	CHECK_COLUMN(COL2, #00H)
	CHECK_COLUMN(COL3, #0FH)
	CHECK_COLUMN(COL4, #0DH)

	setb ROW4

	clr c

	ret

	
; Look-up table for the 7-seg displays. (Segments are turn on with zero) 
T_7seg:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F


; Displays a BCD number in HEX1-HEX0
Display_BCD_7_Seg:
    push acc
    push psw

	mov x+0, TEMP+0
	mov x+1, TEMP+1
    mov x+2, #0
    mov x+3, #0
	lcall hex2bcd

	mov dptr, #T_7seg

    mov a, bcd+2
    swap a
    anl a, #0FH
    movc a, @a+dptr
    mov HEX5, a

    mov a, bcd+2
    anl a, #0FH
    movc a, @a+dptr
    mov HEX4, a

    mov a, bcd+1
    swap a
    anl a, #0FH
    movc a, @a+dptr
    mov HEX3, a

    mov a, bcd+1
    anl a, #0FH
    movc a, @a+dptr
    mov HEX2, a

	mov a, bcd
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX1, a

	mov a, bcd
	anl a, #0FH
	movc a, @a+dptr
	mov HEX0, a

    pop psw
    pop acc

	ret


Check_Select_Button_Press:
    jb SELECT_BUTTON, Not_Pressed
    lcall Wait50ms
    jb SELECT_BUTTON, Not_Pressed

    setb SELECT_BUTTON_FLAG

    jnb SELECT_BUTTON, $

    Not_Pressed:
        ret


Check_Param_Button_Press:
    jb PARAM_BUTTON, Not_Pressed_2
    lcall Wait50ms
    jb PARAM_BUTTON, Not_Pressed_2

    setb PARAM_BUTTON_FLAG

    jnb PARAM_BUTTON, $

    Not_Pressed_2:
        ret

    ;---------------READ KEYPAD-------------------;

    ;A Macro essentially works like CHECK_COL(COL#, Literal value coloumn represents)
    ;If the column is pressed, R7 will contain the column number (1-4)
    

;**************************PWM**************************;
;check for abort button
CheckAbort_1s:
    jnb RI, AbortDone        ; no received byte
    mov A, SBUF              ; read received byte
    clr RI                   ; clear receive flag

    cjne A, #ABORT_TOKEN, AbortDone
    mov ABORT_FLAG, #01h     ; latch abort
AbortDone:
    ret

pwm_for_flatstates:
; ---- LOW_LIM = max(0, T_TGT - T_BAND)
        MOV     A, TARGET          
        CLR     C                 ;clear carry
        SUBB    A, #BAND         ; A = A - BAND
        JNC     flat_low_ok       
        MOV     A, #00h           
flat_low_ok:
        MOV     LOW_LIMIT, A        ; Store low limit in RAM

        ;compute high limit
        MOV     A, TARGET          
        ADD     A, #BAND         
        JNC     flat_high_ok      
        MOV     A, #0FFh          
flat_high_ok:
        MOV     HIGH_LIMIT, A       

        ;turn oven on if curren temp is less than low limit
        MOV     A, TEMP          ;make sure this variables is right!!!!!!!
        CLR     C               
        SUBB    A, LOW_LIMIT       
                                 
        JC      flat_on       ;temp is less than low limit so turn power on since there is carry

        ;if current temp is greater than high lim turn off
        MOV     A, TEMP
        CLR     C                 
        SUBB    A, HIGH_LIMIT       
                                 
        JZ      flat_done         ; If equal to HIGH_LIMit do nothing
        JNC     flat_off      ; If no borrow and not zero T_CUR > HIGH_LIM so turn off

flat_done:
        RET                       ; Inside band do nothing, holds prev values
flat_on:
        SETB p0.0      ;turn power on
        RET

flat_off:
        CLR p0.0      ;power off
        RET



RampBurstPWM:

        ;increment phase until it equals window then pahse =0
        MOV     A, PHASE          
        INC     A                 ; A <- A + 1
        CJNE    A, #WINDOW, phase_ok  ; if A != WINDOW, keep it
        MOV     A, #00h           ; else wrap to 0
phase_ok:
        MOV     PHASE, A          ; store updated phase 

        ; calculate threasheld = target - lead
        MOV     A, TARGET         ; A <- TARGET
        CLR     C                 
        SUBB    A, LEAD           
        JNC     thresh_ok        
        MOV     A, #00h           
thresh_ok:
        MOV     THRESHOLD, A         

        ;if temp is less than threshold, full power
        MOV     A, TEMP           
        CLR     C
        SUBB    A, THRESHOLD         
        JC      force_on         

        ; if temp is greater than target then off
        MOV     A, TEMP           
        CLR     C
        SUBB    A, TARGET         
        JNC     force_off         

        ;else burst pwm 
        ; CMD_ON = 1 if PHASE < ON_SECS else 0
        MOV     A, PHASE          ; A <- PHASE (0..WINDOW-1)
        CLR     C
        SUBB    A, ON_SECS        ; A <- PHASE - ON_SECS
                                 ; borrow => PHASE < ON_SECS
        JC      burst_on          ; if PHASE < ON_SECS => ON
        SJMP    burst_off         ; else OFF

force_on:
        SETB p0.0      ;less than threshold
        RET

force_off:
        CLR p0.0      
        RET

burst_on:
        SETB p0.0      ; ON for ON_SECS seconds each window
        RET

burst_off:
        CLR p0.0      ; OFF for remaining seconds
        RET




;==============SPEAKER FUNCTIONS==============;

BeepSpeaker:
    setb TR0
    mov R3, #7
WaitLoop:
    lcall Wait50ms
    djnz R3, WaitLoop 
UnbeepSpeaker:
    clr TR0
    ret
PlaySong:
    setb TR0 ; Start timer
    
    ; Note 1: C - E - G - C arpeggio
    
    ; Note 1: C5 (523Hz) -> F5A2
    mov T0_RH, #0xF5
    mov T0_RL, #0xA2
    mov R3, #4 ; Duration
    lcall Song_Delay
    
    ; Note 2: E5 (659Hz) -> F7C5
    mov T0_RH, #0xF7
    mov T0_RL, #0xC5
    mov R3, #4
    lcall Song_Delay

    ; Note 3: G5 (784Hz) -> F915
    mov T0_RH, #0xF9
    mov T0_RL, #0x15
    mov R3, #4
    lcall Song_Delay
    
    ; Note 4: C6 (1046Hz) -> FAD1
    mov T0_RH, #0xFA
    mov T0_RL, #0xD1
    mov R3, #15 ; Longer duration
    lcall Song_Delay

    clr TR0 ; Stop timer
    
    ; Restore default beep pitch
    mov T0_RH, #high(TIMER0_RELOAD)
    mov T0_RL, #low(TIMER0_RELOAD)
    ret

Song_Delay:
    lcall Wait50ms
    djnz R3, Song_Delay
    ret


;=============================================;

FSM2_Temp_Time_display:
    ;print time
    Set_Cursor(2,1)
    Send_Constant_String(#TimeLabel)

    mov x+0, TIME+0
    mov x+1, TIME+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,6)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    ;print temp
    Set_Cursor(2,10)
    Send_Constant_String(#TempLabel)

    mov x+0, TEMP+0
    mov x+1, TEMP+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,16-3)      
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    ret

;--- MAIN PROGRAM START ---

; **************************************** Initializations ***********************************************

MAIN:
    mov SP, #0x7F         ; Initialize Stack Pointer (Good practice)
    lcall INITIALIZE      ; intialize pins and adc, for now

    lcall Timer0_Init
    lcall Timer2_Init
    setB EA ; Enable global interrupts
    lcall ELCD_4BIT ; Intialize LCD
    lcall InitSerialPort
    
    clr seconds_flag
    clr START_FLAG
    clr KEYPAD_FLAG
    clr p0.0 ; Make sure oven is off to start
    clr TR0 ; Start speaker off
    clr a
    clr RED_LED
    clr GREEN_LED


    mov STATE_VAR_1, #0x0000
    mov STATE_VAR_2, #0x0000
    mov TIME, #0
    mov TEMP, #0000
    mov POWER, #0
    mov DEGREES60, #60
    mov DEGREES150, #150
    mov DEGREES220, #220
    mov TARGET,       #0

    mov bcd, #0x0000
    mov x+0, #0x0000
    mov x+1, #0x0000
    mov x+2, #0x0000
    mov x+3, #0x0000

    mov soak_temp_set+0, #150
    mov soak_temp_set+1, a

    mov soak_time_set+0, #60
    mov soak_time_set+1, a

    mov reflow_temp_set+0, #220
    mov reflow_temp_set+1, a

    mov reflow_time_set+0, #45
    mov reflow_time_set+1, a


MAIN_LOOP:

; **************************** Preset Settings **************************************

; **************************** Preset Settings **************************************

StatePInit:
    Set_Cursor(1,1)
    Send_Constant_String (#preset_message)
	Set_Cursor(2,1)
	Send_Constant_String (#stop_to_skip_message)

    clr SELECT_BUTTON_FLAG

StateP:
    lcall Check_Select_Button_Press

    jb PRESET1_SW, Presetx1
    jb PRESET2_SW, Presetx2
    jb PRESET3_SW, Presetx3
    jb PRESET4_SW, P0P4
    jb SELECT_BUTTON_FLAG, GoToParam

    jb seconds_flag, P0ADC

sjmp StateP

P0ADC:
lcall ADC_Bible
sjmp StateP

GoToParam:
lcall BeepSpeaker
ljmp PARAM_FSM

P0P4:
ljmp Presetx4

Presetx1:
	mov soak_temp_set, #170
	mov soak_time_set, #75
	mov reflow_temp_set, #230
	mov reflow_time_set, #45

	lcall BeepSpeaker
    lcall Preset_Displays

Preset1_Display:
    lcall Check_Select_Button_Press
	jb SELECT_BUTTON_FLAG, P1GT0
	jnb PRESET1_SW, P1GTI

    jb seconds_flag, P1ADC

	sjmp Preset1_Display

P1GTI:
lcall BeepSpeaker
ljmp StatePInit

P1GT0:
lcall BeepSpeaker
ljmp ReadyStateInit

P1ADC:
lcall ADC_Bible
sjmp Preset1_Display

Presetx2:
	mov soak_temp_set, #160
	mov soak_time_set, #90
	mov reflow_temp_set, #230
	mov reflow_time_set, #60

	lcall BeepSpeaker
    lcall Preset_Displays


Preset2_Display:
    lcall Check_Select_Button_Press
	jb SELECT_BUTTON_FLAG, P2GT0
	jnb PRESET2_SW, P2GTI

    jb seconds_flag, P2ADC

	sjmp Preset2_Display

P2GTI:
lcall BeepSpeaker
ljmp StatePInit

P2GT0:
lcall BeepSpeaker
ljmp ReadyStateInit

P2ADC:
lcall ADC_Bible
sjmp Preset2_Display

Presetx3:
	mov soak_temp_set, #150
	mov soak_time_set, #95
	mov reflow_temp_set, #225
	mov reflow_time_set, #60

	lcall BeepSpeaker
    lcall Preset_Displays
Preset3_Display:
    lcall Check_Select_Button_Press
	jb SELECT_BUTTON_FLAG, P3GT0
	jnb PRESET3_SW, P3GTI

    jb seconds_flag, P3ADC

	sjmp Preset3_Display

P3GTI:
lcall BeepSpeaker
ljmp StatePInit

P3GT0:
lcall BeepSpeaker
ljmp ReadyStateInit

P3ADC:
lcall ADC_Bible
ljmp Preset3_Display

Presetx4:
	mov soak_temp_set, #180
	mov soak_time_set, #60
	mov reflow_temp_set, #235
	mov reflow_time_set, #30

	lcall BeepSpeaker
    lcall Preset_Displays

Preset4_Display:
    lcall Check_Select_Button_Press
	jb SELECT_BUTTON_FLAG, P4GT0
	jnb PRESET4_SW, P4GTI

    jb seconds_flag, P4ADC

	sjmp Preset4_Display

P4GTI:
lcall BeepSpeaker
ljmp StatePInit

P4GT0:
lcall BeepSpeaker
ljmp ReadyStateInit

P4ADC:
lcall ADC_Bible
ljmp Preset4_Display


Preset_Displays:

    push acc
    push psw

	Set_Cursor(1,1)
	Send_Constant_String(#preset_param_message1)
	Set_Cursor(2,1)
	Send_Constant_String(#preset_param_message2)

	mov x+0, soak_temp_set+0
	mov x+1, soak_temp_set+1
	lcall hex2bcd
    ;Set_Cursor(1,5)
	;Send_Constant_String(#':')
	Set_Cursor(1,6)
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	

	mov x+0, soak_time_set+0
	lcall hex2bcd
	Set_Cursor(1,15)
	Display_BCD(bcd+0)

	mov x+0, reflow_temp_set+0
	mov x+1, reflow_temp_set+1
	lcall hex2bcd
    ;Set_Cursor(2,5)
	;Send_Constant_String(#':')
	Set_Cursor(2,6)
	Display_BCD(bcd+1)
	Display_BCD(bcd+0)
	

	mov x+0, reflow_time_set+0
	lcall hex2bcd
	Set_Cursor(2,15)
	Display_BCD(bcd+0)

    pop psw
    pop acc

	ret


PARAM_FSM:

; **************************** FSM for selecting parameters *************************

; 4 main states ->  A: select soak temp
;                   B: select soak time
;                   C: select reflow temp
;                   D: select reflow time
; move to other FSM when start button turns on start flag

StateAInit:
    Set_Cursor(1,1)
    Send_Constant_String (#param_message)
    Set_Cursor(2,1)
    Send_Constant_String (#soak_temp_message)

    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0

StateA:
    jnb RESET_BUTTON, StateA_ResetToMain
    mov R6, STATE_VAR_2
    cjne R6, #0, StateA_B

    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateAtoDone

    jb seconds_flag, AtoADC

    mov x+0, soak_temp_set+0
    mov x+1, soak_temp_set+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    lcall Check_Param_Button_Press
    jb PARAM_BUTTON_FLAG, Inc_Soak_Temp
    sjmp StateA_Keypad

StateA_ResetToMain:
ljmp MAIN   

StateA_B:
ljmp StateBInit

StateAtoDone:
ljmp StateADone

AtoADC:
lcall ADC_Bible
ljmp StateA

StateA_Keypad:
    lcall Keypad
    jnc StateA

    jb KEYPAD_FLAG, StateA_Keypad_Continue
    setb KEYPAD_FLAG

    mov a, #0
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a

StateA_Keypad_Continue:
    lcall Shift_Digits_Left
    jnc StateA

    lcall bcd2hex

    mov soak_temp_set+0, x+0
    mov soak_temp_set+1, x+1

    mov x+0, soak_temp_set+0
    mov x+1, soak_temp_set+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    ljmp StateA


    Inc_Soak_Temp:
        clr PARAM_BUTTON_FLAG

        mov a, soak_temp_set

        jb UPDOWN, Dec_Soak_Temp
        jb TENS, Inc_Soak_Temp_Tens

        add a, #1
        sjmp Soak_Temp_Tens_Done

    Inc_Soak_Temp_Tens:
        add a, #10
        sjmp Soak_Temp_Tens_Done

    Dec_Soak_Temp:
        jb TENS, Dec_Soak_Temp_Tens

        clr c
        subb a, #1
        sjmp Soak_Temp_Tens_Done

    Dec_Soak_Temp_Tens:
        clr c  
        subb a, #10
        sjmp Soak_Temp_Tens_Done

    Soak_Temp_Tens_Done:
        mov soak_temp_set, a
        ljmp StateA

StateADone:
    lcall BeepSpeaker
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0
    ljmp StateA


StateBInit:
    Set_Cursor(2,1)
    Send_Constant_String(#soak_time_message)

    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0

StateB:
    jnb RESET_BUTTON, StateB_ResetToMain
    mov R6, STATE_VAR_2
    cjne R6, #1, StateB_C

    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateBtoDone

    jb seconds_flag, BtoADC

    mov x+0, soak_time_set+0
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+0)

    lcall Check_Param_Button_Press
    jb PARAM_BUTTON_FLAG, Inc_Soak_Time
    ljmp StateB_Keypad

StateB_ResetToMain:
ljmp MAIN

StateB_C:
ljmp StateCInit

StateBtoDone:
ljmp StateBDone

BtoADC:
lcall ADC_Bible
ljmp StateB

StateB_Keypad:
    lcall Keypad
    jnc StateB

    jb KEYPAD_FLAG, StateB_Keypad_Continue
    setb KEYPAD_FLAG

    mov a, #0
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a

StateB_Keypad_Continue:
    lcall Shift_Digits_Left
    jnc StateB

    lcall bcd2hex

    mov soak_time_set+0, x+0

    mov x+0, soak_time_set+0
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+0)

    ljmp StateB


    Inc_Soak_Time:
        clr PARAM_BUTTON_FLAG

        mov a, soak_time_set

        jb UPDOWN, Dec_Soak_Time
        jb TENS, Inc_Soak_Time_Tens

        add a, #1
        sjmp Soak_Time_Tens_Done

    Inc_Soak_Time_Tens:
        add a, #10
        sjmp Soak_Time_Tens_Done

    Dec_Soak_Time:
        jb TENS, Dec_Soak_Time_Tens
        clr c

        subb a, #1
        sjmp Soak_Time_Tens_Done

    Dec_Soak_Time_Tens:
        clr c

        subb a, #10
        sjmp Soak_Time_Tens_Done

    Soak_Time_Tens_Done:
        mov soak_time_set, a
        ljmp StateB

StateBDone:
    lcall BeepSpeaker
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0
    ljmp StateB


StateCInit:
    Set_Cursor(2,1)
    Send_Constant_String(#reflow_temp_message)

    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0

StateC:
    jnb RESET_BUTTON, StateC_ResetToMain
    mov R6, STATE_VAR_2
    cjne R6, #2, StateC_D

    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateCtoDone

    jb seconds_flag, CtoADC

    mov x+0, reflow_temp_set+0
    mov x+1, reflow_temp_set+1
    mov x+2, #0x0000
    mov x+3, #0x0000

    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    lcall Check_Param_Button_Press
    jb PARAM_BUTTON_FLAG, Inc_Reflow_Temp
    ljmp StateC_Keypad

StateC_ResetToMain:
ljmp MAIN

StateC_D:
ljmp StateDInit

StateCtoDone:
ljmp StateCDone

CtoADC:
lcall ADC_Bible
ljmp StateC

StateC_Keypad:
    lcall Keypad
    jnc StateC

    jb KEYPAD_FLAG, StateC_Keypad_Continue
    setb KEYPAD_FLAG

    mov a, #0
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a

StateC_Keypad_Continue:
    lcall Shift_Digits_Left
    jnc StateC

    lcall bcd2hex

    mov reflow_temp_set+0, x+0
    mov reflow_temp_set+1, x+1

    mov x+0, reflow_temp_set+0
    mov x+1, reflow_temp_set+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    ljmp StateC


    Inc_Reflow_Temp:
        clr PARAM_BUTTON_FLAG

        mov a, reflow_temp_set

        jb UPDOWN, Dec_Reflow_Temp
        jb TENS, Inc_Reflow_Temp_Tens

        add a, #1
        sjmp Reflow_Temp_Tens_Done

    Inc_Reflow_Temp_Tens:
        add a, #10
        sjmp Reflow_Temp_Tens_Done

    Dec_Reflow_Temp:
        jb TENS, Dec_Reflow_Temp_Tens

        clr c
        subb a, #1
        sjmp Reflow_Temp_Tens_Done

    Dec_Reflow_Temp_Tens:
        clr c
        subb a, #10
        sjmp Reflow_Temp_Tens_Done

    Reflow_Temp_Tens_Done:
        mov reflow_temp_set, a
        ljmp StateC

StateCDone:
    lcall BeepSpeaker
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0
    ljmp StateC


StateDInit:
    Set_Cursor(2,1)
    Send_Constant_String(#reflow_time_message)

    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0

StateD:
    jnb RESET_BUTTON, StateD_ResetToMain
    mov R6, STATE_VAR_2
    cjne R6, #3, StateD_R

    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateDtoDone

    jb seconds_flag, DtoADC

    mov x+0, reflow_time_set+0
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    Display_BCD(bcd+0)

    lcall Check_Param_Button_Press
    jb PARAM_BUTTON_FLAG, Inc_Reflow_Time
    sjmp StateD_Keypad

StateD_ResetToMain:
ljmp MAIN

StateD_R:
ljmp ReadyStateInit

StateDtoDone:
ljmp StateDDone

DtoADC:
lcall ADC_Bible
ljmp StateD

StateD_Keypad:
    lcall Keypad 
    jnc StateD

    jb KEYPAD_FLAG, StateD_Keypad_Continue
    setb KEYPAD_FLAG

    mov a, #0
    mov bcd+0, a
    mov bcd+1, a
    mov bcd+2, a
    mov bcd+3, a

StateD_Keypad_Continue:
    lcall Shift_Digits_Left
    jnc StateD

    lcall bcd2hex

    mov reflow_time_set+0, x+0
    ;mov reflow_time_set+1, x+1

    mov x+0, reflow_time_set+0
    mov x+1, #0
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd

    Set_Cursor(2,12)
    ;Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    ljmp StateD


    Inc_Reflow_Time:        
        clr PARAM_BUTTON_FLAG
        mov a, reflow_time_set

        jb UPDOWN, Dec_Reflow_Time
        jb TENS, Inc_Reflow_Time_Tens

        add a, #1
        sjmp Reflow_Time_Tens_Done

    Inc_Reflow_Time_Tens:
        add a, #10
        sjmp Reflow_Time_Tens_Done

    Dec_Reflow_Time:
        jb TENS, Dec_Reflow_Time_Tens

        clr c
        subb a, #1
        sjmp Reflow_Time_Tens_Done

    Dec_Reflow_Time_Tens:
        clr c
        subb a, #10
        sjmp Reflow_Time_Tens_Done

    Reflow_Time_Tens_Done:
        mov reflow_time_set, a
        ljmp StateD

StateDDone:
    lcall BeepSpeaker
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0
    ljmp StateD


ReadyStateInit:
    Set_Cursor(1,1)
    Send_Constant_String(#ready_message)
    Set_Cursor(2,1)
    Send_Constant_String(#blank_row)

    clr SELECT_BUTTON_FLAG
    clr PARAM_BUTTON_FLAG
    clr KEYPAD_FLAG
    mov keypad_digit_count, #0

ReadyState:
    jb seconds_flag, RtoADC

    jb START_BUTTON, ReadyState
    lcall wait50ms
    jb START_BUTTON, ReadyState

    Set_Cursor(1,1)
    Send_Constant_String (#state0_message)

    setb START_FLAG
    
    sjmp State0

RtoADC:
lcall ADC_Bible
sjmp ReadyState

;==================Reflow Profile FSM==================;
;Checklist:
; 1. Implement TEMP and TIME variables - DONE
; 2. Implement FSM outputs - DONE
; 3. Implement reset logic - DONE
; 4. Implement abort condition - DONE
; 5. Implement LCD Feedback for Each State - Tentatively Done (Still not tested)
; 6. Speaker beeps for state transitions - DONE
;*Abort condition needs to be in state 1* - FIXED
; 7. Rewrite State Transitions with SUBB - DONE
State0:
    jnb STOP_BUTTON, State0_StopReflow
    CLR p0.0 ;oven off
    mov a, STATE_VAR_1
    mov TIME, #0
    cjne a, #0, State1
    jb START_FLAG, State0Done
    sjmp State0

State0_StopReflow:
ljmp StopReflow

State0Done:
    lcall BeepSpeaker
    MOV PHASE, #00h
    MOV ON_SECS, #4
    MOV LEAD, #10
    inc STATE_VAR_1
    mov POWER, #100
    mov TIME, #0
    sjmp State0
State1:
    jnb RESET_BUTTON, State1_ResetToMain
    jnb STOP_BUTTON, State1_StopReflow
    jb seconds_flag, State1_ADC
    mov a, STATE_VAR_1
    cjne a, #1, State2
    Set_Cursor(1,1)
    Send_Constant_String (#state1_message)
    lcall FSM2_Temp_Time_display
    lcall CheckAbortCondition ;Must abort if 50 degrees isn't reached in first 60 seconds

    mov TARGET, SOAK_TEMP_set
    mov a, TEMP
    clr c
    subb a, TARGET
    jc CheckCarryState1 ; C = 1, Keep heating
    ljmp GreaterThanState1 ; C = 0, Transition States

State1_ADC:
    lcall ADC_Bible
    sjmp State1

State1_ResetToMain:
ljmp ResetToMain

State1_StopReflow:
ljmp StopReflow

CheckCarryState1:
    jc LessThanState1
    sjmp GreaterThanState1
LessThanState1:
    sjmp State1
GreaterThanState1:
    lcall BeepSpeaker
    inc STATE_VAR_1
    mov POWER, #20
    mov TIME, #0
    sjmp State1
State2:
    jnb RESET_BUTTON, State2_ResetToMain
    jnb STOP_BUTTON, State2_StopReflow
    jb seconds_flag, State2_ADC
    mov a, STATE_VAR_1
    cjne a, #2, State2_State3
    Set_Cursor(1,1)
    Send_Constant_String (#state2_message)
    lcall FSM2_Temp_Time_display
    mov TARGET_TIME, SOAK_TIME_set
    mov a, TIME
    clr c
    subb a, TARGET_TIME
    jc CheckCarryState2 ; C = 1, TIME < TARGET_TIME
    ljmp GreaterThanState2 ; C = 0, TIME > TARGET_TIME -> transition states

State2_ADC:
    lcall ADC_Bible
    sjmp State2

State2_State3:
    ljmp State3

State2_ResetToMain:
ljmp ResetToMain

State2_StopReflow:
ljmp StopReflow

State1_CheckOven:
mov a, TIME
cjne a, #60, State1TimeCheckCarry

State1TimeCheckCarry:
jc TrampolineState1 ; 60 Seconds haven't passed
ljmp STOPOVEN

TrampolineState1:
ret

CheckAbortCondition:
    mov a, TEMP
    cjne a, #50, CheckAbortCarry
    ret
CheckAbortCarry:
    jc State1_CheckOven          
    ret
CheckCarryState2:
    jc LessThanState2
    sjmp GreaterThanState2
LessThanState2:
    sjmp State2
GreaterThanState2:
    lcall BeepSpeaker
    MOV PHASE, #00h
    MOV ON_SECS, #4
    MOV LEAD, #10
    inc STATE_VAR_1
    mov POWER, #100
    ljmp State2
State3:
    jnb RESET_BUTTON, State3_ResetToMain
    jnb STOP_BUTTON, State3_StopReflow
    jb seconds_flag, State3_ADC
    mov a, STATE_VAR_1
    cjne a, #3, State4
    Set_Cursor(1,1)
    Send_Constant_String (#state3_message)
    lcall FSM2_Temp_Time_display
    mov TARGET, reflow_temp_set
    mov a, TEMP
    clr c
    subb a, TARGET
    jc CheckCarryState3 ; C = 1, TEMP < TARGET -> Keep Heating
    ljmp GreaterThanState3 ; C = 0, TEMP > TARGET -> Transition States

State3_ADC:
    lcall ADC_Bible
    sjmp State3    

State3_ResetToMain:
ljmp ResetToMain

State3_StopReflow:
ljmp StopReflow

CheckCarryState3:
    jc LessThanState3
    sjmp GreaterThanState3
LessThanState3:
    sjmp State3
GreaterThanState3:
    lcall BeepSpeaker
    inc STATE_VAR_1
    mov POWER, #20
    mov TIME, #0
    sjmp State3
State4:
    jnb RESET_BUTTON, ResetToMainState4
    jnb STOP_BUTTON, StopReflowState4
    jb seconds_flag, State4_ADC
    mov a, STATE_VAR_1
    cjne a, #4, State5
    Set_Cursor(1,1)
    Send_Constant_String (#state4_message)
    lcall FSM2_Temp_Time_display
    mov TARGET_TIME, REFLOW_TIME_set
    mov a, TIME
    clr c
    subb a, TARGET_TIME
    jc CheckCarryState4 ; C = 1, TIME < TARGET_TIME -> Keep Going
    ljmp GreaterThanState4 ; C = 0, TIME > TARGET_TIME -> Transition States
State4_ADC:
    lcall ADC_Bible
    sjmp State4
StopReflowState4:
    ljmp StopReflow
ResetToMainState4:
    ljmp ResetToMain
CheckCarryState4:
    jc LessThanState4
    sjmp GreaterThanState4
LessThanState4:
    sjmp State4 
GreaterThanState4:
    lcall BeepSpeaker
    inc STATE_VAR_1
    mov POWER, #0
    sjmp State4

State5:
    jnb RESET_BUTTON, ResetToMainState5
    jnb STOP_BUTTON, StopReflowState5
    jb seconds_flag, State5_ADC
    CLR OVEN_PIN ;turn oven off
    mov a, STATE_VAR_1    
    cjne a, #5, State5toDone
    mov TARGET, DEGREES60
    mov a, TEMP
    Set_Cursor(1,1)
    Send_Constant_String (#state5_message)
    lcall FSM2_Temp_Time_display
    cjne a, #60, CheckCarryState5
    sjmp State5

State5_ADC:
    lcall ADC_Bible
    sjmp State5

ResetToMainState5:
    ljmp ResetToMain

StopReflowState5:
    ljmp StopReflow
    
State5toDone:
    lcall BeepSpeaker
    Set_Cursor(1,1)
    Send_Constant_String (#reflowdone_message)
    Set_Cursor(2,1)
    Send_Constant_String (#restart_message)
    ljmp ReflowDone

CheckCarryState5:
    jc LessThanState5
    sjmp GreaterThanState5
LessThanState5:
    mov STATE_VAR_1, #0
    clr START_FLAG
    ljmp State5
GreaterThanState5:
    ljmp State5

ResetToMain:
    mov STATE_VAR_1, #0
    mov POWER, #0
    ljmp MAIN

StopReflow:
    setb RED_LED
    Set_Cursor(1,1)
    Send_Constant_String (#stop_message)
    Set_Cursor(2,1)
    Send_Constant_String (#restart_message)
    lcall BeepSpeaker
    clr OVEN_PIN ; Turn power off
    clr EA
ForeverStopped:
    jnb RESET_BUTTON, RestartProcess
    sjmp ForeverStopped

STOPOVEN:
    setb RED_LED
    Set_Cursor(1,1)
    Send_Constant_String (#abortcondition_message)
    Set_Cursor(2,1)
    Send_Constant_String (#restart_message)
    clr EA
    mov R4, #10
STOPOVENSpeakerLoop:
    lcall BeepSpeaker
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    djnz R4, STOPOVENSpeakerLoop
ClearInterrupts:
    clr EA
    clr p0.0
ForeverStop:
    jnb RESET_BUTTON, RestartProcess
    sjmp ForeverStop ; Infinite loop to stop the oven if abort condition is met

RestartProcess:
    ljmp MAIN
    
ReflowDone:
    setb GREEN_LED
    mov R4, #5
    mov R7, #5
SpeakerLoop:
    lcall BeepSpeaker
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    djnz R4, SpeakerLoop
DelaySpeakerSong:
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
    lcall Wait50ms
SongLoop:
    lcall PlaySong
    lcall PlaySong
    lcall PlaySong
    lcall PlaySong
    lcall PlaySong
Forever:
    jnb RESET_BUTTON, ForeverToMain
    sjmp Forever
ForeverToMain:
	ljmp ResetToMain
END
