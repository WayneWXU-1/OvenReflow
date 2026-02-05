$NOLIST
$MODMAX10
$LIST

CLK           EQU 33333333 ; Microcontroller system crystal frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.
TIMER2_RATE   EQU 10000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/(12*TIMER2_RATE))))

SOUND_OUT     equ P1.5
UPDOWN        equ SWA.0
INC_TENS      equ SWA.1

; Reset vector
org 0x0000
    ljmp main

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
STATE_VAR:       DS 1 
SECOND_COUNTER:  DS 1 
TEMP_HIGH_BYTE:  DS 1 
TEMP_LOW_BYTE:   DS 1 

;*** Variables ***
SOAK_TEMP_set       ds 1
SOAK_TIME_set       ds 1
reflow_temp_set     ds 1
REFLOW_TIME_set     ds 1

soak_time       ds 1
REFLOW_TIME     ds 1

; PWM variables
Temp_measured ds 2
Temp_target ds 2
error ds 2
Integral_accumulator ds 2
duty ds 1
duty_max ds 1

x:		ds	4 ;used for 32 bit math for temperature conversion
y:		ds	4 ;used for 32 bit math for temperature conversion

;--ISR RELATED--;
Count1ms:     ds 2 ; Used to determine when half second has passed
BCD_counter:  ds 1 ; The BCD counter incrememted in the ISR and displayed in the main loop

bseg
START_FLAG:         DBIT 1  ; Use DBIT for single bits in bseg
half_seconds_flag   DBIT 1  ; half second flag
SECONDS_FLAG:       DBIT 1  ; can change later depending on how fast we want it

cseg
; These 'equ' must match the hardware wiring
; None of these are implemented yet, we need to match these assignments to the wiring
LCD_RS equ PX.X
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
LCD_E equ  PX.X
LCD_D4 equ PX.X
LCD_D5 equ PX.X
LCD_D6 equ PX.X
LCD_D7 equ PX.X

$NOLIST
$include(LCD_4bit_DE10Lite_no_RW.inc) ; A library of LCD related functions and utility macros
$LIST

$include(math32.asm)

;-INTIALIZE SERIAL PORT FOR INPUT OUTUUT--;
;--Setting baud rate to 115200 with 33.33MHz clock--;
InitSerialPort:
	; Configure serial port and baud rate
	mov TMOD, #00100001B ; TIMER 1 in mode 2 autoreload as timer and 
    
    
	ret


; ******************************* TIMER ISRS ************************************

Timer0_Init:
	mov a, TMOD
	anl a, #0xf0 ; Clear the bits for timer 0
	orl a, #0x01 ; Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz square wave at pin P3.7 ;
;---------------------------------;
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	mov TH0, #high(TIMER0_RELOAD) ; Timer 0 doesn't have autoreload in the CV-8052
	mov TL0, #low(TIMER0_RELOAD)
	cpl SOUND_OUT ; Connect speaker to P3.7!
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
	cpl P1.1 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1

Inc_Done:
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(500), Timer2_ISR_done ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(500), Timer2_ISR_done
	
	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb half_seconds_flag ; Let the main program know half second had passed
	; Toggle LEDR0 so it blinks
	cpl LEDRA.0
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Increment the BCD counter
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
	pop psw
	pop acc
	reti

INITIALIZE:
    mov P0MOD, #0x01      ; CRITICAL: Set P0.0 as output so OVEN_PIN works
    mov ADC_C, #0x00      ; Select ADC Channel 0
    ret                   ; Added RET so it doesn't crash after initializing

; ************************** FUNCTIONS ***********************************

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


display_params_lcd:
    
    

    ret

; Look-up table for the 7-seg displays. (Segments are turn on with zero) 
T_7seg:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F

; Displays a BCD number in HEX1-HEX0
Display_BCD_7_Seg:
	
	mov dptr, #T_7seg

	mov a, BCD_counter
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX1, a
	
	mov a, BCD_counter
	anl a, #0FH
	movc a, @a+dptr
	mov HEX0, a
	
	ret

READ_TEMPERATURE:
    ; Start ADC conversion
    setb ADC_CONTR.7 ; Set ADC start bit SNAPSHOT!
    ;ADC_COTR.7 is cleared by hardware when conversion is done
    ; Load 32-bit 'x' with 12-bit adc result
	mov x+3, #0
	mov x+2, #0
	mov x+1, ADC_H
	mov x+0, ADC_L
    ; Convert ADC reading to temperature in Celsius
    ; Temp (C) = (ADC_value * 5000) / 4096
    Load_y(5000)
	lcall mul32
	Load_y(4096)
	lcall div32
    ; Result is in 'x'
    ;do your displays and stuff
    ret

;--- MAIN PROGRAM START ---
MAIN:
    mov SP, #0x7F         ; Initialize Stack Pointer (Good practice)
    mov P0M1, #0x00
    mov P0M2, #0x00
    mov P1M1, #0x00
    mov P3M2, #0x00
    mov P3M2, #0x00
    mov P3M2, #0x00

    lcall Timer0_Init
    lcall Timer2_Init
    setB EA ; Enable global interrupts
    lcall ELCD_4BIT ; Intialize LCD
    lcall INITIALIZE

    clr half_seconds_flag
    clr START_FLAG

    mov soak_temp_set, #150
    mov soak_time_set, #60
    mov reflow_temp_set, #220
    mov reflow_time_set, #30

MAIN_LOOP:

    RESET_:
        jnb RESET_BUTTON, SOAK_TEMP
        lcall Wait50ms
        jnb RESET_BUTTON, SOAK_TEMP
        
        mov STATE_VAR, $0

        ljmp MAIN

    SOAK_TEMP:
        jnb SOAK_TEMP_BUTTON, SOAK_TIME
        lcall Wait50ms
        jnb SOAK_TEMP_BUTTON, SOAK_TIME
        
        mov a, soak_temp_set

        jb INC_TENS, SOAK_TEMP_TENS
        jb UPDOWN, SOAK_TEMP_DEC
        
        add a, #0x01
        sjmp SOAK_TEMP_DA

        SOAK_TEMP_DEC:
            subb a, #0x01
            sjmp SOAK_TEMP_DA
        
        SOAK_TEMP_TENS:
            jb UPDOWN, SOAK_TEMP_TENS_DEC

            add a, #0x10
            sjmp SOAK_TEMP_DA

        SOAK_TEMP_TENS_DEC:
            subb a, #0x10

        SOAK_TEMP_DA:
            da a
            mov soak_temp_set, a
        
        jb SOAK_TEMP_BUTTON, $
        
        lcall display_params_lcd

    SOAK_TIME:
        jnb SOAK_TIME_BUTTON, REFLOW_TEMP
        lcall Wait50ms
        jnb SOAK_TIME_BUTTON, REFLOW_TEMP

        mov a, soak_time_set

        jb INC_TENS, SOAK_TIME_TENS
        jb UPDOWN, SOAK_TIME_DEC
        
        add a, #0x01
        sjmp SOAK_TIME_DA

        SOAK_TIME_DEC:
            subb a, #0x01
            sjmp SOAK_TIME_DA
        
        SOAK_TIME_TENS:
            jb UPDOWN, SOAK_TIME_TENS_DEC

            add a, #0x10
            sjmp SOAK_TIME_DA

        SOAK_TIME_TENS_DEC:
            subb a, #0x10

        SOAK_TIME_DA:
            da a
            mov soak_time_set, a
    
        lcall display_params_lcd    
        
        
    REFLOW_TEMP:
        jnb REFLOW_TEMP_BUTTON, REFLOW_TIME
        lcall Wait50ms
        jnb REFLOW_TEMP_BUTTON, REFLOW_TIME

        mov a, reflow_temp_set

        jb INC_TENS, REFLOW_TEMP_TENS
        jb UPDOWN, REFLOW_TEMP_DEC
        
        add a, #0x01
        sjmp REFLOW_TEMP_DA

        REFLOW_TEMP_DEC:
            subb a, #0x01
            sjmp REFLOW_TEMP_DA
        
        REFLOW_TEMP_TENS:
            jb UPDOWN, REFLOW_TEMP_TENS_DEC

            add a, #0x10
            sjmp REFLOW_TEMP_DA

        REFLOW_TEMP_TENS_DEC:
            subb a, #0x10

        REFLOW_TEMP_DA:
            da a
            mov reflow_temp_set, a
        
        jb REFLOW_TEMP_BUTTON, $


    REFLOW_TIME:
        jnb REFLOW_TIME_BUTTON, START_STOP_BUTTON
        lcall Wait50ms
        jnb REFLOW_TIME_BUTTON, START_STOP_BUTTON

        mov a, reflow_time_set

        jb INC_TENS, REFLOW_TIME_TENS
        jb UPDOWN, REFLOW_TIME_DEC
        
        add a, #0x01
        sjmp REFLOW_TIME_DA

        REFLOW_TIME_DEC:
            subb a, #0x01
            sjmp REFLOW_TIME_DA
        
        REFLOW_TIME_TENS:
            jb UPDOWN, REFLOW_TIME_TENS_DEC

            add a, #0x10
            sjmp REFLOW_TIME_DA

        REFLOW_TIME_TENS_DEC:
            subb a, #0x10

        REFLOW_TIME_DA:
            da a
            mov reflow_time_set, a
        
        jb REFLOW_TIME_BUTTON, $

        lcall display_params_lcd

    START_STOP_BUTTON:
        jnb REFLOW_TIME_BUTTON, BUTTON_CHECK_DONE
        lcall Wait50ms
        jnb REFLOW_TIME_BUTTON, BUTTON_CHECK_DONE


BUTTON_CHECK_DONE

    jb half_seconds_flag, loop_a
    sjmp MAIN_LOOP;

;==================FSM==================;
;Checklist:
; 1. Implement TEMP and TIME variables
; 2. Implement FSM outputs
; 3. Implement reset logic
; 4. Implement abort condition

loop_a:
    mov a, STATE_VAR
State0:
    cjne a, #0, State1
    jb START_FLAG, State0Done
    sjmp State0
State0Done:
    inc a
    sjmp State0
State1:
    cjne a, #1, State2
    mov R0, #150 ; 150 Degrees
    cjne TEMP, R0, CheckCarryState1 ; NOTE: TEMP is not yet implemented, just a placeholder
    sjmp State1
CheckCarryState1:
    jc LessThanState1
    sjmp GreaterThanState1
LessThanState1:
    sjmp State1
GreaterThanState1:
    inc a
    sjmp State1
State2:
    cjne a, #2, State3
    mov R0, #60 ; 60 seconds
    cjne TIME, R0, CheckCarryState2 ; NOTE: TIME is not yet implemented, just a placeholder
    sjmp State2
CheckCarryState2:
    jc LessThanState2
    sjmp GreaterThanState2
LessThanState2:
    sjmp State2
GreaterThanState2:
    inc a
    sjmp State2
State3:
    cjne a, #3, S
    mov R0, #220; 220 Degrees
    cjne TEMP, R0, CheCheckCarryState3
    sjmpState3    
 
CheckCarryState3:
    jc LessThanState3
    sjmp GreaterThanState3
LessThanState3:
    sjmp State3
GreaterThanState3:
    inc a
    sjmp State3    cjne a, #4, State5
    mov R0, #45 ; 45 Seconds
    cjne TIME, R0, CheckCarryState4
    sjmp State4
CheckCarryState4:
    jc LessThanState4
    sjmp GreaterThanState4
LessThanState4:
    sjmp State4 
GreaterThanState4:
    inc a
    sjmp State4    cjne a, #5, State0
    mov R0, #60 ; 60 Degrees
    cjne TEMP, R0, CheckCarryState5
    sjmp State5
CheckCarryState5:
    jc LessThanState5
    sjmp GreaterThanState5
LessThanState5:
    mov a, #0
    sjmp State5
GreaterThanState5:
    sjmp State5

END