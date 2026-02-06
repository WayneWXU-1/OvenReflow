$NOLIST
$MODMAX10
$LIST

CLK           EQU 33333333 ; Microcontroller system crystal frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.
TIMER2_RATE   EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/(12*TIMER2_RATE))))

; ********* Buttons ***********
SELECT_BUTTON equ Px.x
RESET_BUTTON  equ PX.X
START_BUTTON  equ PX.X
STOP_BUTTON   equ PX.X

OVEN_PIN      equ P0.0
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
STATE_VAR_1:     DS 1 
STATE_VAR_2:     DS 1
SECOND_COUNTER:  DS 1 
TEMP_HIGH_BYTE:  DS 1 
TEMP_LOW_BYTE:   DS 1 

TEMP:            DS 2
TIME:            DS 2
POWER:           DS 2
;*** Variables ***
SOAK_TEMP_set       ds 2
SOAK_TIME_set       ds 2
reflow_temp_set     ds 2
REFLOW_TIME_set     ds 2

soak_time       ds 2
REFLOW_TIME     ds 2

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
ELCD_RS equ P1.7
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
ELCD_E equ  P1.1
ELCD_D4 equ P0.7
ELCD_D5 equ P0.5
ELCD_D6 equ P0.3
ELCD_D7 equ P0.1

;Keypad pin assignments
ROW1 EQU PX.X
ROW2 EQU PX.X
ROW3 EQU PX.x
ROW4 EQU PX.x
COL1 EQU PX.x
COL2 EQU PX.x
COL3 EQU PX.x
COL4 EQU PX.x

$NOLIST
$include(LCD_4bit_DE10Lite_no_RW.inc) ; A library of LCD related functions and utility macros
$LIST

$NOLIST
$include(math32.asm)
$include(Read_keypad.asm)
$LIST

;-----------------------INTIALIZE SERIAL PORT FOR INPUT OUTUUT-----------------------;
;--Setting baud rate to 115200 with 33.33MHz clock--;
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
    anl a, 0x0f ; Clear the bits for timer 1
    orl a, 0x20 ; Configure timer 1 as 8-bit autoreload
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
	cpl P1.1 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
	; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1 ;increment high 8-bits if low 8-bits overflowed

Inc_Done:
	; Check if full second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), Timer2_ISR_done ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_done
    ;a second has passed good to convert temperature;
    ;---------------Temperature reading and conversion function------------------;
    ; Start ADC conversion
    setb ADC_CONTR.7 ; Set ADC start bit SNAPSHOT!
    jb ADC_CONTR,7, $ ;hold until done
    ;ADC_COTR.7 is cleared by hardware when conversion is done
    ; Load 32-bit 'x' with 12-bit adc result
	mov x+3, #0
	mov x+2, #0
	mov x+1, ADC_H
	mov x+0, ADC_L
    ; Convert ADC reading to temperature in Celsius
    ; Voltage = (ADC_value * 5000) / 4096
    Load_y(5000)
	lcall mul32
	Load_y(4096)
	lcall div32
    ; Result is in 'x'

    Load_y(1000) ; convert to microvolts
    lcall mul32
    Load_y(12300) ; 41 * 300
    lcall div32

    Load_y(22) ; add cold junction temperature
    lcall add32
    ;do your displays and stuff
    ;result is still in x
    mov TEMP, x
    ret
;---------------------------------------------------------------------;
	
	;1 second have passed.  Set a flag so the main program knows
	setb seconds_flag ; Let the main program know half second had passed
	; Toggle LEDR0 so it blinks
    inc TIME ; Increment the TIME Variable
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

	mov P0MOD, #10101011b ; P0.0(OVEN_PIN), P0.1, P0.3, P0.5, P0.7 are outputs. 
    mov P1MOD, #10100010b ; P1.7, P1.5, P1.1 are outputs
    mov P2MOD, #0xff
    mov P3MOD, #0xff
    ; for keypad, (ROWS as output-1)1.2, 1.4, 1.6, 2.0 - (COLS as input-0) 2.2, 2.4, 2.6, 3.0
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


;                           1234567890123456
soak_temp_message:      db 'Soak Temp: xxx C', 0
soak_time_message:      db 'Soak Time: xxx C', 0
reflow_temp_message:    db 'Reflow Temp: xxs', 0
reflow_time_message:    db 'Reflow Time: xxs', 0
ready_message:          db 'Ready to Start! ', 0

display_soak_params_lcd:
    
    Set_Cursor(1,1)
    Send_Constant_String(#soak_param_message)
    
    Set_Cursor(2,1)
    Send_Constant_String(#reflow_param_message)

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


    
    ;---------------READ KEYPAD-------------------;
    ;A Macro essentially works like CHECK_COL(COL#, Literal value coloumn represents)
    ;If the column is pressed, R7 will contain the column number (1-4)
    CHECK_COLUMN MAC
        jb 0%, NOT_PRESSED ;if the col value is 1 then not pressed 
        mov R7, %1 ;move the literal value into R7
        jnb 0%, $ ;wait until released
        setb c ;flag to indicate key pressed
        ret
    NOTPRESSED:
    ENDMAC

    ;P1MOD Controls the P1.x pin modes
    ;P2MOD Controls the P2.x pin modes
    
    Configure_Pins:
        ;Honestly figure out the input and output pins later 
        SETB P0MOD.0 ;pin to drive oven o

 
    ;USING DUMMY VARIABLES FOR ROW/COL PINS, REPLACE LATER
    CHECK_KEYPAD:
        ;check row1
        clr ROW1 ;set specific pin to 0 to check columns ROW1 = GND
        CHECK_COLUMN(COL1, 0x01)
        CHECK_COLUMN(COL2, 0x02)
        CHECK_COLUMN(COL3, 0x03)
        CHECK_COLUMN(COL4, 0x04)
        setb ROW1

        ;check row2
        clr ROW
    

;--- MAIN PROGRAM START ---
MAIN:
    mov SP, #0x7F         ; Initialize Stack Pointer (Good practice)
    lcall INITIALIZE      ; intialize pins and adc, for now

    lcall Timer0_Init
    lcall Timer2_Init
    setB EA ; Enable global interrupts
    lcall ELCD_4BIT ; Intialize LCD
    

    clr seconds_flag
    clr START_FLAG

    mov soak_temp_set, #150
    mov soak_time_set, #60
    mov reflow_temp_set, #220
    mov reflow_time_set, #30

    mov STATE_VAR_1, #0x0000
    mov STATE_VAR_2, #0x0000
    mov TIME, #0
    mov TEMP, #0
    mov POWER, #0

MAIN_LOOP:

    RESET_:
        jb RESET_BUTTON, SOAK_TEMP
        lcall Wait50ms
        jb RESET_BUTTON, SOAK_TEMP
        
        mov STATE_VAR, $0

        ljmp MAIN

        

PARAM_FSM:


; **************************** FSM for selecting parameters *************************
; 4 main states ->  A: select soak temp
;                   B: select soak time
;                   C: select reflow temp
;                   D: select reflow time
;
; move to other fsm when start button turns on start flag

    mov a, STATE_VAR_2

StateAInit:
    Send_Constant_String (#soak_temp_message)
StateA:
    cjne a, #0, StateBInit
    jb SELECT_BUTTON_FLAG, StateADone
    
    lcall Keypad
    jnc StateA

    lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    sjmp StateA
StateADone:
    mov soak_temp_set+0, bcd+0
    mov soak_temp_set+1, bcd+1
    inc a
    clr SELECT_BUTTON_FLAG
    sjmp StateA

StateBInit:
    Send_Constant_String(#soak_time_message)
StateB:
    cjne a, #1, StateCInit
    jb SELECT_BUTTON_FLAG, StateBDone
    
    lcall Keypad
    jnc StateB

    lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    
    sjmp StateB
StateBDone:
    mov soak_time_set+0, bcd+0
    mov soak_time_set+1, bcd+1
    inc a
    clr SELECT_BUTTON_FLAG
    sjmp StateB

StateCInit:
    send_constant_string(#reflow_temp_message)
StateC:
    cjne a, #2, StateDInit
    jb SELECT_BUTTON_FLAG, StateCDone

    lcall Keypad
    jnc StateC

    lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    sjmp StateC
StateCDone:
    mov reflow_temp_set+0, bcd+0
    mov reflow_temp_set+1, bcd+1
    inc a
    clr SELECT_BUTTON_FLAG
    sjmp StateC

StateDInit
    send_constant_string(#reflow_time_message)
StateD:
    cjne a, #3, State0
    jb SELECT_BUTTON_FLAG, StateDDone

    lcall Keypad 
    jnc StateD

    lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    Display_BCD(reflow_time_set)

    sjmp St
    mov reflow_time_set+0, bcd+0
    mov reflow_time_set+1, bcd+1ateD
StateDDone:
    inc a
    clr SELECT_BUTTON_FLAG
    sjmp StateD



;==================Reflow Profile FSM==================;
;Checklist:
; 1. Implement TEMP and TIME variables - TIME is done, still waiting on TEMP
; 2. Implement FSM outputs - Added a POWER variable for completeness, not yet implemented
; 3. Implement reset logic - DONE
; 4. Implement abort condition - DONE
; 5. Implement LCD Feedback for Each State
State0:
    jb STOP_BUTTON, StopReflow
    CLR p0.0 ;oven off
    mov a, STATE_VAR_1
    cjne a, #0, State1
    jb START_FLAG, State0Done
    sjmp State0
State0Done:
    inc STATE_VAR_1
    mov POWER, #100
    mov TIME, #0
    sjmp State0
State1:
    jb RESET_BUTTON, ResetToState0
    jb STOP_BUTTON, StopReflow
    mov a, STATE_VAR_1
    SETB p0.0 ;power on 100 percent
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
    inc STATE_VAR_1
    mov POWER, #20
    sjmp State1
State2:
    jb RESET_BUTTON, ResetToState0
    jb STOP_BUTTON, StopReflow
    mov a, STATE_VAR_1
    cjne a, #2, State3
    mov R0,M#60 ; 60 seconds
    mov R0, #60 ; 60 seconds
    cjne TIME, R0, CheckCarryState2 ; NOTE: TIME is not yet implemented, just a placeholder
    sjmp CheckAbortCondition ; Check if Temp. is at least 50 degrees after 60 seconds have passed
CheckAbortCondition:
    mov R1, #50
    cjne TEMP, R1, CheckAbortCarry
    ; TEMP == 50, good enough to proceed
    inc STATE_VAR_1
    mov POWER, #100       ; Set State3 power
    mov TIME, #0          ; Reset timer for State3
    sjmp State3
CheckAbortCarry:
    jc STOPOVEN          
    ; TEMP > 50, definitely good to proceed
    inc STATE_VAR_1
    mov POWER, #100
    mov TIME, #0
    sjmp State3
CheckCarryState2:
    jc LessThanState2
    sjmp GreaterThanState2
LessThanState2:
    sjmp State2
GreaterThanState2:
    inc STATE_VAR_1
    mov POWER, #100
    mov TIME, #0
    sjmp State2
State3:
    jb RESET_BUTTON, ResetToMain
    jb STOP_BUTTON, StopReflow
    SETB p0.0 ;power on 100 percent
    mov a, STATE_VAR_1
    cjne a, #3, State4
    mov R0, #220; 220 Degrees
    cjne TEMP, R0, CheckCarryState3
    sjmp State3    
CheckCarryState3:
    jc LessThanState3
    sjmp GreaterThanState3
LessThanState3:
    sjmp State3
GreaterThanState3:
    inc STATE_VAR_1
    mov POWER, #20
    sjmp State3
State4:
    jb RESET_BUTTON, ResetToMain
    jb STOP_BUTTON, StopReflow
    mov a, STATE_VAR_1
    cjne a, #4, State5
    mov R0, #45 ; 45 Seconds
    cjne TIME, R0, CheckCarryState4
    sjmp State4
CheckCarryState4:
    jc LessThanState4
    sjmp GreaterThanState4
LessThanState4:
    sjmp State4 
GreaterThanState4:
    inc STATE_VAR_1
    mov POWER, #0
    sjmp State4
State5:
    jb RESET_BUTTON, ResetToMain
    jb STOP_BUTTON, StopReflow
    CLR p0.0 ;turn oven off
    mov a, STATE_VAR_1    
    cjne a, #5, State0
    mov R0, #60 ; 60 Degrees
    cjne TEMP, R0, CheckCarryState5
    sjmp State5
CheckCarryState5:
    jc LessThanState5
    sjmp GreaterThanState5
LessThanState5:
    mov STATE_VAR_1, #0
    clr START_FLAG
    sjmp State5
GreaterThanState5:
    sjmp State5

ResetToMain:
    mov STATE_VAR_1, #0
    mov POWER, #0
    ljmp MAIN

StopReflow:
    ljmp MAIN

STOPOVEN:
    jb RESET_BUTTON, RestartProcess
    sjmp STOPOVEN ; Infinite loop to stop the oven if abort condition is met

RestartProcess:
    ljmp MAIN
END