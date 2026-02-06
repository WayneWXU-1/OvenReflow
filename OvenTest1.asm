$NOLIST
$MODMAX10
$LIST

CLK           EQU 33333333 ; Microcontroller system crystal frequency in Hz
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/(12*TIMER0_RATE)))) ; The prescaler in the CV-8052 is always 12 unlike the N76E003 where is selectable.
TIMER2_RATE   EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/(12*TIMER2_RATE))))
BAND          EQU 3 ;for flat states
LEAD          EQU 10 ;for ramp sates

BAUD   EQU 57600
T1_LOAD EQU 256-(2*CLK) / (32*12*BAUD) ;Load 253 so it counts 3 counts before overflowing, which gives us a 57600 baud rate with a 33.333MHz clock


; ********* Buttons ***********
SELECT_BUTTON equ KEY_1
RESET_BUTTON  equ KEY_0
START_BUTTON  equ KEY_3
STOP_BUTTON   equ KEY_2
;EXTRA_BUTTON  equ P4_6

OVEN_PIN      equ P0.0
SOUND_OUT     equ P1.5 ; Speaker attached to this pin
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
DEGREES60:       DS 2
DEGREES150:      DS 2
DEGREES220:      DS 2
TARGET:           DS 2
;*** Variables ***
SOAK_TEMP_set:       ds 2
SOAK_TIME_set:       ds 2
reflow_temp_set:     ds 2
REFLOW_TIME_set:     ds 2

soak_time:       ds 2
REFLOW_TIME:     ds 2

beep_count:      ds 1

; PWM variables
LOW_LIMIT:  ds 2
HIGH_LIMIT: ds 2
THRESHOLD:  ds 2

x:		ds	4 ;used for 32 bit math for temperature conversion
y:		ds	4 ;used for 32 bit math for temperature conversion
bcd:    ds  5 ; <--- ADD THIS: math32 needs 5 bytes for BCD conversions

;--ISR RELATED--;
Count1ms:     ds 2 ; Used to determine when half second has passed
BCD_counter:  ds 1 ; The BCD counter incrememted in the ISR and displayed in the main loop


bseg
START_FLAG:         DBIT 1  ; Use DBIT for single bits in bseg
half_seconds_flag:  DBIT 1  ; half second flag
SECONDS_FLAG:       DBIT 1  ; can change later depending on how fast we want it
SELECT_BUTTON_FLAG: DBIT 1
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
ROw4 EQU P2.0
COL1 EQU P2.2
COL2 EQU P2.4
COL3 EQU P2.6
COL4 EQU P3.0

;                           1234567890123456
soak_temp_message:      db 'Soak Temp: xxx C', 0
soak_time_message:      db 'Soak Time: xxx C', 0
reflow_temp_message:    db 'Reflow Temp: xxs', 0
reflow_time_message:    db 'Reflow Time: xxs', 0
ready_message:          db 'Ready to Start! ', 0

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


; Transfer readings data to the accumulator and serial output
SendSerial:
    mov x+0, TEMP+0 ; reloads the temp into x which will be converted to bcd
    mov x+1, TEMP+1
    mov x+2, #0
    mov x+3, #0
    lcall hex2bcd ; standard math32.asm function

    mov a, bcd+1 ; stores hundreds position in accumulator
    anl a, #0x0F ; and operator to zero out first byte which is thousands( Has no value for our readings to ~250)
    add a, #0x30 ; 3 is the ascii operator code for serial print
    lcall putchar

    mov a, bcd+0 ; first two bytes that are in bcd
    swap a  ; because bcd is packed in two bytes, we must put the upper one in the lower to print
    anl a, #0x0F
    add a, #0x30
    lcall putchar

    mov a, bcd+0 ; no swap needed given the ones place is in the first byte
    anl a, #0x0F
    add a, #0x30
    lcall putchar

    mov a, #'\r'    ; Call both return and newline to avoid terminal view errors
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
	cjne a, #low(1000), Timer2_ISR_Midpoint ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_Midpoint
    
    ;a second has passed good to convert temperature;
    ;---------------Temperature reading and conversion function------------------;
    ; Start ADC conversion

    mov a, ADC_C 
    orl a, #0x80 ; Set start conversion bit

Temp_Conversion_Wait:

    cjne a, #0x00, Temp_Conversion_Wait ; Just a dummy check to waste some time
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
    mov TEMP+0, x+0
    mov TEMP+1, x+1

    lcall Display_BCD_7_seg
    lcall SendSerial

    sjmp Timer2_ISR_Bypass

Timer2_ISR_Midpoint:
ljmp Timer2_ISR_done
Timer2_ISR_Bypass:

;---------------------------------------------------------------------;
	
	;1 second have passed.  Set a flag so the main program knows
	setb seconds_flag ; Let the main program know one second had passed
	; Toggle LEDR0 so it blinks
    inc TIME ; Increment the TIME Variable
	cpl LEDRA.0
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
    
    ;Call PWM funcions
Checkforstate1:
    MOV A, STATE_VAR_1
    CJNE A, #1, NOT_STATE_1
    LCALL pwm_for_ramp
    SJMP PWM_EXIT
NOT_STATE_1:
    CJNE A, #3, NOT_STATE_3
    LCALL pwm_for_ramp
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

	mov P0MOD, #10101011b ; P0.0(OVEN_PIN), P0.1, P0.3, P0.5, P0.7(LCD) are outputs. 
    mov P1MOD, #11110110b ; P1.7, P1.5, P1.1(LCD), 1.2, 1.4, 1.6(ROW) are outputs
    mov P2MOD, #00000001b ; 2.0(ROW), 2.2, 2.4, 2.6(COL)
    mov P3MOD, #00000001b ; 3.0 (COL)
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


; **************************** KEYPAD *******************************




; Look-up table for the 7-seg displays. (Segments are turn on with zero) 
T_7seg:
    DB 0xC0, 0xF9, 0xA4, 0xB0, 0x99        ; 0 TO 4
    DB 0x92, 0x82, 0xF8, 0x80, 0x90        ; 4 TO 9
    DB 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E  ; A to F

; Displays a BCD number in HEX1-HEX0
Display_BCD_7_Seg:

    push acc
    push psw
	
	mov dptr, #T_7seg

    mov a, TEMP+1
    anl a, #0FH
    movc a, @a+dptr
    mov HEX2, a

	mov a, TEMP
	swap a
	anl a, #0FH
	movc a, @a+dptr
	mov HEX1, a
	
	mov a, TEMP
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

    Not_Pressed:

    ret


    
    ;---------------READ KEYPAD-------------------;
    ;A Macro essentially works like CHECK_COL(COL#, Literal value coloumn represents)
    ;If the column is pressed, R7 will contain the column number (1-4)
    

;**************************PWM**************************;
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




pwm_for_ramp:
MOV     A, TARGET          
        CLR     C                 
        SUBB    A, #LEAD         
                                 
        JNC     ramp_thresh_ok    
        MOV     A, #00h           
ramp_thresh_ok:
        MOV     THRESHOLD, A         

        ;if less than threshold turn power on 
        MOV     A, TEMP          
        CLR     C                 
        SUBB    A, THRESHOLD         ; A = curren temp - threshold(target-lead)
                                 
        JC      ramp_force_on     ; If below threshold force on and return

        ;when close to target use deadband
        ; LOW_LIM = max(0, T_TGT - BAND)
        MOV     A, TARGET          
        CLR     C                 
        SUBB    A, #BAND          
        JNC     ramp_low_ok       
        MOV     A, #00h           
ramp_low_ok: 
        MOV     LOW_LIMIT, A        
;compute high limit
        MOV     A, TARGET          
        ADD     A, #BAND         
        JNC     ramp_high_ok      
        MOV     A, #0FFh          
ramp_high_ok:
        MOV     HIGH_LIMIT, A       

        ;if current temp is less than low limit turn power on 
        MOV     A, TEMP          
        CLR     C                 
        SUBB    A, LOW_LIMIT        ;A = current temp - low limit (borrow if CUR < low limit)
        JC      ramp_set_on       ; If below low limit, turn ON

        ;else turn off
        MOV     A, TEMP          
        CLR     C                 
        SUBB    A, HIGH_LIMIT       
        JZ      ramp_done         ; If equal to HIGH_LIM, inside band do nothing
        JNC     ramp_set_off      ; If no borrow and not zero, above high limit set OFF

ramp_done:
        RET                      

ramp_force_on:
        SETB p0.0      ;power on
        RET

ramp_set_on:
        SETB p0.0
        RET

ramp_set_off:
        CLR p0.0
        RET 

;--- MAIN PROGRAM START ---
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

    mov soak_temp_set, #150
    mov soak_time_set, #60
    mov reflow_temp_set, #220
    mov reflow_time_set, #30

    mov STATE_VAR_1, #0x0000
    mov STATE_VAR_2, #0x0000
    mov TIME, #0
    mov TEMP, #0
    mov POWER, #0
    mov DEGREES60, #60
    mov DEGREES150, #150
    mov DEGREES220, #220
    mov TARGET,       #0

    mov bcd, #0x0000

MAIN_LOOP:

PARAM_FSM:


; **************************** FSM for selecting parameters *************************
; 4 main states ->  A: select soak temp
;                   B: select soak time
;                   C: select reflow temp
;                   D: select reflow time
;
; move to other FSM when start button turns on start flag

    mov a, STATE_VAR_2

StateAInit:
    Set_Cursor(1,1)
    Send_Constant_String (#soak_temp_message)
StateA:
    mov a, STATE_VAR_2

    cjne a, #0, StateBInit
    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateADone
    
    ;lcall Keypad
    ;jnc StateA

    ;lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    sjmp StateA
StateADone:
    mov soak_temp_set+0, bcd+0
    mov soak_temp_set+1, bcd+1
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    sjmp StateA

StateBInit:
Set_Cursor(1,1)
    Send_Constant_String(#soak_time_message)
    clr SELECT_BUTTON_FLAG
StateB:
    mov a, STATE_VAR_2

    cjne a, #1, StateCInit
    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateBDone
    
    ;lcall Keypad
    ;jnc StateB

    ;lcall Shift_Digits_Left
    
    Set_Cursor(1,12)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    
    sjmp StateB
StateBDone:
    mov soak_time_set+0, bcd+0
    mov soak_time_set+1, bcd+1
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    sjmp StateB

StateCInit:
    Set_Cursor(1,1)
    send_constant_string(#reflow_temp_message)
StateC:
    mov a, STATE_VAR_2

    cjne a, #2, StateDInit
    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateCDone

    ;lcall Keypad
    ;jnc StateC

    ;lcall Shift_Digits_Left
    
    Set_Cursor(1,14)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)

    sjmp StateC
StateCDone:
    mov reflow_temp_set+0, bcd+0
    mov reflow_temp_set+1, bcd+1
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    sjmp StateC

StateDInit:
    Set_Cursor(1,1)
    send_constant_string(#reflow_time_message)
StateD:
    mov a, STATE_VAR_2

    cjne a, #3, ReadyStateInit
    lcall Check_Select_Button_Press
    jb SELECT_BUTTON_FLAG, StateDDone

    ;lcall Keypad 
    ;jnc StateD

    ;lcall Shift_Digits_Left
    
    Set_Cursor(1,14)
    Display_BCD(bcd+1)
    Display_BCD(bcd+0)
    Display_BCD(reflow_time_set)

    sjmp StateD

StateDDone:
    mov reflow_time_set+0, bcd+0
    mov reflow_time_set+1, bcd+1
    inc STATE_VAR_2
    clr SELECT_BUTTON_FLAG
    sjmp StateD

ReadyStateInit:
    Set_Cursor(1,1)
    Send_Constant_String(#ready_message)
    
ReadyState:
    ;jnb seconds_flag, skipSerial_0 *** not too sure what this does

skipSerial_0:
    jb START_BUTTON, ReadyState
    lcall wait50ms
    jb START_BUTTON, ReadyState

    setb START_FLAG
    sjmp State0


;==================Reflow Profile FSM==================;
;Checklist:
; 1. Implement TEMP and TIME variables - DONE
; 2. Implement FSM outputs - DONE
; 3. Implement reset logic - DONE
; 4. Implement abort condition - DONE
; 5. Implement LCD Feedback for Each State - Not Complete
; 6. Speaker beeps for state transitions - Not Complete
State0:
    jb STOP_BUTTON, State0_StopReflow
    CLR p0.0 ;oven off
    mov a, STATE_VAR_1
    cjne a, #0, State1
    jb START_FLAG, State0Done
    sjmp State0

State0_StopReflow:
ljmp StopReflow

State0Done:
    inc STATE_VAR_1
    mov POWER, #100
    mov TIME, #0
    sjmp State0
State1:
    jb RESET_BUTTON, State1_ResetToMain
    jb STOP_BUTTON, State1_StopReflow
    mov a, STATE_VAR_1
    cjne a, #1, State2
    mov TARGET, DEGREES150
    mov R2, TEMP
    mov a, TARGET
    cjne a, #0x02, CheckCarryState1
    sjmp State1

State1_ResetToMain:
ljmp ResetToMain

State1_StopReflow:
ljmp StopReflow

CheckCarryState1:
    jc GreaterThanState1
    sjmp LessThanState1
LessThanState1:
    sjmp State1
GreaterThanState1:
    inc STATE_VAR_1
    mov POWER, #20
    sjmp State1
State2:
    jb RESET_BUTTON, State2_ResetToMain
    jb STOP_BUTTON, State2_StopReflow
    mov a, STATE_VAR_1
    cjne a, #2, State3
    mov R0, #60 ; 60 seconds
    mov a, TIME
    cjne a, #0x00, CheckCarryState2 
    sjmp CheckAbortCondition ; Check if Temp. is at least 50 degrees after 60 seconds have passed

State2_ResetToMain:
ljmp ResetToMain

State2_StopReflow:
ljmp StopReflow

State2_StopOven:
ljmp STOPOVEN

CheckAbortCondition:
    mov R1, #50
    mov a, TEMP
    cjne a, #0x01, CheckAbortCarry
    ; TEMP == 50, good enough to proceed
    inc STATE_VAR_1
    mov POWER, #100       ; Set State3 power
    mov TIME, #0          ; Reset timer for State3
    sjmp State3
CheckAbortCarry:
    jc State2_StopOven          
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
    jb RESET_BUTTON, State3_ResetToMain
    jb STOP_BUTTON, State3_StopReflow
    mov a, STATE_VAR_1
    cjne a, #3, State4
    mov TARGET, DEGREES220
    mov R2, TEMP
    mov a, TARGET
    cjne a, #0x02, CheckCarryState3
    sjmp State3    

State3_ResetToMain:
ljmp ResetToMain

State3_StopReflow:
ljmp StopReflow

CheckCarryState3:
    jc GreaterThanState3
    sjmp LessThanState3
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
    mov a, TIME
    cjne a, #0x00, CheckCarryState4
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
    cjne a, #5, State5toState0
    mov TARGET, DEGREES60
    mov R2, TEMP
    mov a, TARGET
    cjne a, #0x02, CheckCarryState5
    sjmp State5
    
State5toState0:
    ljmp State0

CheckCarryState5:
    jc GreaterThanState5
    sjmp LessThanState5
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