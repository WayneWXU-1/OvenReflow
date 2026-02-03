$MODMAX10 ;Defines Target Device DE10-Lite

OVEN_PIN    BIT P0_0      ;Pin connected to the oven control (e.g., SSR)
;--------------------------------------------------------
;DEFINE VARIABLES IN DATA RAM
;--------------------------------------------------------
dseg at 0x30
STATE_VAR:  DS 1       ;State Variable
SECOND_COUNTER: DS 1   ;Second Counter
TEMP_HIGH_BYTE: DS 1   ;High Byte of Temperature Reading
TEMP_LOW_BYTE: DS 1    ;Low Byte of Temperature Reading

;--------------------------------------------------------
bseg ;special RAM
START_FLAG:  DS 1       ;Flag to indicate start of reflow process
;--------------------------------------------------------


cseg at 0x00
ljmp MAIN    ;Jump to start of program





INITIALIZE:
    ; Initialize ports
    mov ADC_C, #0x00  ; Use port 0 as ADC input




;--MAIN PROGRAM START--;
MAIN:
    lcall INITIALIZE

    ; Main program loop
MAIN_LOOP:
    
    setb 



    ljmp MAIN_LOOP  ; Infinite loop
    ;At some point to move the ADC values do 
    ;mov A, ADC_L
    ;do something...
    ;mov A, ADC_H
    ;do something...