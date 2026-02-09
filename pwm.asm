

LEAD        EQU 32h    ; lead margin for ramp (e.g., 15)
ON_SECS     EQU 33h    ; how many seconds ON per window 


PHASE       EQU 35h    ;counter for seconds

WINDOW      EQU 10     ; burst window length in seconds 




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
        MOV     THRESH, A         

        ;if temp is less than threshold, full power
        MOV     A, TEMP           
        CLR     C
        SUBB    A, THRESH         
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



;DO THIS BEFORE ENTERRING THE RAMP STATES
MOV PHASE, #00h






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

        ljmp 	ramp_set_off     


ramp_done:
        RET                      

ramp_force_on:
        SETB p0.0      ;power on
        RET

ramp_set_off:
        CLR p0.0
        RET 
        

pwm_for_ramp2:
		MOV     A, TARGET          
        CLR     C                 
        SUBB    A, #LEAD2         
                                 
        JNC     ramp_thresh_ok2    
        MOV     A, #00h           
ramp_thresh_ok2:
        MOV     THRESHOLD, A         

        ;if less than threshold turn power on 
        MOV     A, TEMP          
        CLR     C                 
        SUBB    A, THRESHOLD         ; A = curren temp - threshold(target-lead)
                                 
        JC      ramp_force_on2     ; If below threshold force on and return

        ljmp 	ramp_set_off2     


ramp_done2:
        RET                      

ramp_force_on2:
        SETB p0.0      ;power on
        RET

ramp_set_off2:
        CLR p0.0
        RET 