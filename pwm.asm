pwm_for_flatstates:
; ---- LOW_LIM = max(0, T_TGT - T_BAND)
        MOV     A, TARGET          
        CLR     C                 ;clear carry
        SUBB    A, BAND         ; A = A - BAND
        JNC     flat_low_ok       
        MOV     A, #00h           
flat_low_ok:
        MOV     LOW_LIMIT, A        ; Store low limit in RAM

        ;compute high limit
        MOV     A, TARGET          
        ADD     A, BAND         
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
        SUBB    A, LEAD         
                                 
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
        SUBB    A, BAND          
        JNC     ramp_low_ok       
        MOV     A, #00h           
ramp_low_ok: 
        MOV     LOW_LIMIT, A        
;compute high limit
        MOV     A, TARGET          
        ADD     A, BAND         
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