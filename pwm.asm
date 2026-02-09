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