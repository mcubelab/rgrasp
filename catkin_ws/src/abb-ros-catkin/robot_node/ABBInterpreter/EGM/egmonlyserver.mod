MODULE EGM_test_UDP
    
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_lin1:=[-1,1]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-2,2];! in degees

    PERS tooldata currentTool2:=[TRUE,[[0,0,0],[1,0,0,0]],[0.4,[0,0,20],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata currentWobj2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[600,0,0],[0.707,0,-0.707,0]]];
    
    VAR pose posecorEGM:=[[0,0,0],[1,0,0,0]];
    VAR pose posesenEGM:=[[0,0,0],[1,0,0,0]];
    CONST jointtarget jointsTargetEGM1:=[[-8.19, 3.2, 40.82, -11.69, -44.63, 8.38],  [ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
    CONST jointtarget jointsTargetEGM2:=[[-5.75,17.06,40.71,-6.79,-57.95,3.61],  [ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
    VAR robtarget cartesianTarget:=[[230,-40,230],
                                  [0,0,1,0],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
 
    PROC main()
        !ConfJ\Off;
        !SingArea\Off;
        !MoveAbsJ jointsTargetEGM1, v50, fine, currentTool2 \Wobj:=currentWobj2;
        ConfL\Off;
        MoveL cartesianTarget, v20, fine, currentTool2 \Wobj:=currentWobj2;
        !ConfL\On;
        !MoveAbsJ jointsTargetEGM2, v50, fine, currentTool2 \Wobj:=currentWobj2;
        WaitTime 0.5;
        
        !EXIT;
		EGMReset egmID1;
		EGMGetId egmID1;
		egmSt1 := EGMGetState(egmID1);
		TPWrite "EGM state: "\Num := egmSt1;
		
		IF egmSt1 <= EGM_STATE_CONNECTED THEN
			! Set up the EGM data source: UdpUc server using device "EGMsensor:"and configuration "default"
			EGMSetupUC ROB_2, egmID1, "default", "EGMsensor" \pose;
		ENDIF
		
		!runEGM
		
		EGMActPose egmID1\Tool:=currentTool2 \WObj:=currentWobj2, posecorEGM,EGM_FRAME_WOBJ, posesenEGM, EGM_FRAME_WOBJ 
		\x:=egm_minmax_lin1 \y:=egm_minmax_lin1 \z:=egm_minmax_lin1
		\rx:=egm_minmax_rot1 \ry:=egm_minmax_rot1 \rz:=egm_minmax_rot1\LpFilter:=100\Samplerate:=4\MaxSpeedDeviation:= 40;
				
		EGMRunPose egmID1, EGM_STOP_HOLD \x \y \z\CondTime:=20 \RampInTime:=0.05\RampOutTime:=0.5;
		egmSt1:=EGMGetState(egmID1); 

		IF egmSt1 = EGM_STATE_CONNECTED THEN
			TPWrite "Reset EGM instance egmID1";
			EGMReset egmID1; 
		ENDIF 
        
		TPWrite "EGM STOPPED";
        WHILE TRUE DO
            WaitTime 0.5;
			TPWrite "Reset EGM instance egmID1";
            
        ENDWHILE
    ENDPROC		
 
ENDMODULE
