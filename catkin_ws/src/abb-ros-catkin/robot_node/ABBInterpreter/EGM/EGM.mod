MODULE EGM_test_UDP
    
    VAR egmident egmID1;
    VAR egmstate egmSt1;
    
    CONST egm_minmax egm_minmax_lin1:=[-1,1]; !in mm
    CONST egm_minmax egm_minmax_rot1:=[-2,2];! in degees

    PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[600,0,0],[0.707,0,-0.707,0]]];
    
    VAR pose posecorTable:=[[0,0,0],[1,0,0,0]];
    VAR pose posesenTable:=[[0,0,0],[1,0,0,0]];
  
 
    PROC main()
        ! Move to start position. Fine point is demanded.
        !MoveJ p20,v100,fine,UISpenholder;
        !CONST jointtarget jointsTarget:=[[-17.35, -35.15, 69.13, -29.21, -37.68, 0],  [ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9]]; 
        CONST jointtarget jointsTarget:=[[0, 0, 0, 0, 90, 0],  [ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9]]; 
        ! MoveAbsJ jointsTarget, v50, fine, currentTool \Wobj:=currentWobj;
        testuc_UDP; 
    ENDPROC
  
    
    PROC testuc_UDP()
        EGMReset egmID1;
        EGMGetId egmID1;
        egmSt1 := EGMGetState(egmID1);
        TPWrite "EGM state: "\Num := egmSt1;
        
        IF egmSt1 <= EGM_STATE_CONNECTED THEN
            ! Set up the EGM data source: UdpUc server using device "EGMsensor:"and configuration "default"
            EGMSetupUC ROB_2, egmID1, "default", "EGMsensor" \pose;
        ENDIF
        
        !Which program to run
        runEGM;

        IF egmSt1 = EGM_STATE_CONNECTED THEN
            TPWrite "Reset EGM instance egmID1";
            EGMReset egmID1; 
        ENDIF    
    ENDPROC
        
        
    PROC runEGM()
        EGMActPose egmID1\Tool:=currentTool \WObj:=wobj0, posecorTable,EGM_FRAME_WOBJ, posesenTable, EGM_FRAME_WOBJ 
        \x:=egm_minmax_lin1 \y:=egm_minmax_lin1 \z:=egm_minmax_lin1
        \rx:=egm_minmax_rot1 \ry:=egm_minmax_rot1 \rz:=egm_minmax_rot1\LpFilter:=2\Samplerate:=4\MaxSpeedDeviation:= 40;
                
        EGMRunPose egmID1, EGM_STOP_RAMP_DOWN\x \y \z\CondTime:=20 \RampInTime:=0.05\RampOutTime:=0.5;
        egmSt1:=EGMGetState(egmID1); 
    ENDPROC
 
ENDMODULE
