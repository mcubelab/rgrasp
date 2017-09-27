MODULE INIT
!Task to check when the motion task is stopped after a motion suppervision error.
!In that case the task raises a DO_RESET_ERROR and restarts the tasks with DO_START
!This is a SEMISTATIC Task, so it will start automatically.

!Main procedure
!It calls to the routine that cheks for motion suppervision error
!at at 2 Hz
PROC main()
    MotionSupReset;
    WaitTime 0.5;    
    !Note that this will only loop when the task is configured to be semistatic.
ENDPROC


!Checking routine 
PROC MotionSupReset()
    IF OpMode()=OP_AUTO AND DOutput(SO_MOTIONSUP_ON)=1 AND DOutput(SO_TROB1_EXEC)=0 THEN
        !If the robot is in automode, and Motion suppervision was just
        !triggered, and ROB_1 is not executing then restart execution.
        WaitTime 3;    
        Set DO_RESET_ERROR; !Raise reset error
        WaitTime 3;    
        Set DO_START; !Restart tasks
    ELSEIF (DOutput(DO_RESET_ERROR)=1 OR DOutput(DO_START)=1) AND DOutput(SO_TROB1_EXEC)=1 THEN
        !Once we have raised the error and the task is already executing
	!we reset the error and start signals to 0.
        Reset DO_RESET_ERROR;
        Reset DO_START;
    ENDIF
ENDPROC
ENDMODULE
