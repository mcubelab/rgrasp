MODULE SPECIAL_COMMANDS


CONST jointtarget Target_00:= [[-5.69,-62.98,45.25,0.07,-72.27,5.46],[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
CONST jointtarget Target_01:= [[-5.86,-35.46,-13.37,0.1,-41.16,5.57],[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];

CONST robtarget Target_10:=[[600,700,1100],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget Target_40:=[[600,700,805],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
CONST robtarget Target_50:=[[600,700,800],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];

PROC Catching()
    !TPWrite "Nikhil's Catching code!!";
		
	VAR robtarget Target_20;
	VAR robtarget Target_30;
	
	Target_20:=[[600,700,specialParams{1}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	Target_30:=[[600,700,specialParams{2}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	
	confL\Off;
	!SingArea \Off;
	MoveAbsJ Target_01, v100, currentZone, currentTool \Wobj:=currentWobj;
		
 	MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{3}; 
 	MoveL Target_20,vmax,z5,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{4};
 	MoveL Target_30,vmax,z10,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{5};
 	MoveL Target_40,vmax,z0,currentTool\WObj:=currentWobj;
 	MoveL Target_50,v100,fine,currentTool\WObj:=currentWobj;
	!SingArea \Wrist;

ENDPROC

PROC Env2Fing()
	!TPWrite "Nikhil's E2F code!!";
	VAR robtarget Target_60;
	VAR robtarget Target_70;
	VAR robtarget Target_80;
	
	Target_60:=[[600,700,specialParams{1}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	Target_70:=[[600,700,specialParams{2}],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	Target_80:=[[600,700,specialParams{2}-10],[0.7071,0,0,-0.7071],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	
	confL\Off;
	!SingArea \Off;
	MoveAbsJ Target_00, v100, currentZone, currentTool \Wobj:=currentWobj;
		
 	!MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
	MoveL Target_50,v100,fine,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{3};
 	MoveL Target_60,vmax,z20,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{4};
 	MoveL Target_70,vmax,z5,currentTool\WObj:=currentWobj;
 	MoveL Target_80,v300,fine,currentTool\WObj:=currentWobj;
	!SingArea \Wrist;

ENDPROC

PROC Vibrate()
	
	
	VAR robtarget actualPos;
	VAR clock Vibtimer;
	TPWrite "Vibration code!!";
	actualPos := CRobT(\Tool:=tool0 \WObj:=wobj0);
	ClkReset Vibtimer;
	ClkStart Vibtimer;
	!SingArea \Off;
	WHILE ClkRead(Vibtimer) < specialParams{1} DO
		MoveL Offs(actualPos,0,-1,0), vmax, z0, tool0 \WObj:=wobj0;
		MoveL Offs(actualPos,-0.7071,-1.7071,0), vmax, z0, tool0 \WObj:=wobj0;
		MoveL Offs(actualPos,0,-2.4142,0), vmax, z0, tool0 \WObj:=wobj0;
		MoveL Offs(actualPos,0,-1.4142,0), vmax, z0, tool0 \WObj:=wobj0;				
		MoveL Offs(actualPos,0.7071,-0.7071,0), vmax, z0, tool0 \WObj:=wobj0;				
		MoveL actualPos, vmax, z0, tool0 \WObj:=wobj0;
	ENDWHILE
	ClkStop Vibtimer;
	MoveL actualPos, vmax, fine, tool0 \WObj:=wobj0;
	!SingArea \Wrist;
			
ENDPROC

PROC Fing2EnvHD()
		
	VAR robtarget Target_510;
	VAR robtarget Target_520;
	VAR robtarget Target_530;
	VAR robtarget Target_540;
	VAR jointtarget Target_550;
	
	Target_510:=[[1300,300,specialParams{1}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	Target_520:=[[1300,300,specialParams{2}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	Target_530:=[[1300,300,specialParams{3}],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	!Target_540:=[[1300,300,specialParams{3}-30],[0,0.7071,0.7071,0],[-1,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
	!Target_550:= [[-1.17,48.85,-4.08,0.09,45.23,-1.03],[9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];
	
	TPWrite "F2EHD code!!";
	
	confL\Off;
	!SingArea \Off;
	!MoveAbsJ Target_550, v100, currentZone, currentTool \Wobj:=currentWobj;
		
 	!MoveL Target_10,v100,fine,currentTool\WObj:=currentWobj;
	MoveL Target_510,v100,fine,currentTool\WObj:=currentWobj;
 	!WaitTime 1;
 	WorldAccLim\On:=specialParams{4};
 	MoveL Target_520,vmax,z20,currentTool\WObj:=currentWobj;
 	WorldAccLim\On:=specialParams{5};
 	MoveL Target_530,vmax,z5,currentTool\WObj:=currentWobj;
	!SingArea \Wrist;
 	!WorldAccLim\Off;
 	!MoveL Target_540,v300,fine,currentTool\WObj:=currentWobj;

ENDPROC

ENDMODULE