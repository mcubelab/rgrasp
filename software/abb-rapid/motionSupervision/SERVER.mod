MODULE SERVER
    !////////////////
    !RRI related record
    !////////////////
    !RECORD RobotData
    !    num dummy;   ! a dummy, because Rapid does not allow empty Record
    !ENDRECORD
    !Sensor Declarations
    !PERS sensor Host1:=[0,0,3];
    !PERS RobotData RobData:=[0];


    !////////////////
    !GLOBAL VARIABLES
    !////////////////
    !//To modify the default values go to method Initialize
    PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[1.2,[-18.9,-0.7,463.2],[1,0,0,0],0,0,0.054]];
    PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
    PERS speeddata currentSpeed;
    PERS zonedata currentZone;

    !// Clock Synchronization
    PERS bool startLog:=TRUE;
    PERS bool startRob:=TRUE;

    !// Mutex between logger and changing the tool and work objects
    PERS bool frameMutex:=FALSE;
    !PERS num ForceOffset := 122; !What is this?
    PERS num motSupValue := 90;

    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode;
    VAR string idCode;
    VAR num params{10};
    VAR num specialParams{5};
    VAR num nParams;
    PERS string ipController:= "192.168.37.3"; !"mCube APC 1600id"
    !PERS string ipController:="192.168.125.1";  !"mCube"
    PERS num serverPort1:=5000;
    PERS num loggerPort1:=5001;

    !//Logger sampling rate
    PERS num loggerWaitTime:= 0.01;
    !PERS num loggerWaitTime:= 0.1; This is adequate for virtual server

    !//Motion of the robot
    VAR robtarget cartesianTarget;
    VAR robtarget cartesianTargetInit;
    VAR jointtarget jointsTarget;
    VAR jointtarget jointsTargetInit;
    VAR bool moveTypeIsCart; !True means cartesian motion and False means joint motion 
    VAR bool moveComplete; !True when program pointer leaves a Move instruction.

    !//Buffered move variables
    CONST num MAX_BUFFER:=512;
    VAR num BUFFER_POS:=0;
    VAR num BUFFER_JOINT_POS:=0;
    VAR robtarget bufferTargets{MAX_BUFFER};
    VAR speeddata bufferSpeeds{MAX_BUFFER};
    VAR jointtarget bufferJointPos{MAX_BUFFER};
    VAR speeddata bufferJointSpeeds{MAX_BUFFER};

    !//External axis position variables
    VAR extjoint externalAxis;

    !//Circular move buffer
    VAR robtarget circPoint;


    !//Correct Instruction Execution and possible return values
    VAR num ok;
    VAR num collision;
    CONST num SERVER_BAD_MSG:=0;
    CONST num SERVER_OK:=1;
    CONST num SERVER_COLLISION:=2;
    CONST num SERVER_BAD_IK:=3;
    CONST num SERVER_BAD_FK:=4;
    CONST num SERVER_BUFFER_FULL:=5;

    !//Error Handler
    VAR errnum ERR_MOTIONSUP:=-1;

    !//Interrupt to trap the digital output that signals a motion suppervision error.
    VAR intnum iMotionSup;

    !// Inverse and forward kinematic results
    VAR jointtarget ik_result_j;
    VAR robtarget fk_result_c;

    !// Upper and lower joint bounds, in degrees
    VAR num upper_joint_limits{6};
    VAR num lower_joint_limits{6};

    !// RRI related
    ! Setup Interface Procedure
    !PROC RRI_Open()
    !    SiConnect Host1 \NoStop;
        ! Send and receive data cyclic with 4 ms rate
    !    SiSetCyclic Host1, RobData, 4;
    !ENDPROC
    
    ! Close Interface Procedure
    !PROC RRI_Close()
        ! Close the connection
    !    SiClose Host1;
    !ENDPROC
    !// end RRI related

    !////////////////
    !LOCAL METHODS
    !////////////////

    !Method to parse the message received from a PC through the socket
    ! Loads values on:
    ! - instructionCode.
    ! - idCode: 3 digit identifier of the command. 
    ! - nParams: Number of received parameters.
    ! - params{nParams}: Vector of received params.
    PROC ParseMsg(string msg)
        !Local variables
        VAR bool auxOk;
        VAR num ind:=1;
        VAR num newInd;
        VAR num length;
        VAR num indParam:=1;
        VAR string subString;
        VAR bool end:=FALSE;

        length:=StrMatch(msg,1,"#");
        IF length>StrLen(msg) THEN
            !Corrupt message
            nParams:=-1;
        ELSE
            !Find Instruction code
            newInd:=StrMatch(msg,ind," ")+1;
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,instructionCode);
            IF auxOk=FALSE THEN
                !Corrupt instruction code
                nParams:=-1;
            ELSE
                ind:=newInd;

                !Find Id Code
                newInd:=StrMatch(msg,ind," ")+1;
                idCode:=StrPart(msg,ind,newInd-ind-1);
                ind:=newInd;

                !Set of parameters (maximum of 8)
                WHILE end=FALSE DO
                    newInd:=StrMatch(msg,ind," ")+1;
                    IF newInd>length THEN
                        end:=TRUE;
                    ELSE
                        subString:=StrPart(msg,ind,newInd-ind-1);
                        auxOk:=StrToVal(subString,params{indParam});
                        indParam:=indParam+1;
                        ind:=newInd;
                    ENDIF
                ENDWHILE
                nParams:=indParam-1;
            ENDIF
        ENDIF
    ENDPROC

    !Handshake between server and client:
    ! - Creates socket.
    ! - Waits for incoming TCP connection.
    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;
        TPWrite "SERVER: Server waiting for incomming connections ...";
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
                TPWrite "SERVER: Problem serving an incomming connection.";
                TPWrite "SERVER: Try reconnecting.";
            ENDIF
            !//Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
        TPWrite "SERVER: Connected to IP "+clientIP;

    ENDPROC

    !//Parameter initialization
    !// Loads default values for
    !// - Tool.
    !// - WorkObject.
    !// - Zone.
    !// - Speed.
    !// Gets joint bounds so they can be used later
    PROC Initialize()
        VAR string path;

        currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
        currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
        currentSpeed:=[100,50,5000,1000];
        currentZone:=[FALSE,0.3,0.3,0.3,0.03,0.3,0.03];        !z0

        !// Get all of the joint bounds for later use
        FOR i FROM 1 TO 6 DO
            path:="MOC/ARM/rob1_"+NumToStr(i,0);
            ReadCfgData path,"upper_joint_bound",upper_joint_limits{i};
            ReadCfgData path,"lower_joint_bound",lower_joint_limits{i};

            !// The joint limits are in radians, so convert these to degrees
            upper_joint_limits{i}:=upper_joint_limits{i}*180.0/pi;
            lower_joint_limits{i}:=lower_joint_limits{i}*180.0/pi;
        ENDFOR

    ENDPROC

    !/////////////////
    !//Main procedure
    !/////////////////
    PROC main()
        !//Local variables
        VAR string receivedString;
        VAR string sendString;
        VAR string addString;
        VAR bool connected;        ! //Client connected
        VAR bool reconnected;      ! //Reconnection During the iteration
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;
        VAR clock timer;
        VAR num quatMag;
        VAR num ind;
        VAR bool move;

        !//Book error number for error handler
        BookErrNo ERR_MOTIONSUP;

        !//Configure the interrupt "iMotionReset"
        !//to traps a raise on the digital output "USER_RESET_MOTION"
        !//Meant to signal the need to restart the motion of the robot.
        !//SetDO USER_RESET_MOTION, 0;
        !CONNECT iMotionReset WITH resetMotion;
        !ISignalDO USER_RESET_MOTION,1,iMotionReset;
        
        !//Configures the interrupt "iMotionSup"
        !//to trap a raise to 1 on the digital output "SO_MOTIONSUP_ON"
        !//The interrrupt witll execute the routing CollitionDet 
        CONNECT iMotionSup WITH CollitionDet;
        ISignalDO SO_MOTIONSUP_ON,1,iMotionSup;
        
        !// We are not currently changing the frame
        frameMutex:=FALSE;

        !//Motion configuration
        SingArea\Off;
        !Use \Wrist if we want the robot to charge its course to avoid singularities.
        ConfL\Off;
        !Use \On if we want the robot to enforce the configuration specified in MoveL
        ConfJ\Off;
        moveComplete:=TRUE;
        collision:=0;
        MotionSup\On\TuneValue:=motSupValue;

        !//Timer synchronization with Logger
        startRob:=TRUE;
        WaitUntil startLog\PollRate:=0.01;
        ClkStart timer;

        !//Initialization of WorkObject, Tool, Speed, Zone and Inertia. 
        !//Get the joint limits from Robot Configuration.
        Initialize;

        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort1;
        connected:=TRUE;

        !//Infinite loop to serve commands
        WHILE TRUE DO
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Correctness of executed instruction.
            reconnected:=FALSE;
            !//Has communication dropped after receiving a command?
            addString:="";
            !//String to add to the reply.

            !//Receive a command
            SocketReceive clientSocket\Str:=receivedString\Time:=WAIT_MAX;
            ParseMsg receivedString;
            collision:=0;

            !//Execution of the command
            TEST instructionCode
            CASE 0:
                !Ping
                IF nParams=0 THEN
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 1:
                !Set Cartesian Coordinates
                IF nParams=7 THEN
                    !Linear moves
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                  [params{4},params{5},params{6},params{7}],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=FALSE;
                    SingArea \Wrist;
                    cartesianTargetInit:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                    moveTypeIsCart:=TRUE;
                    MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                    moveComplete:=TRUE;
                ELSEIF nParams=8 THEN
                    !If there iss an extra parameter
                    !it means we want to do a cartesian move interpolating in joints.
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                   [params{4},params{5},params{6},params{7}],
                                   [0,0,0,0],
                                   [9E9,9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=FALSE;
                    SingArea \Wrist;
                    cartesianTargetInit:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                    moveTypeIsCart:=TRUE;
                    MoveJ cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                    moveComplete:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 2:
                !Set Joint Coordinates
                IF nParams=6 THEN
                    move:=true;
                    FOR i FROM 1 TO 6 DO
                        IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                            !// If not, then we'll tell the user that
		            !// their forward kinematics are invalid
                            ok:=SERVER_BAD_FK;
                            move:=false;
                        ENDIF
                    ENDFOR
                    IF move=TRUE THEN
                        jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],[0,9E9,9E9,9E9,9E9,9E9]];
                        ok:=SERVER_OK;
                        moveComplete:=FALSE;
                        jointsTargetInit:=CJointT();
                        moveTypeIsCart:=FALSE;
                        MoveAbsJ jointsTarget,currentSpeed,currentZone,currentTool
                        \Wobj:=currentWobj;
                        moveComplete:=TRUE;
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 3:
                !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams=0 THEN
                    cartesianPose:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                    addString:=NumToStr(cartesianPose.trans.x,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.y,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.z,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q1,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q2,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q3,4)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q4,4);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 4:
                !Get Joint Coordinates
                IF nParams=0 THEN
                    jointsPose:=CJointT();
                    addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_6,2);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 6:
                !Specify Tool
                IF nParams=7 THEN
                    WHILE (frameMutex) DO
                        !// If the frame is being used by logger, wait here
                    ENDWHILE
                    frameMutex:=TRUE;
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    frameMutex:=FALSE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 7:
                !Specify Work Object
                IF nParams=7 THEN
                    WHILE (frameMutex) DO
                        !// If the frame is being used by logger, wait here
                    ENDWHILE
                    frameMutex:=TRUE;
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    frameMutex:=FALSE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 8:
                !Specify Speed of the Robot
                IF nParams=2 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 9:
                !Specify ZoneData
                IF nParams=4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep:=TRUE;
                        currentZone.pzone_tcp:=0.0;
                        currentZone.pzone_ori:=0.0;
                        currentZone.zone_ori:=0.0;
                    ELSE
                        currentZone.finep:=FALSE;
                        currentZone.pzone_tcp:=params{2};
                        currentZone.pzone_ori:=params{3};
                        currentZone.zone_ori:=params{4};
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 12:
                !Inverse Kinematics Solver
                IF nParams=7 THEN
                    !// First, let's make sure the quaternion is normalized
                    IF Abs(1.0-Sqrt(params{4}*params{4}+params{5}*params{5}+params{6}*params{6}+params{7}*params{7}))>0.001 THEN
                        !// If not, then we cannot find the inverse kinematics for this pose
                        ok:=SERVER_BAD_IK;
                    ELSE
                        !// Otherwise, let's normalize our quaternion 
                        cartesianTarget:=[[params{1},params{2},params{3}],
                            NOrient([params{4},params{5},params{6},params{7}]),
                            [0,0,0,0],
                            [9E9,9E9,9E9,9E9,9E9,9E9]];
                        ok:=SERVER_OK;

                        !// Now calculate the joint angles, keeping in mind that if we specified an 
                        !// impossible configuration, this will generate an error (See error handler below)
                        ik_result_j:=CalcJointT(cartesianTarget,currentTool,\WObj:=currentWObj);

                        !// Store our result in a string to return to the user
                        addString:=NumToStr(ik_result_j.robax.rax_1,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_2,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_3,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_4,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_5,2)+" ";
                        addString:=addString+NumToStr(ik_result_j.robax.rax_6,2);
                        !End of string
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 13:
                ! Forward Kinematics Solver
                IF nParams=6 THEN
                    ok:=SERVER_OK;

                    !// First, let's make sure the specified joint angles are within range
                    FOR i FROM 1 TO 6 DO
                        IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                            !// If not, then we'll tell the user that their forward kinematics are invalid
                            ok:=SERVER_BAD_FK;
                        ENDIF
                    ENDFOR

                    !// If our joints are within limits, then let's carry on
                    IF ok=SERVER_OK THEN
                        !// Create a joint target, and then calculate the corresponding cartesian pose
                        jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],
                                [0,9E9,9E9,9E9,9E9,9E9]];
                        fk_result_c:=CalcRobT(jointsTarget,currentTool,\WObj:=currentWObj);

                        !// Now add this pose to our return string
                        addString:=NumToStr(fk_result_c.trans.x,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.trans.y,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.trans.z,2)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q1,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q2,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q3,4)+" ";
                        addString:=addString+NumToStr(fk_result_c.rot.q4,4);
                        !End of string
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 14:
                !Specify Inertia
                IF nParams=7 THEN
                    currentTool.tload.mass:=params{1};
                    currentTool.tload.cog.x:=params{2};
                    currentTool.tload.cog.y:=params{3};
                    currentTool.tload.cog.z:=params{4};
                    currentTool.tload.ix:=params{5};
                    currentTool.tload.iy:=params{6};
                    currentTool.tload.iz:=params{7};
                    currentTool.tload.aom.q1:=1.0;
                    currentTool.tload.aom.q2:=0.0;
                    currentTool.tload.aom.q3:=0.0;
                    currentTool.tload.aom.q4:=0.0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 15:
                !Specify Max acceleration
                IF nParams=2 THEN
                    PathAccLim TRUE\AccMax:=params{1},TRUE\DecelMax:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 30:
                !Add Cartesian Coordinates to buffer
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                    [params{4},params{5},params{6},params{7}],
                                    [0,0,0,0],
                                    externalAxis];
                    IF BUFFER_POS<MAX_BUFFER THEN
                        BUFFER_POS:=BUFFER_POS+1;
                        bufferTargets{BUFFER_POS}:=cartesianTarget;
                        bufferSpeeds{BUFFER_POS}:=currentSpeed;
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31:
                !Clear Cartesian Buffer
                IF nParams=0 THEN
                    BUFFER_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32:
                !Get Buffer Size
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33:
                !Execute moves in cartesianBuffer as linear moves
                IF nParams=0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO
                        IF collision=0 THEN
                            moveComplete:=FALSE;
                            cartesianTargetInit:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                            moveTypeIsCart=TRUE;
                            MoveL bufferTargets{i},bufferSpeeds{i},currentZone,currentTool\WObj:=currentWobj;
                            moveComplete:=TRUE;
                        ENDIF
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 35:
                !Specify circPoint for circular move, and then wait on toPoint
                IF nParams=7 THEN
                    circPoint:=[[params{1},params{2},params{3}],
                            [params{4},params{5},params{6},params{7}],
                            [0,0,0,0],
                            externalAxis];
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 36:
                !specify toPoint, and use circPoint specified previously
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                    [params{4},params{5},params{6},params{7}],
                                    [0,0,0,0],
                                    externalAxis];
                    MoveC circPoint,cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 37:
                !Add Joint Positions to buffer
                IF nParams=6 THEN
                    FOR i FROM 1 TO 6 DO !Check for joint limits
                        IF params{i}>upper_joint_limits{i}OR params{i}<lower_joint_limits{i} THEN
                            ok:=SERVER_BAD_FK;
                        ENDIF
                    ENDFOR
                    IF ok<>SERVER_BAD_FK THEN
                        jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],[0,9E9,9E9,9E9,9E9,9E9]];
                        IF BUFFER_JOINT_POS<MAX_BUFFER THEN
                            BUFFER_JOINT_POS:=BUFFER_JOINT_POS+1;
                            bufferJointPos{BUFFER_JOINT_POS}:=jointsTarget;
                            bufferJointSpeeds{BUFFER_JOINT_POS}:=currentSpeed;
			    ok:=SERVER_OK;
                        ELSE
                            ok:=SERVER_BUFFER_FULL;
                        ENDIF
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 38:
                !Clear Joint Position Buffer
                IF nParams=0 THEN
                    BUFFER_JOINT_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 39:
                !Get Joint Position Buffer Size
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_JOINT_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 40:
                !Execute moves in bufferJointPos
                IF nParams=0 THEN
                    moveComplete:=FALSE;
                    FOR i FROM 1 TO (BUFFER_JOINT_POS) DO
                        IF collision=0 THEN
                            jointsTargetInit:=CJointT();
                            moveTypeIsCart:=FALSE;
                            IF i=BUFFER_JOINT_POS THEN
                                moveComplete:=FALSE;
                                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},currentZone,currentTool,\Wobj:=currentWobj;
                                moveComplete:=TRUE;
                            ELSE
                                moveComplete:=FALSE;
                                MoveAbsJ bufferJointPos{i},bufferJointSpeeds{i},z1,currentTool,\Wobj:=currentWobj;
                                moveComplete:=TRUE;
                            ENDIF
                        ENDIF
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !End the added section for buffer.

            !CASE 50:
                !Establish RRI Connection
                !// create connection using rri
                !IF nParams=0 THEN
                !    RRI_Open;
                !    ok:=SERVER_OK;
                !ELSE
                !    ok:=SERVER_BAD_MSG;
                !ENDIF
            !CASE 51:
                !Close RRI Connection
                !// create connection using rri
            !    IF nParams=0 THEN
            !        RRI_Close;
            !        ok:=SERVER_OK;
            !    ELSE
            !        ok:=SERVER_BAD_MSG;
            !    ENDIF
            CASE 60:
                ! Activate SoftMove
                IF nParams=10 THEN
                    CSSOffsetTune \RefFrame:=CSS_REFFRAME_WOBJ, CSS_NEGX, ForceOffset \StiffnessNonSoftDir:=30;
                    TPWrite NumToStr(ForceOffset,0);
                    IF params{9} = 1 THEN
                        CSSAct \RefFrame:=params{1} \RefOrient:=[params{2},params{3},params{4},params{5}], params{6} \StiffnessNonSoftDir:=params{8} \Stiffness:=params{7} \AllowMove  \Ramp:=params{10}; 
                    ELSE
                        CSSAct \RefFrame:=params{1} \RefOrient:=[params{2},params{3},params{4},params{5}], params{6} \StiffnessNonSoftDir:=params{8} \Stiffness:=params{7} \Ramp:=params{10}; 
                    ENDIF
                    CSSForceOffsetAct CSS_NEGX, ForceOffset;
                    ok:=SERVER_OK;
                    
                    !CSSAct [\RefFrame] [\RefOrient] SoftDir [\StiffnessNonSoftDir]  [\Stiffness] [\AllowMove] [\Ramp]
                    !CSSAct \RefFrame:= CSS_REFFRAME_WOBJ, CSS_Z \StiffnessNonSoftDir:=30 \Stiffness:=0;
                    
                    !param{1}
                    !CSS_REFFRAME_TOOL 1 Softness direction will be in relation to current tool.
                    !CSS_REFFRAME_WOBJ 2 Softness direction will be in relation to current work object.
                    
                    !param{2}
                    !CSS_X := 1;
                    !CSS_Y := 2;
                    !CSS_Z := 3;
                    !CSS_XY := 4;
                    !CSS_XZ := 5;
                    !CSS_YZ := 6;
                    !CSS_XYZ := 7;
                    !CSS_XYRZ := 8;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 61:
                ! Deactivate SoftMove
                IF nParams=7 THEN
                    !Linear moves
                    moveComplete:=FALSE;
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                  [params{4},params{5},params{6},params{7}],
                                  [0,0,0,0],
                                  [9E9,9E9,9E9,9E9,9E9,9E9]];
                    ok:=SERVER_OK;
                    moveComplete:=TRUE;
                    
                    CSSDeactMoveL cartesianTarget,currentSpeed,currentTool \WObj:=currentWObj;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 99:
                !Close Connection
                IF nParams=0 THEN
                    TPWrite "SERVER: Client has closed connection.";
                    connected:=FALSE;
                    !Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;

                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort1;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            DEFAULT:
                TPWrite "SERVER: Illegal instruction code";
                ok:=SERVER_BAD_MSG;
            ENDTEST

            !Finally we compose the acknowledge string to send back to the client
            abort:
            IF connected=TRUE THEN
                IF reconnected=FALSE THEN
                    IF collision=1 THEN
                        ok:=SERVER_COLLISION;
                        TPWrite "Sending message back with collision ack.";
                    ENDIF
                    sendString:=NumToStr(instructionCode,0);
                    sendString:=sendString+" "+idCode;
                    sendString:=sendString+" "+NumToStr(ok,0);
                    sendString:=sendString+" "+NumToStr(ClkRead(timer),2);
                    sendString:=sendString+" "+addString+ByteToStr(10\Char);
                    SocketSend clientSocket\Str:=sendString;
                ENDIF
            ENDIF
        ENDWHILE
    ERROR (LONG_JMP_ALL_ERR)
        ! Note that when the errors are specified in between
        ! parenthesis, it allows to catch errors from routines
        !inside the call stack.
        TPWrite "SERVER: Error Handler: Error "+NumtoStr(ERRNO,0);
        TEST ERRNO
        CASE ERR_MOTIONSUP:
            TPWrite "SERVER: Moton suppervision error.";
            
            !//Prepare robot for restarting
            !StopMove\Quick;  //Carlos was asking why these lines are commented out
            !StopMoveReset;   //Carlos was asking why these lines are commented out
            ClearPath;
            !WaitTime 5.0;
            
            !//Restart motion
            StartMove;
        
            !//For safety, go back to the initial pose where the motion started from
            MotionSup\On\TuneValue:=300;
            IF moveTypeIsCart THEN
                MoveL cartesianTargetInit,v5,z0,currentTool\WObj:=currentWobj;
            ELSE
                MoveAbsJ jointsTargetInit,v5,z0,currentTool\WObj:=currentWobj;
            ENDIF 
            MotionSup\On\TuneValue:=motSupValue;

            !//Just in case we are in the middle of a joint or cartesian move
            !//that did not finish, we set the targets to the current pose of the robot.
            WaitTime 0.5;
            cartesianTarget:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
            jointsTarget:=CJointT();

            !//Just in case we were in the middle of a buffered move, we empty the bufffers
            !Clear Joint Position Buffer
            IF moveTypeIsCart THEN
                BUFFER_POS:=0;            
            ELSE
                BUFFER_JOINT_POS:=0;
            ENDIF

            !//We go bck to the main loop
            TRYNEXT;

        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Client has closed connection.";
            connected:=FALSE;

            !//Closing the server
            TPWrite "SERVER: Closing socket and restarting.";
            SocketClose clientSocket;
            SocketClose serverSocket;

            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort1;
            reconnected:=TRUE;
            connected:=TRUE;
            RETRY;

        CASE ERR_ROBLIMIT:
            TPWrite "SERVER: Pose out of reach.";
            ok:=SERVER_BAD_IK;
            ik_result_j:=[[0,0,0,0,0,0],[0,9E9,9E9,9E9,9E9,9E9]];

            TPWrite "SERVER: ------";
            !// Skip the instruction computing the IK that caused the error
            TRYNEXT;

        DEFAULT:
            TPWrite "SERVER: Unknown error.";
            TPWrite "SERVER: I'll close the socket and restart.";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort1;
            reconnected:=TRUE;
            connected:=TRUE;
            RETRY;
        ENDTEST
        TPWrite "SERVER: ------";
    ENDPROC

    TRAP CollitionDet
        !//Routine triggered when the digital output SO_MOTIONSUP_ON is set to 1.
        !//The digital output is raised by the system when motion suppervision is triggered
        !//In that case, this task will be stopped, but INIT will detect that it is stopped,
        !//will raise a DO_RESET_ERROR, and restart this task.

        TPWrite ("Motion Supervision Trapped");

        !//Report collision
        ok:=SERVER_COLLISION;
        collision:=1;

        !TPWrite("First raise of ERROR_MOTION_SUP.");
        RAISE ERR_MOTIONSUP;
    ERROR
        !TEST ERRNO
        !    CASE ERR_MOTIONSUP:
        !        TPWrite("Second raise of ERROR_MOTION_SUP.");
        !ENDTEST
        RAISE;
    ENDTRAP

ENDMODULE
