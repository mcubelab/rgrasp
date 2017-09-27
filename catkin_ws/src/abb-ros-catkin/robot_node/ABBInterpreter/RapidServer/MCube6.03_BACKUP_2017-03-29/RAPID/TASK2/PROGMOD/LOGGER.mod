MODULE LOGGER

    !////////////////
    !GLOBAL VARIABLES
    !////////////////
    !PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    PERS string ipController2;
    PERS num loggerPort2;

    !Clock Synchronization
    PERS bool startLog2;
    PERS bool startRob2;

    !// Mutex between logger and changing the tool and work objects
    PERS bool frameMutex2;

    !Robot configuration	
    PERS tooldata currentTool2;
    PERS wobjdata currentWobj2;
    VAR speeddata currentSpeed2;
    VAR zonedata currentZone2;
    PERS num loggerWaitTime2;

    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;
        TPWrite "LOGGER: Logger waiting for incomming connections ...";
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            IF SocketGetStatus(clientSocket)<>SOCKET_CONNECTED THEN
                TPWrite "LOGGER: Problem serving an incomming connection.";
                TPWrite "LOGGER: Try reconnecting.";
            ENDIF
            !Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
        TPWrite "LOGGER: Connected to IP "+clientIP;
    ENDPROC

    PROC main()
        VAR string data;
        VAR robtarget position;
        VAR jointtarget joints;
        !VAR fcforcevector forceTorque;
        VAR string sendString;
        VAR bool connected;

        VAR string date;
        VAR string time;
        VAR clock timer;

        frameMutex2:=FALSE;

        startLog2:=FALSE;
        WaitUntil startRob2\PollRate:=0.01;
        startLog2:=TRUE;
        ClkStart timer;

        date:=CDate();
        time:=CTime();

        connected:=FALSE;
        ServerCreateAndConnect ipController2,loggerPort2;
        connected:=TRUE;
        WHILE TRUE DO

            !Cartesian Coordinates
            WHILE (frameMutex2) DO
                !// If the frame is being changed, wait
            ENDWHILE
            frameMutex2:=TRUE;
            position:=CRobT(\Tool:=currentTool2\WObj:=currentWObj2);
            frameMutex2:=FALSE;
            data:="# 0 ";
            data:=data+date+" "+time+" ";
            data:=data+NumToStr(ClkRead(timer),2)+" ";
            data:=data+NumToStr(position.trans.x,1)+" ";
            data:=data+NumToStr(position.trans.y,1)+" ";
            data:=data+NumToStr(position.trans.z,1)+" ";
            data:=data+NumToStr(position.rot.q1,3)+" ";
            data:=data+NumToStr(position.rot.q2,3)+" ";
            data:=data+NumToStr(position.rot.q3,3)+" ";
            data:=data+NumToStr(position.rot.q4,3);
            !End of string	
            IF connected=TRUE THEN
                SocketSend clientSocket\Str:=data;
            ENDIF
            !WaitTime loggerWaitTime;

            !Joint Coordinates
            joints:=CJointT();
            data:="# 1 ";
            data:=data+date+" "+time+" ";
            data:=data+NumToStr(ClkRead(timer),2)+" ";
            data:=data+NumToStr(joints.robax.rax_1,2)+" ";
            data:=data+NumToStr(joints.robax.rax_2,2)+" ";
            data:=data+NumToStr(joints.robax.rax_3,2)+" ";
            data:=data+NumToStr(joints.robax.rax_4,2)+" ";
            data:=data+NumToStr(joints.robax.rax_5,2)+" ";
            data:=data+NumToStr(joints.robax.rax_6,2);
            !End of string
            IF connected=TRUE THEN
                SocketSend clientSocket\Str:=data;
            ENDIF
            WaitTime loggerWaitTime2;
        ENDWHILE
    ERROR
        IF ERRNO=ERR_SOCK_CLOSED THEN
            TPWrite "LOGGER: Client has closed connection.";
        ELSE
            TPWrite "LOGGER: Connection lost: Unknown problem.";
        ENDIF
        connected:=FALSE;
        !Closing the server
        SocketClose clientSocket;
        SocketClose serverSocket;
        !Reinitiate the server
        ServerCreateAndConnect ipController2,loggerPort2;
        connected:=TRUE;
        RETRY;
    ENDPROC

ENDMODULE
