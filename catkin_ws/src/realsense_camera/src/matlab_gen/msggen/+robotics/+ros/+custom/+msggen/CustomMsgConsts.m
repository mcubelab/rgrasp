classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2017 The MathWorks, Inc.
    
    properties (Constant)
        active_vision_recognition = 'active_vision/recognition'
        active_vision_recognitionRequest = 'active_vision/recognitionRequest'
        active_vision_recognitionResponse = 'active_vision/recognitionResponse'
        apc_arduino_Adc = 'apc_arduino/Adc'
        apc_arduino_Calib_Spatula = 'apc_arduino/Calib_Spatula'
        apc_arduino_Calib_SpatulaRequest = 'apc_arduino/Calib_SpatulaRequest'
        apc_arduino_Calib_SpatulaResponse = 'apc_arduino/Calib_SpatulaResponse'
        apc_arduino_HallEffectData = 'apc_arduino/HallEffectData'
        apc_arduino_LEDData = 'apc_arduino/LEDData'
        apc_arduino_Log = 'apc_arduino/Log'
        apc_arduino_MoveToPos = 'apc_arduino/MoveToPos'
        apc_arduino_MoveToPosRequest = 'apc_arduino/MoveToPosRequest'
        apc_arduino_MoveToPosResponse = 'apc_arduino/MoveToPosResponse'
        apc_arduino_RequestMessageInfo = 'apc_arduino/RequestMessageInfo'
        apc_arduino_RequestMessageInfoRequest = 'apc_arduino/RequestMessageInfoRequest'
        apc_arduino_RequestMessageInfoResponse = 'apc_arduino/RequestMessageInfoResponse'
        apc_arduino_RequestParam = 'apc_arduino/RequestParam'
        apc_arduino_RequestParamRequest = 'apc_arduino/RequestParamRequest'
        apc_arduino_RequestParamResponse = 'apc_arduino/RequestParamResponse'
        apc_arduino_RequestServiceInfo = 'apc_arduino/RequestServiceInfo'
        apc_arduino_RequestServiceInfoRequest = 'apc_arduino/RequestServiceInfoRequest'
        apc_arduino_RequestServiceInfoResponse = 'apc_arduino/RequestServiceInfoResponse'
        apc_arduino_ServiceBridge = 'apc_arduino/ServiceBridge'
        apc_arduino_ServiceBridgeRequest = 'apc_arduino/ServiceBridgeRequest'
        apc_arduino_ServiceBridgeResponse = 'apc_arduino/ServiceBridgeResponse'
        apc_arduino_Spatula_Position = 'apc_arduino/Spatula_Position'
        apc_arduino_StrainGaugeData = 'apc_arduino/StrainGaugeData'
        apc_arduino_SucCupOri = 'apc_arduino/SucCupOri'
        apc_arduino_SuctionCupSwitch = 'apc_arduino/SuctionCupSwitch'
        apc_arduino_SuctionCupSwitchRequest = 'apc_arduino/SuctionCupSwitchRequest'
        apc_arduino_SuctionCupSwitchResponse = 'apc_arduino/SuctionCupSwitchResponse'
        apc_arduino_SuctionSensData = 'apc_arduino/SuctionSensData'
        apc_arduino_TopicInfo = 'apc_arduino/TopicInfo'
        apc_arduino_TuneStrainGauge = 'apc_arduino/TuneStrainGauge'
        apc_arduino_TuneStrainGaugeRequest = 'apc_arduino/TuneStrainGaugeRequest'
        apc_arduino_TuneStrainGaugeResponse = 'apc_arduino/TuneStrainGaugeResponse'
        apc_perception_KinectCommand = 'apc_perception/KinectCommand'
        apc_perception_KinectCommandRequest = 'apc_perception/KinectCommandRequest'
        apc_perception_KinectCommandResponse = 'apc_perception/KinectCommandResponse'
        apc_planning_APC_Object = 'apc_planning/APC_Object'
        apc_planning_APC_Scenario = 'apc_planning/APC_Scenario'
        apc_planning_fake_struct = 'apc_planning/fake_struct'
        manual_fit_GetPose = 'manual_fit/GetPose'
        manual_fit_GetPoseRequest = 'manual_fit/GetPoseRequest'
        manual_fit_GetPoseResponse = 'manual_fit/GetPoseResponse'
        manual_fit_SetShelfPose = 'manual_fit/SetShelfPose'
        manual_fit_SetShelfPoseRequest = 'manual_fit/SetShelfPoseRequest'
        manual_fit_SetShelfPoseResponse = 'manual_fit/SetShelfPoseResponse'
        manual_fit_SetTotePose = 'manual_fit/SetTotePose'
        manual_fit_SetTotePoseRequest = 'manual_fit/SetTotePoseRequest'
        manual_fit_SetTotePoseResponse = 'manual_fit/SetTotePoseResponse'
        passive_vision_state = 'passive_vision/state'
        passive_vision_stateRequest = 'passive_vision/stateRequest'
        passive_vision_stateResponse = 'passive_vision/stateResponse'
        pr_msgs_Enable = 'pr_msgs/Enable'
        pr_msgs_EnableRequest = 'pr_msgs/EnableRequest'
        pr_msgs_EnableResponse = 'pr_msgs/EnableResponse'
        pr_msgs_NameTypeValue = 'pr_msgs/NameTypeValue'
        pr_msgs_ObjectPose = 'pr_msgs/ObjectPose'
        pr_msgs_ObjectPoseList = 'pr_msgs/ObjectPoseList'
        pr_msgs_SuctionData1 = 'pr_msgs/SuctionData1'
        pr_msgs_SuctionData1Request = 'pr_msgs/SuctionData1Request'
        pr_msgs_SuctionData1Response = 'pr_msgs/SuctionData1Response'
        realsense_camera_snapshot = 'realsense_camera/snapshot'
        realsense_camera_snapshotRequest = 'realsense_camera/snapshotRequest'
        realsense_camera_snapshotResponse = 'realsense_camera/snapshotResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(49, 1);
                msgList{1} = 'active_vision/recognitionRequest';
                msgList{2} = 'active_vision/recognitionResponse';
                msgList{3} = 'apc_arduino/Adc';
                msgList{4} = 'apc_arduino/Calib_SpatulaRequest';
                msgList{5} = 'apc_arduino/Calib_SpatulaResponse';
                msgList{6} = 'apc_arduino/HallEffectData';
                msgList{7} = 'apc_arduino/LEDData';
                msgList{8} = 'apc_arduino/Log';
                msgList{9} = 'apc_arduino/MoveToPosRequest';
                msgList{10} = 'apc_arduino/MoveToPosResponse';
                msgList{11} = 'apc_arduino/RequestMessageInfoRequest';
                msgList{12} = 'apc_arduino/RequestMessageInfoResponse';
                msgList{13} = 'apc_arduino/RequestParamRequest';
                msgList{14} = 'apc_arduino/RequestParamResponse';
                msgList{15} = 'apc_arduino/RequestServiceInfoRequest';
                msgList{16} = 'apc_arduino/RequestServiceInfoResponse';
                msgList{17} = 'apc_arduino/ServiceBridgeRequest';
                msgList{18} = 'apc_arduino/ServiceBridgeResponse';
                msgList{19} = 'apc_arduino/Spatula_Position';
                msgList{20} = 'apc_arduino/StrainGaugeData';
                msgList{21} = 'apc_arduino/SucCupOri';
                msgList{22} = 'apc_arduino/SuctionCupSwitchRequest';
                msgList{23} = 'apc_arduino/SuctionCupSwitchResponse';
                msgList{24} = 'apc_arduino/SuctionSensData';
                msgList{25} = 'apc_arduino/TopicInfo';
                msgList{26} = 'apc_arduino/TuneStrainGaugeRequest';
                msgList{27} = 'apc_arduino/TuneStrainGaugeResponse';
                msgList{28} = 'apc_perception/KinectCommandRequest';
                msgList{29} = 'apc_perception/KinectCommandResponse';
                msgList{30} = 'apc_planning/APC_Object';
                msgList{31} = 'apc_planning/APC_Scenario';
                msgList{32} = 'apc_planning/fake_struct';
                msgList{33} = 'manual_fit/GetPoseRequest';
                msgList{34} = 'manual_fit/GetPoseResponse';
                msgList{35} = 'manual_fit/SetShelfPoseRequest';
                msgList{36} = 'manual_fit/SetShelfPoseResponse';
                msgList{37} = 'manual_fit/SetTotePoseRequest';
                msgList{38} = 'manual_fit/SetTotePoseResponse';
                msgList{39} = 'passive_vision/stateRequest';
                msgList{40} = 'passive_vision/stateResponse';
                msgList{41} = 'pr_msgs/EnableRequest';
                msgList{42} = 'pr_msgs/EnableResponse';
                msgList{43} = 'pr_msgs/NameTypeValue';
                msgList{44} = 'pr_msgs/ObjectPose';
                msgList{45} = 'pr_msgs/ObjectPoseList';
                msgList{46} = 'pr_msgs/SuctionData1Request';
                msgList{47} = 'pr_msgs/SuctionData1Response';
                msgList{48} = 'realsense_camera/snapshotRequest';
                msgList{49} = 'realsense_camera/snapshotResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(17, 1);
                svcList{1} = 'active_vision/recognition';
                svcList{2} = 'apc_arduino/Calib_Spatula';
                svcList{3} = 'apc_arduino/MoveToPos';
                svcList{4} = 'apc_arduino/RequestMessageInfo';
                svcList{5} = 'apc_arduino/RequestParam';
                svcList{6} = 'apc_arduino/RequestServiceInfo';
                svcList{7} = 'apc_arduino/ServiceBridge';
                svcList{8} = 'apc_arduino/SuctionCupSwitch';
                svcList{9} = 'apc_arduino/TuneStrainGauge';
                svcList{10} = 'apc_perception/KinectCommand';
                svcList{11} = 'manual_fit/GetPose';
                svcList{12} = 'manual_fit/SetShelfPose';
                svcList{13} = 'manual_fit/SetTotePose';
                svcList{14} = 'passive_vision/state';
                svcList{15} = 'pr_msgs/Enable';
                svcList{16} = 'pr_msgs/SuctionData1';
                svcList{17} = 'realsense_camera/snapshot';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
