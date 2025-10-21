function [confirmed,tentative,alltracks,info,detection,pos,cov,vel,meas,measCov] = pedestrianTracker2(observation,currentInfo,pms)

    % if isempty(observation)
        % observation = [double(50*num^2);double(20*num);double(0.1)]';
        % observation = [1000;1000;0.1]';
        % observation = [1000;1000;0.1;0.5;0.5;0.5]';
    % end
    % num_de = size(observation,1);
    % [confirmed,tentative,alltracks,info,detection] = tracker_prog(observation,currentInfo,pms,num_de);
    [confirmed,tentative,alltracks,info,detection] = tracker_prog(observation,currentInfo.time,pms.MeasurementNoise);

    [pos,cov] = getTrackPositions(confirmed,pms.positionSelector);
    vel = getTrackVelocities(confirmed,pms.velocitySelector);
    detectionsP_meas = [detection{:}];
    % detectionsP_meas = [detection(:)];
    if isempty(detectionsP_meas)
        meas = NaN(3,1);
        measCov = [];
    else
        meas = cat(2,detectionsP_meas.Measurement);
        measCov = cat(3,detectionsP_meas.MeasurementNoise);
    end
    
end


% function [confirmed,tentative,alltracks,info,detection] = tracker_prog(observation,currentInfo,pms,num_de)
%     persistent tracker
%     if isempty(tracker)
%        tracker = trackerJPDA('TrackLogic','History','ConfirmationThreshold',[10 30],'DeletionThreshold',[20 40],'AssignmentThreshold',[9 30],'ClutterDensity',1e-10,'MaxNumTracks',500,'MaxNumTracksPerCluster',1,'MaxNumDetectionsPerCluster',5,'EnableMemoryManagement',true,'InitializationThreshold',0.1,'TimeTolerance',0.1,'DetectionProbability',0.8,'OOSMHandling','Retrodiction','MaxNumOOSMSteps',3,'MaxNumDetectionsPerSensor',1000);%'MaxNumEvents',5,,'MaxNumDetections',Inf,'MaxNumDetectionsPerSensor',1000
%     end
%     detection = cell(num_de,1);
%     for i = 1:num_de
%         detection{i} = objectDetection(currentInfo.time,observation(i,1:3)','MeasurementNoise',pms.MeasurementNoise,'ObjectAttributes',struct('myProperty',i));
%     end
%     [confirmed,tentative,alltracks,info] = tracker(detection,currentInfo.time);
% end