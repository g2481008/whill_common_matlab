function [confirmed,tentative,alltracks,info,detection] = tracker_prog(observation,time,MeasurementNoise)
    persistent tracker measParams
    if isempty(tracker)
        measParams = struct('HasVelocity',false);
        tracker = trackerJPDA('FilterInitializationFcn',@initcvekf,'TrackLogic','History','ConfirmationThreshold',[10 30],'DeletionThreshold',[20 40],'AssignmentThreshold',[100 300],'ClutterDensity',1e-10,'MaxNumTracks',500,'MaxNumTracksPerCluster',1,'MaxNumDetectionsPerCluster',5,'EnableMemoryManagement',true,'InitializationThreshold',0.1,'TimeTolerance',0.1,'DetectionProbability',0.8,'OOSMHandling','Retrodiction','MaxNumOOSMSteps',3,'MaxNumDetectionsPerSensor',1000);%'MaxNumEvents',5,,'MaxNumDetections',Inf,'MaxNumDetectionsPerSensor',1000
        if isempty(observation)
            observation = [1000;1000;0.1]';
        end
    end
    num_de = size(observation,1);
    detection = cell(num_de,1);
    if num_de == 0
        detection = {};
    else
        for i = 1:num_de
            detection{i} = objectDetection(time,observation(i,1:3)','MeasurementNoise',MeasurementNoise,'ObjectAttributes',struct('myProperty',i),'MeasurementParameters',measParams);
        end
    end
    [confirmed,tentative,alltracks,info] = tracker(detection,time);
end
