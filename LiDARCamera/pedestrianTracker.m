function [obsPos,obsCov,obsVel,obsDetections,obsConfirmedTracks] = pedestrianTracker(pCloudLocation,currentInfo,pms,labels)   %#codegen
    
    ptCloud = pointCloud(pCloudLocation);
    
    % JPDAFによるオブジェクト状態推定関数
    [obsDetections,obsConfirmedTracks] = ...
        lidarTracker(ptCloud,currentInfo,pms,labels);
    
    % オブジェクト状態抽出
    if not(isempty(obsConfirmedTracks))
        [obsPos,obsCov] = getTrackPositions(obsConfirmedTracks,pms.positionSelector);
        obsVel          = getTrackVelocities(obsConfirmedTracks,pms.velocitySelector);
    else
        obsPos = [];
        obsCov = [];
        obsVel = [];
    end
end

%% Sub function
% JPDAFによるオブジェクト状態推定
function [detections,confirmedTracks] = lidarTracker(ptCloud,currentInfo,pms,labels)

    persistent tracker detectableTracksInput currentNumTracks trackFlag
    
    if isempty(tracker) || isempty(detectableTracksInput) || isempty(currentNumTracks)

        tracker = trackerJPDA('TrackLogic','History',...
                                'AssignmentThreshold',[200 500],...
                                'ClutterDensity',1e-5,...
                                'ConfirmationThreshold',[6 10],...
                                'DeletionThreshold',[20 30],...
                                'HasDetectableTrackIDsInput',true,...
                                'InitializationThreshold',0,...
                                'MaxNumTracks',30);
        
        detectableTracksInput = zeros(tracker.MaxNumTracks,2);
        currentNumTracks = 0;
        trackFlag = false;
    end

    % Detector model
    detections = boundingBoxDetector(ptCloud,currentInfo,pms,labels);
    if not(isempty(detections)) && trackFlag == false
        trackFlag = true;
    end
    
    if trackFlag == true
        % Call tracker
        [confirmedTracks,~,allTracks] = tracker(detections,currentInfo.time,detectableTracksInput(1:currentNumTracks,:));
        % Update the detectability input
        currentNumTracks = numel(allTracks);
        detectableTracksInput(1:currentNumTracks,:) = helperCalcDetectability(allTracks,[1 3 6]);
    else
        confirmedTracks = [];
    end
end

% バウンディングボックスによるオブジェクト検出
function detections = boundingBoxDetector(ptCloud,currentInfo,pms,labels)
    numClusters = max(labels);
    % -----Computing obstacle's point clouds and positions-----------------
    bboxes = zeros(3,numClusters);
    obsNum = 0;
    for i = 1:numClusters
	    pcCluster = ptCloud.Location(labels==i,:);
        [xMin, xMax] = bounds(pcCluster(:,1));
        [yMin, yMax] = bounds(pcCluster(:,2));
        x = (xMin + xMax)/2;
        y = (yMin + yMax)/2;
        r = sqrt(x^2 + y^2);
        obsNum = obsNum + 1;
        bboxes(:,obsNum) = [x;y;r];
    end

    numBoxes = size(bboxes,2);
    detections = cell(numBoxes,1);
    for i = 1:numBoxes
        detections{i} = objectDetection(currentInfo.time,cast(bboxes(:,i),'double'),...
            'MeasurementNoise',pms.boxMeasurementNoise,'ObjectAttributes',struct,...
            'MeasurementParameters',{});
    end
end

% 最大トラック数を指定(MathWorksから引用)
function detectableTracksInput = helperCalcDetectability(tracks,posIndices)
% This is a helper function to calculate the detection probability of
% tracks for the lidar tracking example. It may be removed in a future
% release.

% Copyright 2019 The MathWorks, Inc.

% The bounding box detector has low probability of segmenting point clouds
% into bounding boxes are distances greater than 40 meters. This function
% models this effect using a state-dependent probability of detection for
% each tracker. After a maximum range, the Pd is set to a high value to
% enable deletion of track at a faster rate.
if isempty(tracks)
    detectableTracksInput = zeros(0,2);
    return;
end
rMax = 75;
rAmbig = 40;
stateSize = numel(tracks(1).State);
posSelector = zeros(3,stateSize);
posSelector(1,posIndices(1)) = 1;
posSelector(2,posIndices(2)) = 1;
posSelector(3,posIndices(3)) = 1;
pos = getTrackPositions(tracks,posSelector);
if coder.target('MATLAB')
    trackIDs = [tracks.TrackID];
else
    trackIDs = zeros(1,numel(tracks),'uint32');
    for i = 1:numel(tracks)
        trackIDs(i) = tracks(i).TrackID;
    end
end
[~,~,r] = cart2sph(pos(:,1),pos(:,2),pos(:,3));
probDetection = 0.9*ones(numel(tracks),1);
probDetection(r > rAmbig) = 0.4;
probDetection(r > rMax) = 0.99;
detectableTracksInput = [double(trackIDs(:)) probDetection(:)];
end