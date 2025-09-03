function [confirmed,tentative,alltracks,info,detection,detection_forplot,pos,cov,vel,meas,measCov] = pedestrianTracker2(observation,currentInfo,pms)

    [confirmed,tentative,alltracks,info,detection,detection_forplot] = tracker_prog(observation,currentInfo,pms);

    [pos,cov] = getTrackPositions(confirmed,pms.positionSelector);
    vel = getTrackVelocities(confirmed,pms.velocitySelector);
    detectionsP_meas = [detection{:}];
    if isempty(detectionsP_meas)
        meas = NaN(3,1);
        measCov = [];
    else
        meas = cat(2,detectionsP_meas.Measurement);
        measCov = cat(3,detectionsP_meas.MeasurementNoise);
    end
    
end