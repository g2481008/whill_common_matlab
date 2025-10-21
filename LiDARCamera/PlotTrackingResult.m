function PlotTrackingResult(trackP,detectionP,confirmed,pos,vel,cov,meas,measCov,currentInfo)
    meas(3,:) = 0;
    if numel(confirmed) >= 0
            labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
            % trackP.plotTrack(pos,vel,cov,labels);
            % trackP.plotTrack(pos,vel,labels);
            % trackP.plotTrack(pos,vel);
            trackP.plotTrack(pos,labels);
    end
    % detectionP.plotDetection(meas',measCov);
    meas = meas(1:3,:);
    wheelpos = [currentInfo.X; currentInfo.Y; currentInfo.Z];
    if isempty(meas)
        meas = wheelpos;
    else
        meas = horzcat(meas,wheelpos);
    end
    detectionP.plotDetection(meas');
end