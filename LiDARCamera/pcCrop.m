function [pCloudCrop, pCloudLCS] = pcCrop(pCloud,currentInfo,pms_ROI)
    % -----剛体変換---------------------------------------------------------
    trans   = [currentInfo.X;currentInfo.Y;currentInfo.Z];
    rot     = [cos(currentInfo.yaw)   sin(currentInfo.yaw)  0;
               -sin(currentInfo.yaw)  cos(currentInfo.yaw)  0;
               0                0               1];
    ptCloudTrans = pctransform(pCloud,rigid3d(rot,trans'));
    
    % -----観測領域内の点群抽出-----------------------------------------------
    indices_obs = pms_ROI.observe.xRoi(1)<=ptCloudTrans.Location(:,1) ...
                & ptCloudTrans.Location(:,1)<pms_ROI.observe.xRoi(2) ...
                & pms_ROI.observe.yRoi(1)<=ptCloudTrans.Location(:,2) ...
                & ptCloudTrans.Location(:,2)<pms_ROI.observe.yRoi(2) ...
                & pms_ROI.observe.zRoi(1)<=ptCloudTrans.Location(:,3) ...
                & ptCloudTrans.Location(:,3)<pms_ROI.observe.zRoi(2);
    pCloudCrop = select(ptCloudTrans,indices_obs);
    pCloudLCS = select(pCloud,indices_obs);

    % -----推定領域内の点群を取得-------------------------------------
    estFlowRoi.x = [currentInfo.X+pms_ROI.estimate.xRoi(1) currentInfo.X+pms_ROI.estimate.xRoi(2)];
    estFlowRoi.y = [currentInfo.Y+pms_ROI.estimate.yRoi(1) currentInfo.Y+pms_ROI.estimate.yRoi(2)];
    estFlowRoi.z = [-2 1];
    indices_est = estFlowRoi.x(1)<=pCloudCrop.Location(:,1) & pCloudCrop.Location(:,1)<estFlowRoi.x(2) ...
            & estFlowRoi.y(1)<=pCloudCrop.Location(:,2) & pCloudCrop.Location(:,2)<estFlowRoi.y(2) ...
            & estFlowRoi.z(1)<=pCloudCrop.Location(:,3) & pCloudCrop.Location(:,3)<estFlowRoi.z(2);
    pCloudCrop = select(pCloudCrop,indices_est);
    pCloudLCS = select(pCloudLCS,indices_est);
end