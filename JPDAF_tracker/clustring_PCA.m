function [new_labels, point_cloud] = clustring_PCA(pCloudLocation,currentInfo,pms,pms_roi)
%clustring_PCA この関数の概要をここに記述
%   詳細説明をここに記述
    ptCloud = pointCloud(pCloudLocation);
    nonGPClouds = pcCrop(ptCloud,currentInfo,pms_roi.observe);
    
    % ダウンサンプリング
    if not(isempty(nonGPClouds))
        nonGPClouds = pcdownsample(nonGPClouds,'gridAverage',pms.gridStep);
    end
    point_cloud = nonGPClouds.Location;
    % -----クラスタリング(少数クラスタの除去)---------------------------
    [labels,numClusters] = ...
        pcsegdist(nonGPClouds,pms.minDistance,'NumClusterPoints',pms.numClusterPoints);

    % -----オブジェクトを識別(歩行者点群抽出)---------------------------
    new_labels = double(labels);
    count_cluster = 0;
    for j = 1:numClusters   % ラベルごとに識別
        pcCluster = nonGPClouds.Location(labels==j,:);
        [~,~,latent,~,explained,~] = pca(pcCluster);
        Contri_rate = explained/100;
        if any(Contri_rate< 0.007 | latent>0.5) % 主成分分析の寄与率と固有値の閾値で判別
            new_labels(labels == j) = 0;
        else
            count_cluster = count_cluster + 1;
            new_labels(labels == j) = count_cluster;
        end
    end
%% Sub function
% 関心領域内点群切り取り
function pCloudCrop = pcCrop(pCloud,currentInfo,observeInfo)
    % -----剛体変換---------------------------------------------------------
    trans   = [currentInfo.X;currentInfo.Y;currentInfo.Z];
    rot     = [cos(currentInfo.yaw)   sin(currentInfo.yaw)  0;
               -sin(currentInfo.yaw)  cos(currentInfo.yaw)  0;
               0                0               1];
    ptCloudTrans = pctransform(pCloud,rigid3d(rot,trans'));
    
    % -----観測領域内の点群抽出-----------------------------------------------
    indices = observeInfo.xRoi(1)<=ptCloudTrans.Location(:,1) ...
                & ptCloudTrans.Location(:,1)<observeInfo.xRoi(2) ...
                & observeInfo.yRoi(1)<=ptCloudTrans.Location(:,2) ...
                & ptCloudTrans.Location(:,2)<observeInfo.yRoi(2) ...
                & observeInfo.zRoi(1)<=ptCloudTrans.Location(:,3) ...
                & ptCloudTrans.Location(:,3)<observeInfo.zRoi(2);
    pCloudCrop = select(ptCloudTrans,indices);
end

end