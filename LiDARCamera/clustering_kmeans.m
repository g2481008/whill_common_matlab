function [labels,C] = clustering_kmeans(ptCloud,numCluster,PLP_mean)

    % LiDARの分解能
    AngularVelocity = 50;
    resolution = 360 * AngularVelocity * 0.001 /18750; %手入力ではなく初めから変えてもよし
    Z_Comp = (tan(deg2rad(2))/tan(deg2rad(resolution)));

    Pclouds_resolution = [ptCloud(:,1),ptCloud(:,2),ptCloud(:,3)/Z_Comp];
    [labels,C] = kmeans(Pclouds_resolution,numCluster,'Start',PLP_mean);

end