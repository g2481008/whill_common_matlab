%% Global configurations
clc; close all; clear global; clear variables; warning('off','all');
conf.pc = [ismac; isunix; ispc];
conf.mk = [":"; ":"; ";"];
conf.def= "AppData/Local/Temp/Editor";
conf.usr= pwd;
conf.fpath = split(path, conf.mk(conf.pc));
conf.fcheck= and(~contains(conf.fpath, matlabroot), ~contains(conf.fpath, conf.def));
rmpath(strjoin(conf.fpath(conf.fcheck), conf.mk(conf.pc)));
%% Sensor configurations
vehicleType = 1; % 1:CR1, 2:CR2
sensor(1) = true; % LiDAR
sensor(2) = false; % GNSS
sensor(3) = false; % Camera
sensor(4) = true; % SLAM/Localization
base_sensor = 1; % Standard sensor you use mainly. No standard:0, LiDAR:1, GNSS:2, Camera:3
tspan = 0.05; % Sensor frequency which is corresponded to standard sensor
%% Mode configurations
mode = 3; % 1:Offline, 2:Gazebo simulation, 3:Real exp.
offlinePath = "/home/student/Program/matlab_common/data/20250613/20250613_171110/userLocal.mat";
isParallel = false;
isMultiPC = false;
%% ROS2 configurations
RID = 11;
%% Module configurations
% Save file path
mySavePath = './data';
mySaveFileName = string(datetime("now","Format","yyyyMMdd_HHmmss")); 
Datadir = strcat(mySavePath,filesep,string(datetime("now","Format","yyyyMMdd")),filesep,mySaveFileName);
if ~exist(Datadir,"dir"), mkdir(Datadir); end

% You can supply your own class instead of default if you need.
% addpath(genpath("./MyEstimate"))
estimator = estimator.Estimate2(mode,offlinePath);
controller = controller.Control2();
logger = logger.DataLogger(Datadir,'tmp');

% Activation paralell worker
if isempty(gcp('nocreate')), parpool; end

cfg = struct( ...
    "modeNumber"  , mode, ...
    "isParallel"  , isParallel, ...
    "isMultiPC"   , isMultiPC, ...
    "RID"         , RID, ...
    "sensorIdx"   , sensor, ...
    "base_sensor" , base_sensor, ...
    "vehicleType" , vehicleType, ...
    "tspan"       , tspan, ...
    "offlinePath" , offlinePath, ...
    "rosNamespace", "matlab", ...
    "estimator"   , estimator, ...
    "controller"  , controller, ...
    "logger"      , logger, ...
    "manualCon"   , false);

sys = core.SystemFactory.build(cfg);
sys.run();

%% Plot
plotter.DataPlotter(Datadir,cfg)

