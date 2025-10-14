classdef OfflineMode < mode.ModeStrategy
    properties
        path
        sensorIdx
    end
    methods
        function obj = OfflineMode(cfg)
            % cfg.offlinePath, cfg.sensorIdx
            disp("Setting up Offline mode")
            obj.path = cfg.offlinePath;
            obj.sensorIdx = cfg.sensorIdx;
        end
        function setup(obj), end
        function [data,Plant] = receiveData(obj)
            data = [];
            Plant = [];
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd), end
        function shutdown(obj)
            close(findall(groot, 'Type', 'figure'));
            disp("Finished.")
        end
    end
end
