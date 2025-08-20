classdef OfflineMode < mode.ModeStrategy
    properties
        path
        sensorIdx
    end
    methods
        function obj = OfflineMode(path, idx)
            disp("Setting up Offline mode")
            obj.path = path;
            obj.sensorIdx = idx;
        end
        function setup(obj), end
        function [data,Plant] = receiveData(obj)
            data = [];
            Plant = [];
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd), end
        function shutdown(obj)
            disp("Finished.")
        end
    end
end
