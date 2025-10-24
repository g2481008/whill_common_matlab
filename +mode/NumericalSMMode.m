classdef NumericalSMMode < mode.ModeStrategy
    properties
        plantmodel
    end
    methods
        function obj = NumericalSMMode(cfg)
            disp("Setting up numerical simulation mode")
            obj.plantmodel = cfg.plant;
        end
        function setup(obj), end
        function [data,Plant] = receiveData(obj)
            data = [];
            Uc.V = [0;0];
            [~,Plant] = obj.plantmodel.main(Uc);
        end
        function exeProcess(~,~), end
        function sendData(obj, cmd)
            obj.plantmodel = obj.plantmodel.main(cmd);
        end
        function shutdown(obj)
            close(findall(groot, 'Type', 'figure'));
            disp("Finished.")
        end
    end
end
