classdef WheelchairSystem < handle
    properties (Access = private)
        Est
        Ctrl
        DLog
        DLogC
        StepTimer   
        Mode        
        Bridge      
        SMgr
        Timer
        cnt = 0;
    end

    methods
        function obj = WheelchairSystem(cfg, modeObj, SMgr)
            arguments
                cfg struct
                modeObj {mustBeA(modeObj, "mode.ModeStrategy")}
                SMgr {mustBeA(SMgr, "session.SessionStrategy")}
            end
            obj.Est   = cfg.estimator;
            obj.Ctrl  = cfg.controller;
            obj.DLog  = cfg.logger;
            obj.Timer = utils.TimeTracker(cfg.tspan);
            obj.Mode  = modeObj;
            obj.SMgr  = SMgr;
        end

        function run(obj)
            c = onCleanup(@()obj.Mode.shutdown());
            obj.Mode.setup();
            obj.SMgr.start();
            obj.Timer.start();
            while obj.SMgr.Stopper.UserData
                obj.cnt = obj.cnt + 1;
                % Main process
                [sens,Plant] = obj.Mode.receiveData();
                [resEst,sendData]  = obj.Est.main(sens, Plant,obj.Timer.elapsed());
                resCtrl  = obj.Ctrl.main(sendData,obj.Timer.elapsed());
                cmd = struct('V',resCtrl.V,'sequence',obj.cnt);
                obj.Mode.sendData(cmd);

                % Post process
                resEst.send = sendData;
                resEst.T = posixtime(datetime('now'))*1000;
                resCtrl.T = posixtime(datetime('now'))*1000;
                resEst.sequence = obj.cnt;
                resCtrl.sequence = obj.cnt;
                result = struct('Estimator',resEst, 'Controller',resCtrl);
                obj.DLog.addData(result);
                drawnow
                obj.Timer.wait();
            end
            obj.DLog.stop();
        end
    end
end
