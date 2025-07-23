classdef SHMMode < session.SessionStrategy
    properties (Access = public)
        SMgrSHM
        sessionRole
        Stopper
        
    end
    properties (Access = private)
        vars = {'working','uint8',[1 1]};
        checkBin = 0;

    end
    methods
        function obj = SHMMode(role)
            obj.sessionRole = role;
            fname = strcat(tempdir,"matlab_smgr.bin");
            switch role
                case 'initiator'
                    obj.Stopper = figure(101);
                    set(obj.Stopper,'Name','Press any key to stop systems while this window is active', ...
                        'KeyPressFcn',@session.SHMMode.checkSignal, 'UserData',true);
                    obj.SMgrSHM = bridge.SharedMem(fname,obj.checkBin,obj.vars);                    
                case 'participant'
                    pause(3)
                    obj.SMgrSHM = bridge.SharedMem(fname,obj.checkBin);
                    obj.SMgrSHM.timeout = 0.001;
            end
        end
        function start(obj)
            switch obj.sessionRole
                case 'initiator'
                    utils.waitPressEnterkey(); % start
                    d.working = 1;
                    obj.SMgrSHM.write(d);
                case 'participant'
                    while 1
                        [data,ok] = obj.SMgrSHM.read();
                        if ok && data.working, break; end
                    end
            end
        end
        function ret = isWorking(obj) % Only for participant
            [data,~] = obj.SMgrSHM.read();
            if ~isempty(data)
                ret = data.working;
            else
                ret = 1;
            end
        end
        function stop(obj) % Only for initiator
            d.working = 0;
            obj.SMgrSHM.write(d);
            pause(1)
            if isvalid(obj.Stopper)
                close(obj.Stopper)
            end
        end
    end
    methods (Static, Access = private)
        function checkSignal(src, ~)
            disp('Stop system');
            src.UserData = false;
        end       



    end    

end