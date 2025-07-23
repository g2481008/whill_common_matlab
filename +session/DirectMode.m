classdef DirectMode < session.SessionStrategy
    properties (Access = public)
        sessionRole
        Stopper
        
    end
    
    methods
        function obj = DirectMode(role)
            obj.sessionRole = role;
            obj.Stopper = figure(101);
            set(obj.Stopper,'Name','Press any key to stop systems while this window is active', ...
                'KeyPressFcn',@session.DirectMode.checkSignal, 'UserData',true);
        end
        function start(obj)
            utils.waitPressEnterkey(); % start            
        end
        function ret = isWorking(obj)
            ret = obj.Stopper.UserData;
        end
        function stop(obj)
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