classdef ROS2Mode < session.SessionStrategy
    properties (Access = public)
        Comm
        sessionRole
        Stopper
        
    end
    properties (Access = private)
        % vars = {'working','uint8',[1 1]};

    end
    methods
        function obj = ROS2Mode(role)
            obj.sessionRole = role;
            switch role
                case 'initiator'
                    obj.Stopper = figure(101);
                    set(obj.Stopper,'Name','Press any key to stop systems while this window is active', ...
                        'KeyPressFcn',@session.DirectMode.checkSignal, 'UserData',true);
                case 'participant'
            end
        end
        function start(obj)
            switch obj.sessionRole
                case 'initiator'
                    utils.waitPressEnterkey(); % start
                case 'participant'
            end
            
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