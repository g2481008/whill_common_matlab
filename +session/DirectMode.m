classdef DirectMode < session.SessionStrategy
    properties (Access = public)
        sessionRole
        Stopper
        numStep
        
    end
    
    methods
        function obj = DirectMode(cfg,role)
            obj.sessionRole = role;
            obj.Stopper = session.createKeyCon();
            if cfg.modeNumber == 1
                if exist(cfg.offlinePath,"file")
                    S = load(cfg.offlinePath);
                    uddname = fieldnames(S);
                    udd = S.(uddname{1});
                    obj.numStep = size(udd,1);
                else
                    warning("Matfile not found. Using ""tend"" instead.")
                    obj.numStep = cfg.tend/cfg.tspan;
                end
            end
        end
        function start(obj)
            utils.waitPressEnterkey(obj.Stopper); % start            
        end
        function ret = isWorking(obj,varargin)
            % ret = obj.Stopper.UserData;
            if ~isempty(obj.numStep)
                ret = obj.numStep >= varargin{1};
            end
            if ~obj.Stopper.UserData
                ret = 0;
            end
        end
        function stop(obj)
            if isvalid(obj.Stopper)
                delete(obj.Stopper)
            end
        end
    end
end