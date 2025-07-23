classdef SessionManager
    enumeration
        initiator
        participant
    end
    methods (Static)
        function sessionObj = build(cfg, role)
            arguments
                cfg 
                role (1,1) session.SessionManager
            end
            switch cfg.isMultiPC
                case true
                    % bridge = 'ROS2';
                    sessionObj = session.ROS2Mode(role);
                case false
                    switch cfg.isParallel
                        case false
                            % bridge = 'NONE';
                            sessionObj = session.DirectMode(role);
                        case true
                            % bridge = 'SHM'; 
                            sessionObj = session.SHMMode(role);
                        otherwise
                            error('Invalid value for cfg.isParallel');
                    end
                otherwise
                    error('Invalid value for cfg.isMultiPC');
            end
        end
    end

    
end
