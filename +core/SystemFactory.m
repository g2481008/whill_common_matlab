classdef SystemFactory
    methods (Static)
        function sys = build(cfg)
            import mode.*;
            import bridge.*;
            import session.*;

            modeObj = mode.ModeFactory.build(cfg);
            SMgrObj = session.SessionManager.build(cfg,session.SessionManager.initiator);

            % --- WheelchairSystem 本体 ---
            sys = core.WheelchairSystem(cfg, modeObj, SMgrObj);
        end
    end
end
