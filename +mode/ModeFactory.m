classdef ModeFactory
    methods (Static)
        function modeObj = build(cfg)
            switch cfg.modeNumber
                case 1
                    modeObj = mode.OfflineMode(cfg.offlinePath, cfg.sensorIdx);
                case 2
                    modeObj = mode.GazeboMode(cfg.vehicleType, cfg.sensorIdx, cfg.RID, cfg.rosNamespace, cfg.base_sensor);
                case 3
                    modeObj = mode.RealMode(cfg.vehicleType, cfg.sensorIdx, cfg.RID, cfg.rosNamespace, cfg.base_sensor,cfg.manualCon);
                otherwise
                    error("unknown modeNumber");
            end
        end
    end
end
