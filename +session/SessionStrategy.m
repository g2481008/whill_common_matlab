classdef (Abstract) SessionStrategy
    methods (Abstract)
        start(obj)
        ret = isWorking(obj)
        stop(obj)
    end
end