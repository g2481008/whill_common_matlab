classdef (Abstract) LoggerStrategy < handle
    methods (Abstract)
        dstruct = addData(result);
        finish = saveData(result);
    end
end