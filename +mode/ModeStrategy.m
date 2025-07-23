classdef (Abstract) ModeStrategy < handle
    methods (Abstract)
        setup(obj)
        [data1,data2] = receiveData(obj)
        exeProcess(obj, ReceivedData)
        sendData(obj, ProcessedData)
        shutdown(obj)
    end
end
