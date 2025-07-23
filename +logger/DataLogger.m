classdef DataLogger < logger.LoggerStrategy
    properties
        buffer       
        filePath    
        f
        fileName
        useParpool = false;
        idx = 1;
    end
    properties(Access=private)
        iswarn = 0;
        isReached = 0;

    end
    properties (Access=private, Constant)
        limBuf = 10000;
    end

    
    methods
        function obj = DataLogger(filePath,fname)
            obj.buffer = cell(obj.limBuf,1);
            obj.filePath = filePath;
            obj.f = parallel.FevalFuture;
            obj.fileName = fname;
        end
        
        function addData(obj, newData)
            obj.buffer{obj.idx,1} = newData;
            if obj.idx == obj.limBuf
                obj.idx = 1;
                obj.iswarn = 1;
                obj.isReached = 1;
            else
                obj.idx = obj.idx+1;
            end
            if obj.iswarn
                warning('Logger limitation has reached. Overwriting data.')
            end
        end
        
        function finish = saveData(obj)
            if isempty(obj.buffer{1,1})
                finish = 1;
                return;
            end
            if obj.isReached
                startIdx = obj.idx-1;
                if startIdx == 0
                    startIdx = obj.limBuf;
                end
            else
                startIdx = 1;
            end
            newIdx = mod((startIdx-1):(startIdx-1)+obj.limBuf-1, obj.limBuf) + 1;
            newBuf = obj.buffer(newIdx,1);
            numData = sum(cellfun(@(x) isstruct(x) && ~isempty(x), newBuf));
            
            % 保存処理を実行            
            matObj = matfile(strcat(obj.filePath,filesep,"userLocal_",obj.fileName,".mat"), 'Writable', true);
            matObj.(obj.fileName) = newBuf(1:numData,1); % ファイルに保存
            obj.buffer = {};
            finish = 1;
            
        end
        
        function stop(obj)
            if obj.useParpool
                obj.f = parfeval(@obj.saveData,1);
            else
                [~] = obj.saveData();
            end
        end

        function ok = isDone(obj)
            if obj.useParpool
                ok = fetchOutputs(obj.f);
            else
                ok = 1;
            end
        end
    end
end
