classdef SpinnerStatus < handle
    properties (Access = private)
        baseMsg      char            
        chars        char = '|/-\';  
        idx          double = 1;     
        tObj                      	  
        active       logical = true; 
        lineLen      double = 0;     
    end

    methods
        function obj = SpinnerStatus(msg, period)
            if nargin < 1, msg = '';     end
            if nargin < 2, period = 0.1; end

            obj.baseMsg = char(msg);

            % --- 初期表示（改行なし） ---
            fprintf('%s%s', obj.baseMsg, obj.chars(1)); 
            obj.lineLen = length(obj.baseMsg) + 1;

            % --- タイマーでスピナー更新 ---
            obj.tObj = timer( ...
                'ExecutionMode','fixedRate', ...
                'Period', period, ...
                'TimerFcn', @(~,~)obj.spin() );
            start(obj.tObj);
        end

        function done(obj, finalMsg)
            % スピナー停止 → 同じ行を finalMsg で置換して改行
            if nargin < 2, finalMsg = 'Done.'; end
            if ~obj.active, return; end

            stop(obj.tObj); delete(obj.tObj);

            % 行頭へ戻り、完了メッセージ＋余白消し＋改行
            fprintf('\b%s%s\n', finalMsg, ...
                blanks(max(0, obj.lineLen - length(finalMsg))));
            obj.active = false;
        end

        function delete(obj)
            obj.done();
        end
    end

   
    methods (Access = private)
        function spin(obj)
            obj.idx = mod(obj.idx, numel(obj.chars)) + 1;
            fprintf('\b%s', obj.chars(obj.idx));
        end
    end
end
