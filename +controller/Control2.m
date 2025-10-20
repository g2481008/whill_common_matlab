classdef Control2 < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 制御結果の保存
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        
    end

    properties
        
    end

    methods
        function obj = Control2()
            
        end

        function [result] = main(obj,Est,T)
            
            
            result.V = [0;0];
            
        end
    end
end