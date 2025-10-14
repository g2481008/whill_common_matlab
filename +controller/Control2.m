classdef Control2 < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 計算結果を推定器に送信
    %　Estimatorにフィードバックしたい変数を"sendEst"に格納
    %% 制御結果の保存
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        
    end

    properties
        
    end

    methods
        function obj = Control2()
            
        end

        function [result,sendEst] = main(obj,Est,T)
            
            
            result.V = [1;0];
            
            sendEst = [];
            
        end
    end
end