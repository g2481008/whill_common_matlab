classdef (Abstract) ROS2CommStrategy < handle
    %ABSTRACTROS2COMMMANAGER ROS2通信管理の抽象基底クラス
    %   各機体固有の実装は本クラスを継承して作成する
    
    properties (Constant)
        % 共通のデータ型マッピング
        MATLAB_TYPE = {'int8','uint8','int16','uint16','int32','uint32',...
                      'int64','uint64','single','double','string','char'};
        ROS2MSG_TYPE = {'std_msgs/Int8MultiArray','std_msgs/UInt8MultiArray', ...
                       'std_msgs/Int16MultiArray','std_msgs/UInt16MultiArray', ...
                       'std_msgs/Int32MultiArray','std_msgs/UInt32MultiArray', ...
                       'std_msgs/Int64MultiArray','std_msgs/UInt64MultiArray', ...
                       'std_msgs/Float32MultiArray','std_msgs/Float64MultiArray', ...
                       'std_msgs/String','std_msgs/String'};
        
        % 共通QoSプロファイル
        DEFAULT_QOS = struct("Reliability","reliable","Durability","volatile",...
                           "History","keeplast","Depth",1);
        BESTEFFORT_QOS = struct("Reliability","besteffort","Durability","volatile",...
                              "History","keeplast","Depth",1);
    end
    
    properties (Access = protected)
        node            % ROS2ノード
        vehicleType     % 機体タイプ
        mode           % 動作モード (実機/シミュレータ等)
        sensorIdx      % センサー有効フラグ
        
        % Publisher/Subscriber
        estimatorPubs  % 推定データ用Publisher
        estimatorMsgs  % 推定データ用メッセージ
        controllerSubs % 制御データ用Subscriber
        sensorSubs     % センサーデータ用Subscriber
        vehiclePubs    % 機体制御用Publisher
        vehicleSubs    % 機体状態用Subscriber
        vehicleMsg     % 機体制御メッセージ
        
        % メタデータ管理
        varpub         % 変数情報送信用
        varmsg         % 変数情報メッセージ
        
        % 制御コマンド管理
        cmdMissed = 0  % コマンド欠損カウンタ
        prevCmd        % 前回コマンド
        
        % センサー管理
        sensorFetcher  % センサーデータ取得オブジェクト
    end
    
    methods (Abstract)
        % 【機体固有】各機体で実装必須のメソッド
        % これらは機体ごとに異なるため、サブクラスで具体的に実装する
        topics = getSensorTopics(obj)           % センサートピック定義（機体固有）
        types = getSensorMsgTypes(obj)          % センサーメッセージ型定義（機体固有）
        topics = getVehicleTopics(obj)          % 機体制御トピック定義（機体固有）
        types = getVehicleMsgTypes(obj)         % 機体制御メッセージ型定義（機体固有）
        qos = getQoSProfile(obj, topicType)     % QoSプロファイル取得（機体固有）
        msg = createVehicleCommand(obj, cmd)    % 機体制御コマンド作成（機体固有）
        validateCommand(obj, cmd)               % コマンド検証（機体固有）
        data = parseSensorData(obj, sensorData) % センサーデータ解析（機体固有）
    end
    
    methods
        % 【共通機能】全ての機体で同じ処理を行うメソッド
        % サブクラスはこれらをそのまま使用できる（オーバーライドも可能）
        
        function obj = ROS2CommStrategy(vehicleType, mode, sensorIdx, nodeName, rid)
            obj.vehicleType = vehicleType;
            obj.mode = mode;
            obj.sensorIdx = sensorIdx;
            obj.node = obj.createNode(nodeName, rid);
        end
        
        function generateSensorSubscribers(obj)
            %【共通】センサーデータ用Subscriber群を生成
            % 機体固有のトピック・型情報を使って、共通の手順でSubscriber作成
            topics = obj.getSensorTopics();    % 機体固有メソッドを呼び出し
            msgTypes = obj.getSensorMsgTypes(); % 機体固有メソッドを呼び出し
            
            if length(topics) ~= length(msgTypes)
                error('AbstractROS2CommManager:TopicMismatch', ...
                      'Number of topics and message types must match');
            end
            
            n = length(topics);
            obj.sensorSubs = cell(n, 1);
            
            for i = 1:n
                if obj.sensorIdx(i) && ~isempty(topics{i})
                    qos = obj.getQoSProfile('sensor'); % 機体固有メソッドを呼び出し
                    obj.sensorSubs{i} = ros2subscriber(obj.node, ...
                        topics{i}, msgTypes{i}, ...
                        "Reliability", qos.Reliability, ...
                        "Durability", qos.Durability, ...
                        "History", qos.History, ...
                        "Depth", qos.Depth);
                else
                    obj.sensorSubs{i} = [];
                end
            end
        end
        
        function generateVehicleSubscribers(obj)
            %機体状態受信用Subscriberを生成
            topics = obj.getVehicleTopics();
            msgTypes = obj.getVehicleMsgTypes();
            qos = obj.getQoSProfile('vehicle_state');
            
            obj.vehicleSubs = ros2subscriber(obj.node, ...
                topics.state{obj.vehicleType}, ...
                msgTypes.state{obj.vehicleType}, ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability, ...
                "History", qos.History, ...
                "Depth", qos.Depth);
        end
        
        function generateVehiclePublishers(obj)
            %機体制御用Publisherを生成
            topics = obj.getVehicleTopics();
            msgTypes = obj.getVehicleMsgTypes();
            qos = obj.getQoSProfile('vehicle_command');
            
            obj.vehiclePubs = ros2publisher(obj.node, ...
                topics.command{obj.vehicleType}, ...
                msgTypes.command{obj.vehicleType}, ...
                "Reliability", qos.Reliability, ...
                "Durability", qos.Durability, ...
                "History", qos.History, ...
                "Depth", qos.Depth);
            obj.vehicleMsg = ros2message(obj.vehiclePubs);
        end
        
        function sendEstimationData(obj, counter, estData, estVarNames)
            %推定データを送信
            if isempty(estData)
                return;
            end
            
            % メッセージ準備
            for k = 1:length(obj.estimatorPubs)
                data = estData.(estVarNames{k});
                
                if ~isa(data, 'string') && ~isa(data, 'char')
                    obj.estimatorMsgs{k}.layout.dim.label = ...
                        char(join(string(size(data)), ","));
                    obj.estimatorMsgs{k}.layout.data_offset = uint32(counter);
                else
                    data = char(join([string(counter), data], ","));
                end
                
                obj.estimatorMsgs{k}.data = data;
            end
            
            % 一括送信
            for k = 1:length(obj.estimatorPubs)
                send(obj.estimatorPubs{k}, obj.estimatorMsgs{k});
            end
        end
        
        function sendVehicleCommand(obj, cmd)
            %機体制御コマンドを送信
            obj.validateCommand(cmd);
            
            % コマンド欠損チェック
            if ~isempty(obj.prevCmd) && cmd.sequence == obj.prevCmd.sequence
                obj.cmdMissed = obj.cmdMissed + 1;
            else
                obj.prevCmd = cmd;
                obj.cmdMissed = 0;
            end
            
            % セーフティチェック
            if obj.cmdMissed >= 5
                cmd = obj.createSafetyStopCommand();
                warning('AbstractROS2CommManager:CommandTimeout', ...
                        'Control command timeout. Applying safety stop.');
            end
            
            % 機体固有のコマンド作成と送信
            obj.vehicleMsg = obj.createVehicleCommand(cmd);
            send(obj.vehiclePubs, obj.vehicleMsg);
        end
        
        function stopController(obj)
            %制御器停止信号を送信
            if ~isempty(obj.varpub)
                obj.varmsg.data = 'stop';
                send(obj.varpub, obj.varmsg);
            end
        end
        
        function [sensorData, vehicleState] = getSensorData(obj)
            %センサーデータと機体状態を取得
            if ~isempty(obj.sensorFetcher)
                [sensorData, vehicleState] = obj.sensorFetcher.getSensorData(...
                    obj.sensorSubs, obj.vehicleSubs);
                
                % 機体固有の解析処理
                sensorData = obj.parseSensorData(sensorData);
            else
                sensorData = [];
                vehicleState = [];
            end
        end
    end
    
    methods (Access = protected)
        function n = createNode(obj, nodeName, rid)
            %ROS2ノードを作成（シングルトンパターン）
            persistent sNode;
            if isempty(sNode) || ~isvalid(sNode)
                fprintf('Establishing ROS2 Node: %s\n', nodeName);
                sNode = ros2node(nodeName, rid);
            end
            n = sNode;
        end
        
        function sendVariableMetadata(obj, varNames, varTypes, filename)
            %変数メタデータを送信
            varNameStr = strjoin(varNames, ',');
            varTypeStr = strjoin(varTypes, ',');
            varData = strjoin({varNameStr, varTypeStr, char(filename)}, '.');
            
            obj.varpub = ros2publisher(obj.node, "/estimation_data0", obj.ROS2MSG_TYPE{11});
            obj.varmsg = ros2message(obj.ROS2MSG_TYPE{11});
            obj.varmsg.data = varData;
            send(obj.varpub, obj.varmsg);
        end
        
        function cmd = createSafetyStopCommand(obj)
            %緊急停止コマンドを作成
            cmd = struct();
            cmd.V = [0, 0]; % 直進速度・角速度を0に
            cmd.sequence = -1; % 緊急停止フラグ
        end
    end
end