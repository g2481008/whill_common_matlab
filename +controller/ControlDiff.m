classdef ControlDiff < handle
    %% Controller
    %% Method
    % Control2: 初回のみ呼び出されるMethod．このクラスで用いる変数の初期定義を行う．
    % Main: 毎時刻呼び出されるMethod． 実行する推定プログラムを作る．
    %% 制御結果の保存
    % "result"を構造体とし，保存したい値を格納．
    properties(Constant)
        l = 0.3;
        eta = 0.03;% learning rate
    end

    properties
        To
        TT
        Ve
        ue
        nu
        H
        stateNum
        inputNum
        delta
        state
        Q
        R
        Qe
        wp
        poly
        AeqB
        beqB
        C_t
        options
        pos_1_des
        pos_2_des
        pos_3_des
        Vx
        Vy
        dT
        Aeq
        beq
        F_T
        ref
        count
        xpr
        ypr
        yawpr
        V
        dc_t
        dL
    end

    methods
        function obj = ControlDiff()
            % obj.nu = [1.5;0.5];
            obj.nu = [1.0;0.0]; % 目標入力
            obj.H = 20; % ホライゾン数
            obj.Ve = repmat(obj.nu,1,obj.H);
            obj.stateNum = 2; % 状態数
            obj.inputNum = 2; % 入力数
            obj.delta = 0.1; % ホライゾンのステップ間隔
            obj.state = zeros(obj.stateNum, obj.H);
            obj.Q = [10 0;0 10]; % ステージコストの重み
            obj.R = [3 0; 0 3]; % 入力コストの重み
            obj.Qe = [30 0;0 30]; % 終端コストの重み
            obj.wp = [0.0 0.0;
                      5.0 0.0;
                      5.1 1.0;
                      10.0 1.0;
                      10.1 -1.0;
                      15.0 -1.0];% waypoints
            dt = obj.delta;

            % KKT行列の作成
            obj.Aeq = [1 0 dt 0 -1 0 0 0
                   0 1 0 dt 0 -1 0 0
                   0 0 0 0 0 0 0 0
                   0 0 0 0 0 0 0 0];
            obj.beq = [0;0;0;0];
            obj.F_T = [1 0 dt 0;
                       0 1 0 dt];
            obj.AeqB = zeros((obj.stateNum+obj.inputNum)*(obj.H),(obj.stateNum+obj.inputNum)*(obj.H));
            obj.AeqB(1:(obj.stateNum+obj.inputNum),+1:(obj.stateNum+obj.inputNum)*2) = [0 0 0 0 -1 0 0 0
                                                                                        0 0 0 0 0 -1 0 0
                                                                                        0 0 0 0 0 0 0 0
                                                                                        0 0 0 0 0 0 0 0];
            x0 = [0 0 0 0]';
            obj.beqB = zeros((obj.stateNum+obj.inputNum)*(obj.H),1);
            obj.beqB(1:(obj.stateNum)) = -obj.F_T*x0;
            for k = 2:obj.H-1
                obj.AeqB((obj.stateNum+obj.inputNum)*(k-1)+1:(obj.stateNum+obj.inputNum)*k,(obj.stateNum+obj.inputNum)*(k-1)+1:(obj.stateNum+obj.inputNum)*(k-1)+(obj.stateNum+obj.inputNum)*2) = obj.Aeq;
                obj.beqB((obj.stateNum+obj.inputNum)*(k-1)+1:(obj.stateNum+obj.inputNum)*(k-1)+(obj.stateNum+obj.inputNum)) = obj.beq;
            end

            obj.C_t = repmat([obj.Q, zeros(obj.stateNum); zeros(obj.stateNum),obj.R],1,1,obj.H);
            obj.C_t(:,:,end) = [obj.Qe, zeros(obj.stateNum); zeros(obj.stateNum), obj.R];
            
            obj.options = optimoptions('quadprog','Display','none');

            % 円軌道の経路
            obj.count = 1;
            obj.TT = 0;
            obj.dc_t = zeros(4,1,obj.H);

            %%%%% loss function
            LA = [10 10 0.1 0.1];
            % L =@(y,y_pred)  sqrt(1/size(y,2).*sum((y - y_pred).^2));
            % obj.dL =@(y,y_pred) 1./(L(y,y_pred).*size(y,2)) .* 2.*-(y-y_pred);
            L =@(y,y_pred)  sqrt(1/size(y,2).*LA*(y - y_pred).^2);
            obj.dL =@(y,y_pred) 1./(L(y,y_pred).*size(y,2)) .*LA'.* 2.*-(y-y_pred);
        end

        function result = main(obj,Est,T)
            % [U,Omega,ld] = obj.controller([Est.pose(1),Est.pose(2),Est.pose(3)]); % [x y theta] 
            dt = T-obj.To;         % 時間ステップ [s]
            obj.To = T;
            T = obj.TT + 0.05;
            obj.TT = T;
            %% -------------------- メインシミュレーションループ --------------------
            % 現在の車両状態を履歴に保存
            % vehicle_state = Est.pose';          % 車両の初期状態 [x(m), y(m), yaw(rad)]'

            %%%%%% プラントモデルの状態
            x = Est.pose(1);
            y = Est.pose(2);
            yaw = Est.pose(3);
            %%%%% 予測状態を生成
            if obj.count == 1
                obj.ue = [cos(yaw), -obj.l*sin(yaw);
                          sin(yaw),  obj.l*cos(yaw)]*obj.Ve;
                obj.count = 2;
            end
            obj.state(1,1) = x + obj.l * cos(yaw) + obj.ue(1,1)*obj.delta;
            obj.state(2,1) = y + obj.l * sin(yaw) + obj.ue(2,1)*obj.delta;
            
            % obj.state(3,1) = yaw + obj.Ve(2,1) * obj.delta;
            for k = 1:obj.H-1
                obj.state(1,k+1) = obj.state(1,k) + obj.delta * obj.ue(1,k+1);
                obj.state(2,k+1) = obj.state(2,k) + obj.delta * obj.ue(2,k+1);
                % obj.state(3,k+1) = obj.state(3,k) + obj.Ve(2,1) * obj.delta;
            end
            
            %%%%%%%%%%% リファレンスの生成
            obj.ref = zeros(4,1,obj.H);
            
            %%%%% 経路リファレンスの生成 
            % uref = [cos(yaw), -obj.l*sin(yaw);
            %         sin(yaw),  obj.l*cos(yaw)]*obj.Ve;
            uref = repmat(obj.nu,1,obj.H);
            stateref(1,1) = x + obj.l * cos(yaw) + uref(1,1)*obj.delta;
            stateref(2,1) = y + obj.l * sin(yaw) + uref(2,1)*obj.delta;

            % obj.state(3,1) = yaw + obj.Ve(2,1) * obj.delta;
            for k = 1:obj.H-1
                stateref(1,k+1) = stateref(1,k) + obj.delta * uref(1,k+1);
                stateref(2,k+1) = stateref(2,k) + obj.delta * uref(2,k+1);
                % obj.state(3,k+1) = obj.state(3,k) + obj.Ve(2,1) * obj.delta;
            end

            obj.wp = [0.0 0.0;
                  5.0 0.0;
                  5.0 1.0;
                  10.0 1.0;
                  10.0 -1.0;
                  15.0 -1.0];
            obj.ref = zeros(4,1,obj.H);
            for i=1:obj.H
                min_wp = obj.wp((obj.wp(:,1) - stateref(1,i)')<0,:);
                closedmin_wp = min_wp(end,:);
                if obj.wp(end,1)>=stateref(1,i)
                    obj.ref(1:2,1,i) = [stateref(1,i);closedmin_wp(2)];
                    obj.ref(3:4,1,i) = obj.nu;
                else
                    obj.ref(1:2,1,i) = [obj.wp(end,1);closedmin_wp(2)];
                    obj.ref(3:4,1,i) = [0;0];
                end
            end

            x0 = [obj.state;obj.ue];

            %%%%% MPCの評価関数
            C = mat2cell(obj.C_t, obj.stateNum+obj.inputNum, obj.stateNum+obj.inputNum, ones(1,obj.H));   % 各スライスを cell 配列に分解
            C_k = 2.*blkdiag(C{:}); 
            cB = -2*pagemtimes(obj.ref,'transpose',obj.C_t,'none')-obj.eta.*pagetranspose(obj.dc_t);
            c_k = reshape(cB,(obj.stateNum+obj.inputNum)*obj.H,1);
            C_T = obj.C_t(:,:,end);
            c_T = -2*obj.ref(:,end)'*obj.C_t(:,:,end);
            obj.F_T = [1 0 obj.delta 0;
                       0 1 0 obj.delta];
            obj.AeqB = zeros((obj.stateNum+obj.inputNum)*(obj.H),(obj.stateNum+obj.inputNum)*(obj.H));
            obj.AeqB(1:(obj.stateNum+obj.inputNum),1:(obj.stateNum+obj.inputNum)*2) = [-1 0 0 0 0 0 0 0
                                                                                       0 -1 0 0 0 0 0 0
                                                                                       0 0 0 0 0 0 0 0
                                                                                       0 0 0 0 0 0 0 0];
            obj.beqB = zeros((obj.stateNum+obj.inputNum)*(obj.H),1);
            obj.beqB(1:(obj.stateNum)) = -obj.F_T*[x + obj.l * cos(yaw),y + obj.l * sin(yaw),obj.ue(1,1),obj.ue(2,1)]';
            for k = 1:obj.H-1
                obj.AeqB((obj.stateNum+obj.inputNum)*(k-1)+5:(obj.stateNum+obj.inputNum)*k+4,(obj.stateNum+obj.inputNum)*(k-1)+1:(obj.stateNum+obj.inputNum)*(k-1)+(obj.stateNum+obj.inputNum)*2) = obj.Aeq;
                obj.beqB((obj.stateNum+obj.inputNum)*(k-1)+5:(obj.stateNum+obj.inputNum)*(k-1)+(obj.stateNum+obj.inputNum)+4) = obj.beq;
            end
            
            [ws,fval,exitflag,output,lambda] = quadprog(C_k, c_k, [],[],obj.AeqB,obj.beqB,[],[],x0,obj.options);

            %%%%% 状態の表示
            plot(ws(1:4:end,1),ws(2:4:end,1),'Marker','o','LineStyle','-');hold on
            quiver(x + obj.l * cos(yaw),y + obj.l * sin(yaw),obj.ue(1),obj.ue(2),'Color','g','Marker','square','MarkerSize',10)
            plotref = reshape(obj.ref,4,obj.H);
            plot(plotref(1,:),plotref(2,:),'Marker','o','LineStyle','-');
            hold off;
            % xlim([-3.2 3.2]);
            % ylim([-3.2 3.2]);
            xlim([0 16]);
            ylim([-1.5 1.5]);
            
            %%%%% Differentiable MPCの更新
            KC = mat2cell([2.*obj.C_t obj.F_T'.*ones(4,2,20);obj.F_T.*ones(2,4,20) zeros(2,2,20)],...
                2*obj.stateNum+obj.inputNum, 2*obj.stateNum+obj.inputNum, ones(1,obj.H));   % 各スライスを cell 配列に分解
            C_Ck = blkdiag(KC{:});
            for k = 1:obj.H-1
                C_Ck((obj.stateNum+obj.inputNum+2)*(k-1) + (obj.stateNum+obj.inputNum+1):(obj.stateNum+obj.inputNum+2)*(k-1) + (obj.stateNum+obj.inputNum+2), (obj.stateNum+obj.inputNum+2)*(k-1)+(obj.stateNum+obj.inputNum+3):(obj.stateNum+obj.inputNum+2)*(k-1)+(obj.stateNum+obj.inputNum+6)) = -[eye(2) zeros(2)];
                C_Ck((obj.stateNum+obj.inputNum+2)*(k-1) + (obj.stateNum+obj.inputNum+3):(obj.stateNum+obj.inputNum+2)*(k-1) + (obj.stateNum+obj.inputNum+6), (obj.stateNum+obj.inputNum+2)*(k-1)+(obj.stateNum+obj.inputNum+1):(obj.stateNum+obj.inputNum+2)*(k-1)+(obj.stateNum+obj.inputNum+2)) = -[eye(2); zeros(2)];
            end 
            Yws = reshape(ws,obj.stateNum+obj.inputNum,obj.H);
            Yref = reshape(obj.ref,obj.stateNum+obj.inputNum,obj.H);
            dk = -C_Ck\reshape([obj.dL(Yref,Yws);zeros(2,obj.H)],(obj.stateNum+obj.inputNum+2)*obj.H,1);
            dtau = reshape([dk(1:6:end), dk(2:6:end), dk(3:6:end), dk(4:6:end)]',obj.stateNum+obj.inputNum,1,obj.H);
            d_lambda = reshape([dk(5:6:end), dk(6:6:end)]',2,1,obj.H);
            dL_Ck = 1/2*(pagemtimes(dtau,'none',reshape(Yws,(obj.stateNum+obj.inputNum),1,obj.H),'transpose')+pagemtimes(reshape(Yws,(obj.stateNum+obj.inputNum),1,obj.H),'none',dtau,'transpose'));
             
            obj.C_t = obj.C_t - obj.eta.*dL_Ck;
            minus_C_t = (obj.C_t<0) & eye(size(obj.C_t,1),'logical');
            if ~isempty(obj.C_t(minus_C_t))
                obj.C_t(minus_C_t) = 0;
            end
            obj.dc_t = obj.dc_t + dtau;
            
            
            obj.ue(1,:) = ws(3:4:end);
            obj.ue(2,:) = ws(4:4:end);


            obj.V = ([cos(yaw), -obj.l*sin(yaw);
                      sin(yaw),  obj.l*cos(yaw)])\obj.ue(1:2,1);
            obj.xpr = x;
            obj.ypr = y;
            obj.yawpr = yaw;
            
            result.X = x;
            result.Y = y;
            result.yaw = yaw;
            result.ue = obj.ue;
            result.ws = ws;
            result.l = obj.l;
            result.plotref = plotref;
            result.C_t = obj.C_t;
            result.c_t = cB;


            result.V = obj.V;
            
            
        end
    end
end