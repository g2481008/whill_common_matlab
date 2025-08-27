# MATLAB共通プログラム (ACSL 電動車椅子班)
電動車椅子をMATLABで制御するテンプレート
ダウンロードするか，ブランチを切って使用・編集すること

## 対応機種(2025/08/15現在)
* Whill Model CR

## 動作確認済みの環境
* Ubuntu22.04
* ROS2 Humble
* MATLAB 2024b

## 使用可能センサ
* Velodyne LiDAR VLP-16
* Ublox-GNSS (RTK可)
* Realsense D435-i(/D455)

## フォルダ構造について
| 名称          | 説明                                | 編集権限  | 抽象クラス有無   |
|:--------------|:------------------------------------|:--------:|:--------------:|
| `+app`          | 非同期処理用セッションApp           | ARC      | ×              |
| `+bridge`       | ROS2,共有メモリ等の通信クラス       | ARC      | ×              |
| `+controller`   | 制御計算クラス                     | **USR**  | ×              |
| `+core`         | 逐次処理(デバッグ)用セッションApp   | ARC      | ×              |
| `+estimator`    | 推定計算クラス                     | **USR**  | ×              |
| `+logger`       | 計算結果記録クラス                 | **USR**  | ○              |
| `+mode`         | 機種・モード別モジュール            | BLD      | ○              |
| `+plotter`      | 結果描画クラス                     | **USR**  | ○              |
| `+session`      | セッション制御                     | ARC      | ○              |
| `+utils`        | その他ツール                       | ARC      | ×              |
| `main.m`        | 逐次処理用main文                   | **USR**  | -              |
| `main_split.m`  | 非同期処理用main文                 | **USR**  | -              |

#### 追加/編集権限
* **USR**: エンドユーザー(システムの利用者．ARC,BLD含)
* ARC: システム根幹の構築者
* BLD: 新機能・モジュールの追加実装者(ARC含)

#### 抽象クラス有無
* 自作クラスの導入時，抽象クラスが存在するパッケージなら**必ず**継承してサブクラスにする．
* 抽象クラスが定義する入出力数は遵守する．

## パッケージ群の呼び出し
"+"が付くフォルダはパッケージ群．中のファイルをスクリプト内で呼び出す際は，
**"フォルダ名.ファイル名**または**import フォルダ名**とし，ファイル名のみで呼び出す

例）
```
estimator.EstimateEKF()
```
または
```
import estimator.*
EstimateEKF()
```

## Estimator, Controllerクラス実装方法
クラスはパッケージフォルダ(`+estimator`,`+controller`)に格納し，従属する関数ファイルは新規作成したフォルダで管理する．
```
例）matlab_common---`+estimator`  ---Estimate2.m (テンプレート)
                  |                |-EstimateJPDAF.m (JPDA Tracker)
                  |                |-EstimateEKF.m (EKF Tracker)
                  |
                  |-`+controller` ---Control2.m (テンプレート)
                  |                |-ControlPID.m (PID Controller)
                  |                |-ControlMPC.m (MPC Controller)
                  |
                  |-`TrackerJPDAF`---A.m
                  |                |-B.m
                  |                |-...
                  |-`ControllerPID`---C.m
                  |                |-D.m
                  |                |-...
                  :
```

### 呼び出し:
* `main.m`: `estimator = estimator.EstimateJPDAF();`
* `main_split.m`: `addpath(genpath("./TrackerJPDAF")); estimator = estimator.EstimateJPDAF();`
* `main_split.m`では**addpath**を忘れずに．

## Logger, Plotterについて
* ユーザーによって保存したいデータ，描画したいグラフが異なるため，**各々でクラスを実装することを推奨**
* デフォルトで用意したクラスがあるのでそれらを使っても良い
#### Loggerの実装

#### Plotterの実装

## 使用方法 (デバッグ編)
#### 推定器，制御器を作成し終えてこれからデバッグする人
#### 計算時間が短く(0.1s以内目安)逐次処理で問題ない人
1. `main.m`を使用
2. 用法に従って実行


## 使用方法 (本番編)
#### デバッグが終了⇒検証する人
#### 計算時間が長く，効率化が必要な人
1. Matlabセッションを3つ起動
2. `main_split.m`を使用
3. 上部をセクション実行
4. 下部のセクションEstimator, Controller Appを実行後，**最後に**Node Appを実行

## 実装者(BLD)向け開発ポリシー
#### 新機種や新モードを実装する際に気を付けること
* 元のクラスは削除や編集しない
* 自身が作成したクラスを新たに追加するだけで目的の機種や新モードが機能するように設計する
* 実装が終了したらPull Requestする：
  * 実装目的
  * 実装内容(何ができるか詳細に書く)
  * 配置予定のフォルダ階層
  * 抽象クラスの有無，有りなら継承しているか
  * 動作確認して問題無かったか

