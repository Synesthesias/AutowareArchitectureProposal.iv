# ndt_scan_matcher

## 概要 (Overview)
`ndt_scan_matcher` は NDT (Normal Distributions Transform) を用いて LiDAR 点群を地図に整合させ，`map` 座標系での自車姿勢を推定する ROS 2 コンポーネントです。高解像度地図と生点群の間で安定した初期姿勢を確保するため，EKF 出力を時間補間して初期推定とし，収束判定に基づいて Pose/TF を配信します。

### 特長 (Key Features)
- 初期姿勢を時間順にバッファし，センサ時刻へ線形補間した姿勢を NDT へ提供
- センサ点群を `base_frame` へ変換してから NDT 入力に設定し，結果を `map` 座標へ再投影
- 変換確率と反復数を監視し，閾値未満の結果は publish せず WARN/diagnostics で通知
- Monte Carlo 法を用いた `ndt_align_srv` で初期姿勢のばらつきを吸収しつつサービス経由で整合結果を返す
- 実行時間・変換確率・ iteration 数を Float32 topic で公開し，性能を可視化

## ノード (Nodes)
### ndt_scan_matcher::NDTScanMatcherComponent
- **型**: `rclcpp_components::NodeFactory` からロードできるコンポーネント。`ndt_scan_matcher_node` でスタンドアロン起動も可能。
- **役割**: map 点群と LiDAR 生点群を NDT で整合させ，Pose/TF/診断/デバッグ情報を出力しつつサービスでオフライン整合も提供。

#### 購読トピック (Subscribed Topics)
| 名称                       | 型                                            | 説明                                                                         |
| -------------------------- | --------------------------------------------- | ---------------------------------------------------------------------------- |
| `pointcloud_map`           | `sensor_msgs/msg/PointCloud2`                 | トリガ付き `TransientLocal` map 点群。受信時に NDT の target を更新。        |
| `points_raw`               | `sensor_msgs/msg/PointCloud2`                 | LiDAR 生点群。SensorDataQoS。TF で `base_frame` へ変換後 NDT source に設定。 |
| `ekf_pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped` | EKF 推定姿勢のリングバッファ。センサ時刻で線形補間して初期推定として使用。   |

#### 公開トピック (Published Topics)
| 名称                                                    | 型                                            | 説明                                                                  |
| ------------------------------------------------------- | --------------------------------------------- | --------------------------------------------------------------------- |
| `ndt_pose`                                              | `geometry_msgs/msg/PoseStamped`               | `map` 基準の整合結果。収束条件を満たした時のみ publish。              |
| `ndt_pose_with_covariance`                              | `geometry_msgs/msg/PoseWithCovarianceStamped` | 上記 Pose + 固定共分散。                                              |
| `points_aligned`                                        | `sensor_msgs/msg/PointCloud2`                 | 整合後の点群 (`map` フレーム)。                                       |
| `initial_pose_with_covariance`                          | `geometry_msgs/msg/PoseWithCovarianceStamped` | センサ時刻へ補間した初期姿勢を可視化用途で出力。                      |
| `exe_time_ms`, `transform_probability`, `iteration_num` | `std_msgs/msg/Float32`                        | NDT 実行時間／変換確率／最終反復数。                                  |
| `initial_to_result_distance(_old/_new)`                 | `std_msgs/msg/Float32`                        | 初期姿勢と結果との差分長。                                            |
| `ndt_marker`, `monte_carlo_initial_pose_marker`         | `visualization_msgs/msg/MarkerArray`          | 反復した Pose 系列と Monte Carlo 粒子。                               |
| `/diagnostics`                                          | `diagnostic_msgs/msg/DiagnosticArray`         | `state`, `skipping_publish_num`, `transform_probability` などを通知。 |

#### サービス (Services)
| 名称                       | 型                                                         | 説明                                                            |
| -------------------------- | ---------------------------------------------------------- | --------------------------------------------------------------- |
| `ndt_align_srv`            | `autoware_localization_srvs/srv/PoseWithCovarianceStamped` | 共分散付き初期姿勢を受け取り Monte Carlo 探索後の Pose を返却。 |
| `ndt_align_pose_array_srv` | `autoware_localization_srvs/srv/PoseArray`                 | 事前生成した候補姿勢リストの中からベスト整合を返却。            |

#### 公開 TF (Published TF)
- `map -> ndt_base_link` (`ndt_base_frame` パラメータで変更可) を `points_raw` 受信タイミングで送信。

### 主なパラメータ (Key Parameters)
| カテゴリ | パラメータ                              | 既定値          | 説明                                      |
| -------- | --------------------------------------- | --------------- | ----------------------------------------- |
| フレーム | `base_frame`                            | `base_link`     | LiDAR を投影する車体基準フレーム。        |
|          | `ndt_base_frame`                        | `ndt_base_link` | TF で使用する子フレーム。                 |
|          | `map_frame`                             | `map`           | 推定結果の親フレーム。                    |
| 入力制御 | `input_sensor_points_queue_size`        | 1               | 初期姿勢 deque の最大保持数。             |
| NDT      | `ndt_implement_type`                    | 2 (OMP)         | 0:PCL GENERIC,1:PCL MODIFIED,2:OMP。      |
|          | `trans_epsilon`                         | 0.01            | NDT の収束許容誤差。                      |
|          | `step_size`                             | 0.1             | Newton line search の最大ステップ。       |
|          | `resolution`                            | 2.0             | NDT ボクセル解像度。                      |
|          | `max_iterations`                        | 30              | 反復回数上限。                            |
| 収束判定 | `converged_param_transform_probability` | 3.0             | 変換確率がこの値未満なら publish を抑制。 |
| OMP      | `omp_neighborhood_search_method`        | 0               | 近傍探索手法 (KDTREE/DIRECT*)。           |
|          | `omp_num_threads`                       | 4               | OMP 実行スレッド数。                      |

詳細は `launch/ndt_scan_matcher.launch.py` で上書きできます。

### 機能とテスト対応 (Feature-to-Test Traceability)
| 機能 ID | 機能説明                                                                               | テスト ID    | テストで検証する観点                                                               |
| ------- | -------------------------------------------------------------------------------------- | ------------ | ---------------------------------------------------------------------------------- |
| F-001   | センサ時刻に最も近い2件の初期姿勢から補間姿勢を生成し NDT へ渡す。                     | UT_NDTSM_001 | `getNearestTimeStampPose` と `interpolatePose` が目標時刻で線形補間する。          |
| F-002   | センサ時刻より古い初期姿勢を deque から除去し、最新姿勢だけを保持する。                | UT_NDTSM_002 | `popOldPose` が `time_stamp` 以上の要素だけに削減する。                            |
| F-003   | RPY 差分を +/-pi 範囲に正規化し、滑らかな角速度を算出する。                            | UT_NDTSM_003 | `calcTwist` がヨー跨ぎでも連続した角速度を返す。                                   |
| F-004   | map / 初期姿勢 / センサの3条件が揃うまで NDT 出力と TF を publish しない。             | IT_NDTSM_001 | 未準備の入力では WARN のみ、全て揃った直後に Pose/TF が現れる。                    |
| F-005   | 変換確率と反復数が閾値を満たさない場合は publish をスキップし diagnostics へ反映する。 | IT_NDTSM_002 | `skipping_publish_num` が増加し WARN が出る一方、閾値緩和後は Pose/TF が再開する。 |
| F-006   | `ndt_align_srv` が共分散に基づく Monte Carlo 粒子から最高スコア姿勢を返す。            | IT_NDTSM_003 | サービス応答が ground truth に収束し、Marker へ粒子情報が表示される。              |

## 起動ファイル (Launch Files)
### `launch/ndt_scan_matcher.launch.py`
Composable Node Container 用 launch。主な引数:
| 引数                                                                                         | 既定値                       | 説明                             |
| -------------------------------------------------------------------------------------------- | ---------------------------- | -------------------------------- |
| `container_name`                                                                             | `ndt_scan_matcher_container` | 使用するコンポーネントコンテナ。 |
| `node_name`                                                                                  | `ndt_scan_matcher`           | ノード名。                       |
| `input_sensor_points_topic`                                                                  | `/points_raw`                | LiDAR 入力トピック。             |
| `input_initial_pose_topic`                                                                   | `/ekf_pose_with_covariance`  | 初期姿勢トピック。               |
| `input_map_points_topic`                                                                     | `/pointcloud_map`            | 地図点群トピック。               |
| `output_pose_topic`                                                                          | `ndt_pose`                   | 推定 Pose。                      |
| `output_pose_with_covariance_topic`                                                          | `ndt_pose_with_covariance`   | 共分散付き Pose。                |
| `output_diagnostics_topic`                                                                   | `/diagnostics`               | 診断出力。                       |
| `base_frame`, `ndt_implement_type`, `trans_epsilon`, `resolution`, `max_iterations`, `omp_*` | (上記参照)                   | 動作パラメータ群。               |

## 設定 (Configuration)
- コンポーネント固有の YAML は不要ですが、launch 引数でパラメータ辞書を渡せます。
- フレーム構成 (`base_frame`, `map_frame`) は車両 TF ツリーに合わせて調整してください。
- 大規模点群では `resolution` を大きくすると収束が安定し、`omp_num_threads` で CPU 資源を活用できます。

## 使用方法 (Usage)
1. 依存ビルド
   ```bash
   colcon build --packages-select ndt_scan_matcher
   source install/setup.bash
   ```
2. 既定設定で起動
   ```bash
   ros2 launch ndt_scan_matcher ndt_scan_matcher.launch.py \
     input_sensor_points_topic:=/sensing/lidar/points_raw \
     input_initial_pose_topic:=/localization/pose_estimator/ekf/pose_with_covariance \
     input_map_points_topic:=/map/pointcloud_map
   ```
3. サービスで外部整合を取得
   ```bash
   ros2 service call /ndt_align_srv autoware_localization_srvs/srv/PoseWithCovarianceStamped "{...}"
   ```

## テスト (Testing)
- 仕様とケースの詳細は `TEST_SPEC.md` を参照。
- ユニットテスト: `colcon test --packages-select ndt_scan_matcher --ctest-args -R ndt_scan_matcher_ut` などで gtest を実行。
- 結合テスト: `colcon test --packages-select ndt_scan_matcher --pytest-args -k ndt_scan_matcher_it` で launch_testing を実行。

## トラブルシューティング (Troubleshooting)
- `No map pointcloud set.` や `No initial pose available.` WARN が続く場合は入力トピックの QoS/フレームを確認し、`tf2_echo map base_link` で TF が届いているかチェックしてください。
- `NDT did not converge.` が頻発する場合は `resolution` を大きくするか、`converged_param_transform_probability` を小さく調整して観測ノイズを吸収します。
- `/diagnostics` の `skipping_publish_num` が増え続ける場合は LiDAR 点群と地図の座標差分や `base_frame` の TF 遅延を確認してください。
