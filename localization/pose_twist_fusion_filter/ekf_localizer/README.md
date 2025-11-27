# ekf_localizer

## 概要 (Overview)
`ekf_localizer` は 2D 走行モデルと新鮮な Pose/Twist 入力を拡張カルマンフィルタで融合し，自己位置姿勢と速度を滑らかに推定する ROS 2 コンポーネントです。Pose と Twist が異なる遅延やレートで届く自動運転ロボットを想定し，遅延補償，外れ値除去，およびヨー・バイアス推定を内包しています。

### 特長 (Key Features)
- おのおのの入力 Stamp を用いた遅延補償と過去状態参照により，センサ遅延や通信ばらつき下でも一貫した統合が可能
- Pose/Twist それぞれに Mahalanobis ゲートと NaN/Inf フィルタを備え，異常測定を抑制
- `enable_yaw_bias_estimation` により yaw バイアスをオンライン推定し，バイアスあり/なしの Pose を同時出力
- `use_pose_with_covariance` / `use_twist_with_covariance` でメッセージ付属共分散または推定パラメータを選択
- `tf_rate` で制御可能な `map -> base_link` TF と `~/debug` 系トピックで運用時の可視化を支援

## ノード (Nodes)
### ekf_localizer::EKFLocalizerComponent
- **型**: `rclcpp_components::NodeFactory` からロードできるコンポーネント。`ekf_localizer` 実行ファイルで単体ノードとしても動作。
- **役割**: Pose/Twist 測定の遅延を補償しながら状態予測・更新を繰り返し，Pose/Twist/TF/デバッグ情報を publish。

#### 購読トピック (Subscribed Topics)
| 名称                       | 型                                             | 説明                                                                                     |
| -------------------------- | ---------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `initialpose`              | `geometry_msgs/msg/PoseWithCovarianceStamped`  | 推定状態のリセットに使用。QoS は Transient Local Reliable。                              |
| `in_pose`                  | `geometry_msgs/msg/PoseStamped`                | Pose 測定 (covariance を使わないモード)。SensorDataQoS。                                 |
| `in_pose_with_covariance`  | `geometry_msgs/msg/PoseWithCovarianceStamped`  | `use_pose_with_covariance=true` の際に利用。covariance を measurement noise として流用。 |
| `in_twist`                 | `geometry_msgs/msg/TwistStamped`               | Twist 測定 (covariance を使わないモード)。                                               |
| `in_twist_with_covariance` | `geometry_msgs/msg/TwistWithCovarianceStamped` | `use_twist_with_covariance=true` の際に利用。                                            |

#### 公開トピック (Published Topics)
| 名称                                       | 型                                             | 説明                                                                 |
| ------------------------------------------ | ---------------------------------------------- | -------------------------------------------------------------------- |
| `ekf_pose`                                 | `geometry_msgs/msg/PoseStamped`                | 推定 Pose。`pose_frame_id` を `frame_id` に設定。                    |
| `ekf_pose_with_covariance`                 | `geometry_msgs/msg/PoseWithCovarianceStamped`  | Pose + 共分散。`ekf_pose` と同じ orientation (yaw bias 含む)。       |
| `ekf_pose_without_yawbias`                 | `geometry_msgs/msg/PoseStamped`                | バイアスを除いた Pose。F-004 の検証対象。                            |
| `ekf_pose_with_covariance_without_yawbias` | `geometry_msgs/msg/PoseWithCovarianceStamped`  | 上記 Pose の covariance 付きバージョン。                             |
| `ekf_twist`                                | `geometry_msgs/msg/TwistStamped`               | 推定 Twist (`base_link` 基準)。                                      |
| `ekf_twist_with_covariance`                | `geometry_msgs/msg/TwistWithCovarianceStamped` | Twist + 共分散。                                                     |
| `~/estimated_yaw_bias`                     | `std_msgs/msg/Float64`                         | ヨー・バイアス推定値。F-004。                                        |
| `~/debug`                                  | `std_msgs/msg/Float64MultiArray`               | `[estimated yaw [deg], measured yaw [deg], yaw bias [deg]]` 를格納。 |
| `~/debug/measured_pose`                    | `geometry_msgs/msg/PoseStamped`                | 最新 Pose 測定をノード時間で再配信し，遅延補償結果を可視化。         |

#### 公開 TF (Published TF)
- `map` (既定 `pose_frame_id`) → `base_link` の `geometry_msgs/msg/TransformStamped` を `tf_rate` で配信。

### 主なパラメータ (Key Parameters)
パラメータは `config/ekf_localizer.param.yaml` で管理し，Launch から差し替え可能です。

| カテゴリ       | パラメータ                   | 既定値  | 説明                                                                             |
| -------------- | ---------------------------- | ------- | -------------------------------------------------------------------------------- |
| 一般           | `predict_frequency`          | 50.0    | EKF 予測・publish 周波数 [Hz]。                                                  |
|                | `tf_rate`                    | 10.0    | TF 送信周期 [Hz]。                                                               |
|                | `extend_state_step`          | 50      | 遅延補償の最大ステップ数。`predict_frequency` と合わせて補償可能時間を決定。     |
|                | `pose_frame_id`              | `map`   | 推定 Pose の出力フレーム。IT_EKFL_002 で上書き挙動をテスト。                     |
| Yaw bias       | `enable_yaw_bias_estimation` | true    | バイアス推定と関連出力の有効化。                                                 |
| Pose 入力      | `pose_additional_delay`      | 0.0     | Pose Stamp 誤差を補正する追加遅延 [s]。                                          |
|                | `pose_gate_dist`             | 10000.0 | Pose Mahalanobis 距離しきい値。F-002 の要素。                                    |
|                | `use_pose_with_covariance`   | false   | PoseWithCovariance を measurement noise として使用。                             |
| Twist 入力     | `twist_additional_delay`     | 0.0     | Twist 用追加遅延 [s]。                                                           |
|                | `twist_gate_dist`            | 10000.0 | Twist Mahalanobis しきい値。                                                     |
|                | `use_twist_with_covariance`  | false   | TwistWithCovariance を利用するか。F-003 で切替テスト。                           |
| プロセスノイズ | `proc_stddev_vx_c`           | 5.0     | 直進加速度ノイズ。                                                               |
|                | `proc_stddev_wz_c`           | 1.0     | 角加速度ノイズ。                                                                 |
|                | `proc_stddev_yaw_c`          | 0.005   | ヨーと角速度の相関ノイズ。                                                       |
|                | `proc_stddev_yaw_bias_c`     | 0.001   | ヨー・バイアス変化ノイズ。`enable_yaw_bias_estimation=false` の場合は 0 に固定。 |

詳細は `config/ekf_localizer.param.yaml` を参照してください。

### 機能とテスト対応 (Feature-to-Test Traceability)
| 機能 ID | 機能説明                                                                                      | テスト ID   | テストで検証する観点                                                                            |
| ------- | --------------------------------------------------------------------------------------------- | ----------- | ----------------------------------------------------------------------------------------------- |
| F-001   | Pose Stamp を遅延補償しつつ EKF の過去状態へ正しく適用する。                                  | UT_EKFL_001 | 過去時刻で publish された Pose が `pose_frame_id` に変換されて出力へ反映される。                |
| F-002   | Pose Mahalanobis ゲートで外れ値を拒否し、状態を保護する。                                     | UT_EKFL_002 | ゲート距離を超える測定後も推定結果が不連続にならず WARN を残す。                                |
| F-003   | Twist 入力経路を `use_twist_with_covariance` に合わせて一意に選択する。                       | UT_EKFL_003 | パラメータ切替で `in_twist`/`in_twist_with_covariance` どちらが推定に使われるかが即座に変わる。 |
| F-004   | Yaw バイアス推定を有効化すると Bias 付き/なし Pose と `~/estimated_yaw_bias` を同時提供する。 | UT_EKFL_004 | バイアス推定 ON/OFF で Pose orientation の差分とバイアス値が一致/消失する。                     |
| F-005   | 既定 launch で Pose/Twist/TF/デバッグ出力を安定周波数で publish する。                        | IT_EKFL_001 | 50 Hz 付近で全出力と `map -> base_link` TF が継続配信される。                                   |
| F-006   | Launch 引数のトピック remap と `pose_frame_id` 上書きが即時反映される。                       | IT_EKFL_002 | 入出力・TF の frame とトピック名が指定値へ切り替わる。                                          |

## 起動ファイル (Launch Files)
### `launch/ekf_localizer.launch.py`
| 引数                                                | 既定値                                     | 説明                            |
| --------------------------------------------------- | ------------------------------------------ | ------------------------------- |
| `param_file`                                        | `config/ekf_localizer.param.yaml`          | YAML で渡すパラメータセット。   |
| `input_initial_pose_topic`                          | `initialpose`                              | 初期化 Pose トピック。          |
| `input_pose_topic`                                  | `in_pose`                                  | Pose 測定 (非共分散) トピック。 |
| `input_pose_with_covariance_topic`                  | `in_pose_with_covariance`                  | PoseWithCovariance 入力。       |
| `input_twist_topic`                                 | `in_twist`                                 | Twist 測定。                    |
| `input_twist_with_covariance_topic`                 | `in_twist_with_covariance`                 | TwistWithCovariance 入力。      |
| `output_pose_topic`                                 | `ekf_pose`                                 | Fused Pose 出力。               |
| `output_pose_with_covariance_topic`                 | `ekf_pose_with_covariance`                 | Pose+cov。                      |
| `output_pose_without_yawbias_topic`                 | `ekf_pose_without_yawbias`                 | Bias 除去 Pose。                |
| `output_pose_with_covariance_without_yawbias_topic` | `ekf_pose_with_covariance_without_yawbias` | Bias 除去 Pose+cov。            |
| `output_twist_topic`                                | `ekf_twist`                                | Twist 出力。                    |
| `output_twist_with_covariance_topic`                | `ekf_twist_with_covariance`                | Twist+cov。                     |

例: `ros2 launch ekf_localizer ekf_localizer.launch.py input_pose_topic:=/localization/pose`。

## 設定 (Configuration)
- 既定パラメータは `config/ekf_localizer.param.yaml` に格納されています。
- `pose_frame_id`・QoS・ゲート閾値などは launch 時に `param_file` を差し替えることで一括変更できます。
- 実システムに合わせて `pose_additional_delay` / `twist_additional_delay` を調整し、センサヘッダ時間のずれを補正してください。

## 使用方法 (Usage)
1. 依存ビルド
   ```bash
   colcon build --packages-select ekf_localizer
   source install/setup.bash
   ```
2. 既定パラメータで起動
   ```bash
   ros2 launch ekf_localizer ekf_localizer.launch.py \
     input_pose_topic:=/localization/pose \
     input_twist_topic:=/localization/twist
   ```
3. コンポーネントコンテナへロードする場合は `component_container_mt` などに `ekf_localizer::EKFLocalizerComponent` を追加し，同じ引数で remap してください。

## テスト (Testing)
- 詳細シナリオと観測ポイントは `TEST_SPEC.md` を参照。
- ユニットテスト: `colcon test --packages-select ekf_localizer --ctest-args -R ekf_localizer`。
- 結合テスト (launch_testing): `colcon test --packages-select ekf_localizer --pytest-args -k ekf_launch` 等で実行できます。

## トラブルシューティング (Troubleshooting)
- Pose/Twist のヘッダ時間が系全体で単調増加しているか `ros2 topic hz`/`ros2 topic delay` で確認。負の遅延は F-001 の WARN を誘発します。
- Pose フレームが `pose_frame_id` と一致していない場合は `map` への TF が必要です。`tf2_echo pose_frame_id pose.header.frame_id` で整合性を確認してください。
- 推定が発散する場合は `pose_gate_dist` / `twist_gate_dist` を縮めて異常値を切り捨て、`proc_stddev_*` を調整してモデル追従性を上げてください。

