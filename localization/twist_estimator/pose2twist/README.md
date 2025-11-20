# pose2twist

## 概要 (Overview)
`pose2twist` は 連続する `geometry_msgs/msg/PoseStamped` の差分から最尤（さいゆう）な速度を推定し `geometry_msgs/msg/TwistStamped` とスカラー速度トピックとして出力します。

### 特長 (Key Features)
- 位置のユークリッド距離と経過時間から並進速度（`twist.linear.x`）を算出。
- RPY 角差を `[-pi, pi]` へ正規化し、ヨー角が ±pi を跨ぐ際も角速度が連続。
- 最初のメッセージの値は0でパブリッシュし、以降の計算用にキャッシュ。
- `TwistStamped` に加え、`std_msgs/msg/Float32` の `linear_x` / `angular_z` を同時に publish して利用先ごとの購読を容易にする。
- コンポーネントコンテナへのロードと単体ノード実行の両方をサポート。

## ノード (Nodes)
### pose2twist::Pose2TwistComponent
- **型**: `rclcpp_components::NodeFactory` からロード可能なコンポーネント。スタンドアロンの場合は `pose2twist_node` で同クラスをインスタンス化。

#### 購読トピック (Subscribed Topics)
| 名称   | 型                              | 説明                                                 |
| ------ | ------------------------------- | ---------------------------------------------------- |
| `pose` | `geometry_msgs/msg/PoseStamped` | 外部で算出されたタイムスタンプ付の位置姿勢推定の入力 |

#### 公開トピック (Published Topics)
| 名称        | 型                               | 説明                                                                                      |
| ----------- | -------------------------------- | ----------------------------------------------------------------------------------------- |
| `twist`     | `geometry_msgs/msg/TwistStamped` | 推定した並進・角速度。ヘッダは最新の `pose` から引き継ぎ、`frame_id` はパラメータで設定。 |
| `linear_x`  | `std_msgs/msg/Float32`           | 並進速度の大きさ (m/s)。`TwistStamped.twist.linear.x` と同値                              |
| `angular_z` | `std_msgs/msg/Float32`           | ヨー角速度 (rad/s)。`TwistStamped.twist.angular.z` と同値                                 |

#### パラメータ (Parameters)
| 名前             | 型     | 既定値      | 説明                                                                                                        |
| ---------------- | ------ | ----------- | ----------------------------------------------------------------------------------------------------------- |
| `twist_frame_id` | string | `base_link` | `twist` に設定する `frame_id`。`base_link` や `odom` など、下流ノードが参照する座標系に合わせて変更します。 |

### 機能とテスト対応 (Feature-to-Test Traceability)
各テストケースは外部仕様で抑えるべき機能に 1:1 対応しています。詳細なテスト手順は `TEST_SPEC.md` を参照してください。

| 機能 ID | 機能説明                                                                                         | テスト ID                             | テストで検証する観点                                                                            |
| ------- | ------------------------------------------------------------------------------------------------ | ------------------------------------- | ----------------------------------------------------------------------------------------------- |
| F-001   | 連続する Pose の距離/時間から並進速度の大きさを算出し `TwistStamped.twist.linear.x` に格納する。 | UT_POSE2TWIST_001                     | Pose 間距離 0.2236 m を 0.1 s で割った結果が線形速度に反映される。                              |
| F-002   | 異常な経過時間を検出した時，速度0を出力し、WARN ログで通知する。                                 | UT_POSE2TWIST_002 / IT_POSE2TWIST_003 | Δt<=0 で全成分 0 の `twist` を出し、"non-positive delta time" をスロットル警告。                |
| F-003   | ヨーが ±π を跨いでも最短角距離で角速度を算出する。                                               | UT_POSE2TWIST_003                     | 179°→-179° の遷移でも -0.698 rad/s 付近の角速度になる。                                         |
| F-004   | 初回`pose`受信時はゼロ速度を各トピックへpublishし、`pose`をキャッシュする。                      | UT_POSE2TWIST_004                     | `twist`/`linear_x`/`angular_z` が 0 で publish される                                           |
| F-005   | 2 件目以降はキャッシュを更新し、Twist とスカラートピックが同じ数値で出力される。                 | UT_POSE2TWIST_005                     | 並進 1.0 m/s・角速度 0.1 rad/s が 3 トピックで一致する。                                        |
| F-006   | 実行時に上書きした `twist_frame_id` パラメータを出力に即時反映する。                             | UT_POSE2TWIST_006 / IT_POSE2TWIST_002 | `set_parameters` や launch 引数で `twist_frame_id` を変更するとヘッダ frame_id が上書きされる。 |
| F-007   | コンポーネントコンテナ経由でも メッセージのQoSとremapを維持し、想定通りの速度を推定する。        | IT_POSE2TWIST_001                     | launch.py で起動した際に 2.0±0.05 m/s の連続出力とスカラー値の同期を確認する。                  |
| F-008   | launch 引数でトピック名や frame_id を上書きした場合でも Remap が機能する。                       | IT_POSE2TWIST_002                     | `/test/*` への remap と `twist_frame_id:=odom` が出力に反映される。                             |

## 起動ファイル (Launch Files)
### `launch/pose2twist.launch.py`
Composable Node Container 用の Python launch。以下の引数を提供します。

| 引数                   | 既定値                              | 説明                                   |
| ---------------------- | ----------------------------------- | -------------------------------------- |
| `container_name`       | `pose2twist_container`              | 利用するコンポーネントコンテナ名。     |
| `container_executable` | `component_container_mt`            | マルチスレッドコンテナの実行ファイル。 |
| `input_pose_topic`     | `/localization/pose_estimator/pose` | 購読するポーズトピック。               |
| `output_twist_topic`   | `/estimate_twist`                   | 公開するツイストトピック。             |
| `twist_frame_id`       | `base_link`                         | 出力する frame_id。                    |
| `params_file`          | `params/pose2twist.param.yaml`      | 追加パラメータを含む YAML。            |

起動例:
```bash
ros2 launch pose2twist pose2twist.launch.py \
  input_pose_topic:=/localization/pose_estimator/pose \
  output_twist_topic:=/estimate_twist \
  twist_frame_id:=odom
```

### `launch/pose2twist.launch.xml`
スタンドアロンノード (`pose2twist_node`) を起動する XML 形式。Python 版と同じ引数を提供し、`ros2 launch pose2twist pose2twist.launch.xml` で利用します。

## 設定 (Configuration)
- 既定パラメータは `params/pose2twist.param.yaml` に格納されています。
- `twist_frame_id` をスタック内の座標系に合わせて変更してください。地図基準で速度を扱いたい場合は `odom` や `map` などを指定します。
- QoS は入力に `SensorDataQoS`、出力に `QoS{KeepLast(10)}.reliable()` を使用しています。必要に応じてソースコードを翻案してください。

## 使用方法 (Usage)
1. 依存関係をビルド:
   ```bash
   colcon build --packages-select pose2twist
   source install/setup.bash
   ```
2. **コンポーネントコンテナ実行**:
   ```bash
   ros2 launch pose2twist pose2twist.launch.py \
     input_pose_topic:=/your_pose_topic \
     output_twist_topic:=/your_twist_topic
   ```
3. **ノード実行**:
   ```bash
   ros2 launch pose2twist pose2twist.launch.xml \
     twist_frame_id:=base_link
   ```

## テスト (Testing)
- 仕様は `TEST_SPEC.md` を参照してください。
- 単体テストは `gtest`、結合テストは `launch_testing_ros` 想定です。
```bash
colcon test --packages-select pose2twist
```

## トラブルシューティング (Troubleshooting)
- 入力 Pose の timestamp が単調増加しているか、`ros2 topic hz` で確認してください。不正な Δt では WARN ログとゼロ速度が出力されます。
- 異常なクォータニオン（正規化されていない姿勢）は角速度計算の発散を招きます。必要に応じて入力側で正規化してください。
- 速度が 0 のままの場合、`input_pose_topic` のリマップと QoS が一致しているか、Launch 引数が意図通り渡っているかを確認してください。
- 高頻度で publish する場合は `component_container_mt` を使い、CPU 使用率が高い場合は単体ノード実行に切り替えて計測してください。
