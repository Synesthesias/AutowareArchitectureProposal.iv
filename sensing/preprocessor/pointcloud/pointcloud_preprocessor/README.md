# pointcloud_preprocessor

## 概要 (Overview)
- `pointcloud_preprocessor` は `sensor_msgs/msg/PointCloud2` の前処理を行うパッケージです。
- 複数 LiDAR の結合や、地図との比較フィルタといった上流の前処理を ROS2コンポーネントおよび、単体ノードとして提供します。

### ノード一覧
- **ConcatenateDataNode**: 複数点群の同期・TF 変換・Twist 補償・欠落監視を行い 1 本の点群を出力。
- **DistanceBasedCompareMapFilterNode**: 参照マップとのユークリッド距離で入力点群を選別し、任意のフレームで出力。

## ConcatenateDataNode (`concatenate_data`)

### 機能
- `input_topics` で指定したすべての点群をセンサ QoS で購読し、`output_frame` へ TF 変換したあと最新時刻のヘッダで結合。
- `timeout_sec` 以内に揃わない入力は部分的に出力し、`concat_num` と `not_subscribed_topic_name` で結合数/欠落トピックを可視化。
- 入力が高速なトピックは `pending_clouds` に繰り越して次サイクルへ反映し、データ欠落を防止。
- `use_twist_compensation=true` の場合は `twist_topic` から取得した `geometry_msgs/msg/TwistStamped` を用いて非同期入力間の相対運動を補償。`max_twist_dt` を超えるギャップでは警告し補償をスキップ。

### subscribe トピック
| 名前              | 型                               | QoS                                           | 説明                                                                        |
| ----------------- | -------------------------------- | --------------------------------------------- | --------------------------------------------------------------------------- |
| `input_topics[i]` | `sensor_msgs/msg/PointCloud2`    | `SensorDataQoS`                               | 結合対象となる複数点群。topic 名はパラメータから動的に決定。                |
| `twist_topic`     | `geometry_msgs/msg/TwistStamped` | `QoS{KeepLast(max_queue_size)}.best_effort()` | 任意。モーション補償に利用。`use_twist_compensation=true` 時のみsubscribe。 |

### publish トピック
| 名前                        | 型                            | QoS             | 説明                                                         |
| --------------------------- | ----------------------------- | --------------- | ------------------------------------------------------------ |
| `output`                    | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | TF 変換と結合済み点群。タイムスタンプは最新の入力と一致。    |
| `concat_num`                | `std_msgs/msg/Int32`          | `QoS(10)`       | 直近の結合で利用されたトピック数。                           |
| `not_subscribed_topic_name` | `std_msgs/msg/String`         | `QoS(10)`       | タイムアウト時に未受信だったトピック名をカンマ区切りで通知。 |

### パラメータ
| 名前                     | 型                 | 既定値                  | 説明                                                                    |
| ------------------------ | ------------------ | ----------------------- | ----------------------------------------------------------------------- |
| `input_topics`           | list&lt;string&gt; | `[]`                    | 結合する点群トピック名のリスト。                                        |
| `output_frame`           | string             | `""`                    | TF 変換先。空文字なら各入力のフレームを維持。                           |
| `timeout_sec`            | double             | `0.1`                   | 全入力が揃わない場合に部分結合を許可する待機時間 [s]。                  |
| `max_queue_size`         | int                | `10`                    | 各publishのキューの深さ。QoS `KeepLast` と Twist バッファサイズに利用。 |
| `use_twist_compensation` | bool               | `true`                  | Twist によるモーション補償の有効/無効。                                 |
| `twist_topic`            | string             | `/vehicle/status/twist` | モーション補償に利用する Twist トピック。                               |
| `max_twist_dt`           | double             | `0.1`                   | Twist サンプル間の最大許容時差。超過時は補償をスキップ。                |

### TF 要件
- `output_frame` が空でなければ `input.header.frame_id → output_frame` が必要。
- Twist 補償のみでは TF を生成しないため、他の TF 供給元を用意する。

## DistanceBasedCompareMapFilterNode (`distance_based_compare_map_filter`)

### 機能
- `map` を Reliable + Transient Local で購読し、`pcl::search::KdTree` を再構築して最近傍距離を高速に算出。
- `distance_threshold` 以内の点のみを保持し、`target_frame` が指定されていれば入力・マップをそのフレームへ変換。
- マップ未受信時は入力をそのまま（必要に応じて `target_frame` へ変換後）透過し、ログで通知。
- `distance_threshold` / `target_frame` / `tf_timeout_sec` は `ros2 param set` で即時反映される。

### 購読トピック
| 名前    | 型                            | QoS                                   | 説明                                                   |
| ------- | ----------------------------- | ------------------------------------- | ------------------------------------------------------ |
| `input` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS`                       | フィルタ対象の点群。必要に応じてマップフレームへ変換。 |
| `map`   | `sensor_msgs/msg/PointCloud2` | `QoS(1).reliable().transient_local()` | 参照マップ。受信時に内部 KdTree を更新。               |

### 公開トピック
| 名前     | 型                            | QoS             | 説明                                                          |
| -------- | ----------------------------- | --------------- | ------------------------------------------------------------- |
| `output` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | フィルタ済み点群。`target_frame` が空なら入力フレームで返却。 |

### パラメータ
| 名前                 | 型     | 既定値 | 説明                                                             |
| -------------------- | ------ | ------ | ---------------------------------------------------------------- |
| `distance_threshold` | double | `1.0`  | 最近傍距離の閾値 [m]。この値以内の点を保持。                     |
| `target_frame`       | string | `""`   | マップ保持および出力に用いるフレーム。空なら入力フレームを維持。 |
| `tf_timeout_sec`     | double | `0.2`  | TFを参照する際のタイムアウト [s]。                               |

### TF 要件
- `target_frame` 指定時: `map.header.frame_id → target_frame` と `input.header.frame_id → target_frame`。
- `target_frame` 未指定時: `input.header.frame_id → map.header.frame_id`（入力をマップ座標に変換）

## 起動ファイル (Launch)

### `launch/preprocessor.launch.py`
- `ConcatenateDataNode` を `component_container_mt` で起動し、以下の引数を提供します。

| 引数                     | 既定値                     | 説明                                      |
| ------------------------ | -------------------------- | ----------------------------------------- |
| `input_topics`           | `['/points_raw']`          | 結合する点群。Python リスト文字列で指定。 |
| `output_frame`           | `base_link`                | 結合結果のフレーム。                      |
| `timeout_sec`            | `0.1`                      | 待機時間 [s]。                            |
| `max_queue_size`         | `10`                       | QoS 履歴深さ。                            |
| `use_twist_compensation` | `true`                     | モーション補償の有効/無効。               |
| `twist_topic`            | `/vehicle/status/twist`    | Twist トピック。                          |
| `max_twist_dt`           | `0.1`                      | Twist サンプル最大間隔 [s]。              |
| `output_topic`           | `/points_raw/concatenated` | 出力トピック名。                          |

起動例:

```bash
ros2 launch pointcloud_preprocessor preprocessor.launch.py \
	input_topics:="['/points_raw','/points_rear']" \
	output_frame:=base_link \
	timeout_sec:=0.15 \
	use_twist_compensation:=true
```

## ノード単体実行

```bash
# Composition を使わずに個別ノードを起動
ros2 run pointcloud_preprocessor concatenate_data_node \
	--ros-args -p input_topics:="['/lidar/front','/lidar/rear']" -p output_frame:=map

ros2 run pointcloud_preprocessor distance_based_compare_map_filter_node \
	--ros-args -p distance_threshold:=0.5 -p target_frame:=map \
	-r input:=/points_no_ground -r map:=/global_map
```

## 機能とテスト対応 (Feature-to-Test Traceability)

| 機能ID     | ノード                            | 機能説明                                                                              | テストID                          | テストで確認する観点                                                                      |
| ---------- | --------------------------------- | ------------------------------------------------------------------------------------- | --------------------------------- | ----------------------------------------------------------------------------------------- |
| F-CDN-001  | ConcatenateDataNode               | 複数入力を最新時刻で結合し `output` のヘッダ stamp を最も新しい入力時刻に揃える。     | UT-CDN-001 / CDN-001              | gtest で最新時刻が選択されること、結合シナリオでヘッダ一致と `concat_num=入力数` を確認。 |
| F-CDN-002  | ConcatenateDataNode               | 欠落トピックを `not_subscribed_topic_name` に列挙し、`concat_num` に参加数を出力。    | UT-CDN-002 / CDN-002              | 欠落検知ロジックと実ノードでのタイムアウト通知を確認。                                    |
| F-CDN-003  | ConcatenateDataNode               | `timeout_sec` 経過で部分結合を出力し、バッファをリセット。                            | UT-CDN-003 / CDN-007              | タイムアウト後のリセットと次サイクルの初期化を確認。                                      |
| F-CDN-004  | ConcatenateDataNode               | 早期に到着したデータを `pending_clouds` に繰り越して次サイクルへ引き継ぐ。            | UT-CDN-004 / CDN-006              | 保留バッファ繰り上げでデータ欠落が起きないことを確認。                                    |
| F-CDN-005  | ConcatenateDataNode               | Twist 補償で非同期入力を整合する。                                                    | UT-CDN-005 / CDN-004              | 解析解と一致する座標で結合されることを確認。                                              |
| F-CDN-006  | ConcatenateDataNode               | `output_frame` で TF 変換できない入力を警告し、変換可能なデータのみで出力。           | CDN-003                           | 実動作で TFの変換失敗時に警告・除外されることを確認。                                     |
| F-CDN-007  | ConcatenateDataNode               | `max_twist_dt` を超えるギャップを検出して補償を停止し WARN を出力。                   | UT-CDN-006 / CDN-005              | ギャップ発生時に補償をスキップし WARN が出ることを確認。                                  |
| F-DCMF-001 | DistanceBasedCompareMapFilterNode | マップ未受信時は入力（必要に応じ target_frame へ変換）をパススルー。                  | UT-DCMF-001 / DCMF-001            | ロジックと実ノードでのパススルー挙動を確認。                                              |
| F-DCMF-002 | DistanceBasedCompareMapFilterNode | 最近傍距離が `distance_threshold` 以内の点のみ保持。                                  | UT-DCMF-002 / DCMF-003            | 最近傍距離計算としきい値フィルタを検証。                                                  |
| F-DCMF-003 | DistanceBasedCompareMapFilterNode | `target_frame` 管理でマップ保持と出力フレームを切替え、未設定時は入力フレームへ戻す。 | UT-DCMF-003 / DCMF-002 / DCMF-004 | 変換/復元処理を確認。                                                                     |
| F-DCMF-004 | DistanceBasedCompareMapFilterNode | `distance_threshold` / `target_frame` / `tf_timeout_sec` のパラメータ更新を即時反映。 | UT-DCMF-004 / DCMF-005            | `ros2 param set` 直後の挙動変化を確認。                                                   |
| F-DCMF-005 | DistanceBasedCompareMapFilterNode | TF 変換に失敗した入力を WARN して破棄し、復旧後は処理を継続。                         | DCMF-006                          | TF 切断時のフェイルセーフを確認。                                                         |

## テスト
- 仕様は `TEST_SPEC.md` を参照してください。
- 単体テスト/結合テストをまとめて実行するには次を利用します。

```bash
colcon test --packages-select pointcloud_preprocessor
```

## トラブルシューティング
- Twist 補償の WARN が頻発する場合は `max_twist_dt` を増やすか Twist 周期を上げてください。
- `output_frame` が TF ツリーに存在しない場合、該当入力は自動的に除外されます。TF を確認するか `output_frame` を空に設定します。
- `DistanceBasedCompareMapFilterNode` で出力が空になる場合、`distance_threshold` が小さすぎるかマップと入力のフレームが一致していない可能性があります。

