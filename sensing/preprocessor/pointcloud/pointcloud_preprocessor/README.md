# pointcloud_preprocessor

## 概要

- `sensor_msgs/msg/PointCloud2` を処理するコンポーネント群
  - **ConcatenateDataNode**: 複数の LiDAR ストリームを購読し、共通フレームへ変換したうえで、結合した点群を出力
    - 任意で Twist によるモーション補償も適用
  - **DistanceBasedCompareMapFilterNode**: 入力点群を参照マップと比較し、ユーザが設定した距離の閾値以内の点のみを残し、出力

いずれのノードも、ノード単体での実行またはマルチスレッドの `rclcpp_components` コンテナ内コンポーネントとして動作

## ノード

### ConcatenateDataNode (`concatenate_data`)

#### subscribed topics
- `input_topics` で指定した各トピック（`sensor_msgs/msg/PointCloud2`、SensorDataQoS）
- `twist_topic`（`geometry_msgs/msg/TwistStamped`、BestEffort、任意。`use_twist_compensation=true` のとき有効。既定: `/vehicle/status/twist`）

#### published topics
- `output`（`sensor_msgs/msg/PointCloud2`）: 結合済み点群
- `concat_num`（`std_msgs/msg/Int32`）: 直近の結合に含まれる点群数
- `not_subscribed_topic_name`（`std_msgs/msg/String`）: タイムアウト時に未受信だった入力トピック名をカンマ区切りで格納

#### services
- なし

#### action API
- なし

#### parameters

| 名称                     | 型           | 既定値                  | 説明                                                     |
| ------------------------ | ------------ | ----------------------- | -------------------------------------------------------- |
| `input_topics`           | list<string> | `[]`                    | 結合する点群トピックの一覧                               |
| `output_frame`           | string       | `""`                    | 結果を変換する目標フレーム（空ならセンサフレームを維持） |
| `timeout_sec`            | double       | `0.1`                   | 部分的な結合で出力するまでの最大待機時間                 |
| `max_queue_size`         | int          | `10`                    | 各サブスクリプションの QoS 履歴の大きさ                  |
| `use_twist_compensation` | bool         | `true`                  | Twist によるモーション補償の有効/無効                    |
| `twist_topic`            | string       | `/vehicle/status/twist` | `TwistStamped` のトピック名                              |
| `max_twist_dt`           | double       | `0.1`                   | 補償に利用する Twist サンプル間の最大許容時差（秒）      |

#### required tf
- センサフレームと異なる文字列(文字数>0)が `output_frame` に指定されている場合:
	- センサフレーム -> `output_frame`
- それ以外の場合:
    - なし

#### provided tf
- なし

### DistanceBasedCompareMapFilterNode (`distance_based_compare_map_filter`)

#### subscribed topics
- `input`（`sensor_msgs/msg/PointCloud2`、SensorDataQoS）
- `map`（`sensor_msgs/msg/PointCloud2`、Reliable + Transient Local）

#### published topics
- `output`（`sensor_msgs/msg/PointCloud2`）: フィルタ済み点群

#### services
- なし

#### action API
- なし

#### parameters

| 名称                 | 型     | 既定値 | 説明                                                         |
| -------------------- | ------ | ------ | ------------------------------------------------------------ |
| `distance_threshold` | double | `1.0`  | マップ最近傍点からの最大距離（m）。この距離以内の点を保持    |
| `target_frame`       | string | `""`   | マップおよび出力を変換するフレーム。空なら入力フレームを維持 |
| `tf_timeout_sec`     | double | `0.2`  | TF ルックアップのタイムアウトまでの秒数                      |

`distance_threshold`、`target_frame`、`tf_timeout_sec` は `ros2 param set` により実行時に更新されます。

#### required tf
- 次のフレーム間で必要に応じて TF を要求します（対象時刻のルックアップ）。
	- `target_frame` を指定した場合: `map.header.frame_id -> target_frame`、`input.header.frame_id -> (map のフレームまたは target_frame)`
	- `target_frame` 未指定の場合: `input.header.frame_id -> map.header.frame_id`（入力をマップ座標に変換）

#### provided tf
- なし

## 起動方法

### コンポーネントコンテナ
本パッケージには `launch/preprocessor.launch.py` が含まれており、`ConcatenateDataNode` をマルチスレッドコンテナにロードします。例:
```bash
ros2 launch pointcloud_preprocessor preprocessor.launch.py \
	input_topics:="['/points_raw', '/points_rear']" \
	output_frame:=base_link \
	timeout_sec:=0.15 \
	use_twist_compensation:=true
```

結合結果トピックを変更するには、起動引数 `output_topic` を使用してください。

### ノード単体実行
```bash
ros2 run pointcloud_preprocessor concatenate_data_node
ros2 run pointcloud_preprocessor distance_based_compare_map_filter_node \
	--ros-args -p distance_threshold:=0.5 -p target_frame:=map \
	-r input:=/points_no_ground -r map:=/global_map
```

## 運用上の注意
- いずれのノードもTFの同期を前提
- デフォルト挙動が TF ツリーに合わない場合は、`input_frame`/`output_frame` パラメータを設定
- `ConcatenateDataNode` は約 1 秒分の Twist をバッファに保持し、`max_twist_dt` を超えるギャップがある場合は補償をスキップしてスロットル付き警告を出力
- `DistanceBasedCompareMapFilterNode` は新しいマップ受信時に KdTree を再構築
- マップは Reliable + Transient QoS で配信し、常時利用可能にする必要がある
- 結合数や欠落トピックの検知には `concat_num` と `not_subscribed_topic_name` を利用する

