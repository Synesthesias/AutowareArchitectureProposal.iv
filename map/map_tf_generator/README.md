# map_tf_generator

## 概要 (Overview)
- `map_tf_generator` は地図点群の幾何中心を並進成分とした静的 TF（`map_frame`→`viewer_frame`）を生成し、ビューアで利用する可視化のためのTFを出力するノードです
- 入力は `sensor_msgs/msg/PointCloud2` で、`tf2_ros::StaticTransformBroadcaster` を通じて `/tf_static` に配信されます

### 特長 (Key Features)
- `pointcloud_map` の `SensorDataQoS` で点群を入力として受け取り、`x/y/z` フィールドの平均値を算出
- parent/child フレーム名をパラメータから即時反映し、可視化ツールや RViz のビュー原点を統一
- 空点群・必須フィールド欠如・非有限値（NaN/Inf）を検出して WARN ログとともに無視
- 最新のタイムスタンプを保持する静的 TF を参照し、後続のTFリスナーに即時共有
- 付属の XML launch で入力トピックやフレーム名を引数からパラメータとして上書きが可能

## Nodes
### map_tf_generator::MapTfGeneratorNode
- **型**: 単体ノード（`rclcpp::Node`）
- **実行ファイル**: `map_tf_generator`

#### Subscribed Topics
| 名称             | 型                            | 説明                                                                                                        |
| ---------------- | ----------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `pointcloud_map` | `sensor_msgs/msg/PointCloud2` | 入力地図点群。`SensorDataQoS`（BestEffort, depth=5）で subscribe。既定のでは `/map/pointcloud_map` へ remap |

#### Published Topics
| 名称 | 型  | 説明                      |
| ---- | --- | ------------------------- |
| なし | -   | TF は `/tf_static` で提供 |

#### Provided TF
| parent → child               | 内容                                                                                                                                                     |
| ---------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `map_frame` → `viewer_frame` | `translation` は入力点群の重心、`rotation` は (0,0,0,1)。`header.stamp` は入力のものを参照し、`tf2_ros::StaticTransformBroadcaster` がラッチして再配信。 |

#### Parameters
| 名前           | 型 / 既定値       | 説明                                                                    |
| -------------- | ----------------- | ----------------------------------------------------------------------- |
| `map_frame`    | string / `map`    | 生成する静的 TF の parent フレーム。RViz の Fixed Frame に設定する      |
| `viewer_frame` | string / `viewer` | 重心を原点とする child フレーム。可視化用のフレーム名を任意に設定できる |

#### QoS / etc.
- `pointcloud_map` の QoS は `rclcpp::SensorDataQoS()` をそのまま採用。Publisher が QoS で Reliable を要求する場合は、Subscriber 側も合わせて互換設定が必要になります
- 入力点群の TF 依存性はなく、`tf2` との同期は不要です

## 機能とテスト対応 (Feature-to-Test Traceability)
- テストケースと外部仕様機能の対応関係を 1:1 で整理
- 詳細な手順は `TEST_SPEC.md` を参照

| 機能 ID | 機能説明                                                                              | テスト ID | テストで検証する観点                                                              |
| ------- | ------------------------------------------------------------------------------------- | --------- | --------------------------------------------------------------------------------- |
| F-001   | `x/y/z` の平均を並進に用いて静的 TF を生成する                                        | UT-001    | 既知の 3 点を入力して重心=(4,5,6) が `translation` に反映される                   |
| F-002   | 空点群を検知して TF を更新せず WARN を出す                                            | UT-002    | `width*height=0` で TF が出ず、"Received empty pointcloud" がログ出力される       |
| F-003   | `x/y/z` 欠損点群の受取を拒否し、エラーログで通知する                                  | UT-003    | 欠損フィールドの点群で TF を生成しないこととWARNのログ出力を確認                  |
| F-004   | NaN/Inf を含む点群をそのまま加算し、結果をそのまま出力する（現行仕様の挙動を把握）    | UT-004    | 非有限値が `translation` に伝播する現行仕様の挙動                                 |
| F-005   | organized/unorganizedのどちらでも同一の重心を計算する                                 | UT-005    | 同一集合を height=1 と height>1 で入力して同値になる                              |
| F-006   | 約 10^6 点規模でも O(N) の線形走査で重心を算出できる                                  | UT-006    | 疑似点群を入力してタイムアウトせずに処理できる                                    |
| F-007   | 既定パラメータのままでも map→viewer の静的 TF を配信する                              | IT-001    | (1,2,3),(4,5,6),(7,8,9) を publish し、親=`map`, 子=`viewer`, 並進=(4,5,6) になる |
| F-008   | ノード実行時にも空点群を無視して既存 TF を保持する                                    | IT-002    | 空点群 publish 後も TF が増えずクラッシュしない                                   |
| F-009   | 欠損フィールドを含む点群を検出し、TF を更新しない                                     | IT-003    | `z` 欠如点群で TF が変化せず警告ログが出る                                        |
| F-010   | フレーム指定パラメータを静的 TF の親子 frame_id に即時反映する                        | IT-004    | `map_frame:=world`, `viewer_frame:=viz` で起動し、生成 TF の frame_id を置換する  |
| F-011   | 最新点群の `header.stamp` を `/tf_static` に反映                                      | IT-005    | 既知スタンプを持つ点群を publish し、TF 側 stamp が一致する                       |
| F-012   | `/tf_static` の参照により最後の TF を再subscribe時も即時取得できる                    | IT-006    | 重心を 2 度更新後に新規リスナーを接続し、最新の並進を即時取得できる               |
| F-013   | organized / unorganized 点群を順に publish しても同じ TF が得られる                   | IT-007    | 2 形態の publish 後で並進が許容誤差内で一致する                                   |
| F-014   | 非有限値を含む点群を publish した際の TF 出力挙動を把握し、将来の仕様変更の破綻を防ぐ | IT-008    | NaN/Inf を含む点群で並進に同じ値が現れることを確認し、挙動をログ出力              |
| F-015   | Launch 引数で入力トピックを remap しつつ TF が生成される                              | IT-009    | `input_map_points_topic:=/custom/map_cloud` 経由の publish で同じ TF が出力される |

## 起動ファイル (Launch Files)
### `launch/map_tf_generator.launch.xml`
- 引数: `input_map_points_topic` (既定 `/map/pointcloud_map`), `map_frame` (`map`), `viewer_frame` (`viewer`).
- Launch では入力トピックを remap しつつノードを単体起動します
- 静的 TF の parent / child frame_id は指定の引数がそのまま適用されます

起動例:
```bash
ros2 launch map_tf_generator map_tf_generator.launch.xml \
  input_map_points_topic:=/map/pointcloud_map \
  map_frame:=map \
  viewer_frame:=viewer
```

## 使用方法 (Usage)
1. 依存パッケージをビルドして環境を source:
   ```bash
   colcon build --packages-select map_tf_generator
   source install/setup.bash
   ```
2. **ノード単体実行**:
   ```bash
   ros2 run map_tf_generator map_tf_generator
   ```
3. **Launch 実行**:
   ```bash
   ros2 launch map_tf_generator map_tf_generator.launch.xml \
     input_map_points_topic:=/map/pointcloud_map \
     map_frame:=map \
     viewer_frame:=viewer
   ```

## テスト (Testing)
- 仕様詳細とテスト手順は `TEST_SPEC.md` を参照してください。
- gtest を想定した単体テストと、`launch_testing` / rclcpp ベースの結合テストで F-001〜F-015 をカバーしています
- 例:
  ```bash
  colcon test --packages-select map_tf_generator
  ```

## トラブルシューティング (Troubleshooting)
- 入力 QoS が `SensorDataQoS` と互換か確認してください。一致しないと購読できません。
- 空点群や `x/y/z` 欠損により TF が生成されない場合、警告ログを確認し点群生成パイプラインを修正してください。
- RViz で TF が見えない場合は `map_frame` / `viewer_frame` の命名が重複していないか、`tf2_echo` で確認してください。