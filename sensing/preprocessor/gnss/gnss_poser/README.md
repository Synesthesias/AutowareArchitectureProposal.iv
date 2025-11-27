# gnss_poser

## 概要 (Overview)
- GNSS の `sensor_msgs/msg/NavSatFix` を受け取り、GeographicLib と `geo_pos_conv` を用いて UTM / MGRS / 日本の平面直角座標へ変換し、中央値フィルタ後の姿勢を `map` フレームで公開する
- 状態判定に応じて `gnss_fixed` を出力し、`map`->`gnss_base_link` の TF もブロードキャストする

## 特長 (Key Features)
- UTM / MGRS / 平面直角の座標変換と正高（Orthometric Height）補正を提供
- GNSS アンテナ位置を中央値フィルタで平滑化し、差分ベクトルからヨー角を推定
- `base_frame` と `gnss_frame` 間の静的 TF を参照し、車両ベース座標に投影
- FIX 状態をboolトピックとタイムスタンプ付き姿勢で即時通知
- 入力共分散が未知の場合はフォールバック共分散を自動付与

## ノード (Nodes)
### GNSSPoser::GNSSPoser
- **型**: `rclcpp::Node`
- **役割**: `NavSatFix` を座標変換し、平滑化した姿勢と TF・状態を公開

#### 購読トピック (Subscribed Topics)
| 名称  | 型                          | QoS       | 説明                                                |
| ----- | --------------------------- | --------- | --------------------------------------------------- |
| `fix` | `sensor_msgs/msg/NavSatFix` | `QoS(10)` | 緯度/経度/高度とステータス、共分散を含む GNSS Fix。 |

#### 公開トピック (Published Topics)
| 名称            | 型                                            | QoS       | 説明                                                                        |
| --------------- | --------------------------------------------- | --------- | --------------------------------------------------------------------------- |
| `gnss_pose`     | `geometry_msgs/msg/PoseStamped`               | `QoS(10)` | 座標変換＆中央値フィルタ後の車両姿勢。ヘッダは `map_frame`。                |
| `gnss_pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `QoS(10)` | `gnss_pose` と同じ座標の共分散付き姿勢。未知共分散時は対角 10.0/10.0/10.0。 |
| `gnss_fixed`    | `std_msgs/msg/Bool`                           | `QoS(10)` | `NavSatStatus` が `STATUS_FIX` 以上かを通知。                               |

#### TF
- ブロードキャスト: `map_frame` -> `gnss_base_frame`
- 入力要件: `base_frame` -> `gnss_frame` の静的 TF（未提供時は警告し恒等変換）

#### パラメータ (Parameters)
| 名前                | 型       | 既定値           | 説明                                                          |
| ------------------- | -------- | ---------------- | ------------------------------------------------------------- |
| `coordinate_system` | `int`    | `1`              | `0:UTM` / `1:MGRS` / `2:PLANE` を選択。                       |
| `base_frame`        | `string` | `base_link`      | 出力姿勢の基準となる車両フレーム。                            |
| `gnss_frame`        | `string` | `gnss`           | GNSS アンテナに対応するフレーム名。                           |
| `gnss_base_frame`   | `string` | `gnss_base_link` | TF で公開する子フレーム。                                     |
| `map_frame`         | `string` | `map`            | 世界座標系の基準フレーム。                                    |
| `buff_epoch`        | `int`    | `1`              | 中央値フィルタのバッファ長（1 未満を指定しても 1 に丸める）。 |
| `plane_zone`        | `int`    | `9`              | 平面直角座標系（JGD2011）の系番号。                           |

## 機能とテスト対応 (Feature-to-Test Traceability)
| 機能 ID | 機能説明                                                                            | テスト ID | テストで検証する観点                                                                              |
| ------- | ----------------------------------------------------------------------------------- | --------- | ------------------------------------------------------------------------------------------------- |
| F-001   | UTM 座標系へ変換し、東距/北距と正高が基準値と一致する。                             | UT-GP-001 | `NavSatFix2UTM` が国土地理院ベンチマーク値と ±0.01 m 以内。                                       |
| F-002   | MGRS 変換で指定精度 (0.1 mm) の桁が保持される。                                     | UT-GP-002 | `NavSatFix2MGRS` 出力が UTM 結果と整合し、`x/y` が ±0.1 m。                                       |
| F-003   | 平面直角座標 (PLANE) で `geo_pos_conv` と同じ軸入れ替えを行う。                     | UT-GP-003 | `NavSatFix2PLANE` の `x/y` が `geo_pos_conv` の `y/x` と一致。                                    |
| F-004   | UTM→MGRS 変換で指定精度ごとに桁落ちが変化する。                                     | UT-GP-004 | `UTM2MGRS` の結果桁が 1 m / 10 m 精度に応じて丸められる。                                         |
| F-005   | Orthometric Height への変換が成功し、例外時は入力高度を保持しつつエラーログを出す。 | UT-GP-005 | `EllipsoidHeight2OrthometricHeight` が EGM2008 と ±0.05 m で一致し、異常時は警告+フォールバック。 |
| F-006   | FIX 測位を受信すると中央値フィルタ後の姿勢/共分散/TF/固定フラグを出力する。         | IT-GP-001 | `gnss_pose`/`gnss_pose_cov`/`gnss_fixed` が正しい値と姿勢ヨーを出力。                             |
| F-007   | `STATUS_NO_FIX` など非固定時は姿勢出力をスキップし `gnss_fixed=false` を通知。      | IT-GP-002 | `gnss_pose*` が未発行のまま、`gnss_fixed` が false。                                              |
| F-008   | `buff_epoch>1` で中央値フィルタを満たすまで出力を抑制し、中央値を採用する。         | IT-GP-003 | 5 件バッファ後に中央値に従う姿勢とヨーが配信される。                                              |
| F-009   | `base_frame->gnss_frame` TF が欠落しても恒等変換で継続し、警告を出す。              | IT-GP-004 | 恒等 TF で姿勢出力が継続し、警告ログがスロットル出力。                                            |
| F-010   | 入力共分散が UNKNOWN の場合、フォールバック対角値を設定する。                       | IT-GP-005 | `gnss_pose_cov` の xyz 共分散が 10.0 に置換される。                                               |

## 起動ファイル (Launch Files)
| ファイル                       | 既定値/説明                                                                                                                                                                    |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `launch/gnss_poser.launch.xml` | `input_topic_fix`, `output_topic_gnss_pose`, `coordinate_system`, `plane_zone`, `buff_epoch`, 各フレーム ID を引数化した XML launch。コンポーネント/単体ノードいずれにも対応。 |

起動例:
```bash
ros2 launch gnss_poser gnss_poser.launch.xml \
	input_topic_fix:=/sensing/gnss/fix \
	output_topic_gnss_pose:=/localization/gnss_pose \
	coordinate_system:=2 plane_zone:=9 buff_epoch:=5
```

## 使用方法 (Usage)
1. 依存をビルド:
	 ```bash
	 colcon build --packages-select gnss_poser
	 source install/setup.bash
	 ```
2. TF 構成を確認し、`ros2 launch gnss_poser gnss_poser.launch.xml` で起動。
3. `ros2 topic echo /gnss_pose` や `tf2_echo map gnss_base_link` で出力を検証。

## テスト (Testing)
- 仕様は `TEST_SPEC.md` を参照。単体テストは gtest、結合テストは `launch_testing_ros` を想定。
- 実行例:
	```bash
	colcon test --packages-select gnss_poser
	```

## トラブルシューティング (Troubleshooting)
- **Fix が false のまま**: `NavSatFix.status` を確認し、RTK 解や高精度モードになっているかをチェック。
- **姿勢が出力されない**: `buff_epoch` に応じてバッファが溜まるのを待つか、`STATUS_FIX` のメッセージを送信してください。
- **高さが期待と異なる**: `GEOGRAPHICLIB_DATA` が EGM2008 データを指すか確認。
- **TF エラーが頻発**: `base_frame -> gnss_frame` の静的 TF を `ros2 run tf2_ros static_transform_publisher ...` で公開し、警告を解消します。
