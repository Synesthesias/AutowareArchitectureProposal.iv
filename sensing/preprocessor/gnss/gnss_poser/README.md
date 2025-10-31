# gnss_poser

## 概要
`gnss_poser` パッケージは、GNSS の `NavSatFix` 観測値を車両ローカライゼーションのための map フレームの姿勢へ変換します。GNSS fix を取り込み、（UTM、MGRS、日本の平面直角座標の）座標変換を適用し、アンテナ位置に対して中央値フィルタを行い、車両のベースフレームに整合した姿勢を出力します。さらに、Fix 状態をブール値で出力し、 `map` から `gnss_base_link` への TF をブロードキャストします。

## ノード概要
| 名称         | 種別           | 説明                                                                                      |
| ------------ | -------------- | ----------------------------------------------------------------------------------------- |
| `gnss_poser` | `rclcpp::Node` | `NavSatFix` を購読し、スムージングした車両姿勢を公開し、TF をブロードキャストするノード。 |

## クイックスタート
1. ワークスペースをビルド: `colcon build --packages-select gnss_poser`
2. セットアップを読み込み: `source install/setup.bash`
3. ノードを起動: `ros2 launch gnss_poser gnss_poser.launch.xml`
4. 出力を確認: `ros2 topic echo /gnss_pose` および `ros2 topic echo /gnss_fixed`

## 起動ファイル
| 起動ファイル            | 目的                                                       | 主な引数                                                                                                  |
| ----------------------- | ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `gnss_poser.launch.xml` | トピックのリマップとフレーム設定を行った状態でノード起動。 | `input_topic_fix`, `output_topic_gnss_pose`, `coordinate_system`, `buff_epoch`, `plane_zone`, フレーム ID |

## パラメータ
| 名称                | 型       | 既定値           | 説明                                                    |
| ------------------- | -------- | ---------------- | ------------------------------------------------------- |
| `coordinate_system` | `int`    | `1`              | 変換に用いる座標系（`0:UTM`, `1:MGRS`, `2:PLANE`）。    |
| `base_frame`        | `string` | `base_link`      | 出力姿勢の基準となる車両ベースフレーム。                |
| `gnss_frame`        | `string` | `gnss`           | GNSS アンテナに紐づくフレーム。                         |
| `gnss_base_frame`   | `string` | `gnss_base_link` | map フレーム下でブロードキャストされる子フレーム。      |
| `map_frame`         | `string` | `map`            | 姿勢が出力されるグローバル基準フレーム。                |
| `buff_epoch`        | `int`    | `1`              | 中央値フィルタのバッファ長（最低 1 に丸め）。           |
| `plane_zone`        | `int`    | `9`              | `coordinate_system=2`（平面直角）のときに用いる系番号。 |

## Subscribed topics
| トピック | 型                          | QoS | 説明                                                                |
| -------- | --------------------------- | --- | ------------------------------------------------------------------- |
| `fix`    | `sensor_msgs/msg/NavSatFix` | 10  | 緯度・経度・高度・状態・共分散を含む RTK もしくは GNSS フィックス。 |

## Published topics
| トピック        | 型                                            | QoS | 説明                                                                             |
| --------------- | --------------------------------------------- | --- | -------------------------------------------------------------------------------- |
| `gnss_pose`     | `geometry_msgs/msg/PoseStamped`               | 10  | 座標変換とベースフレーム整合後の `map` フレームにおける車両姿勢。                |
| `gnss_pose_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 10  | 共分散付き姿勢。入力共分散が不明な場合は対角のフォールバック（10 m^2）を用いる。 |
| `gnss_fixed`    | `std_msgs/msg/Bool`                           | 10  | 高品質（STATUS_FIX 以上）の GNSS ステータスを示すブール。                        |

## TF フレーム
- ブロードキャスト: 推定姿勢に基づく `map` -> `gnss_base_link`。
- 入力要件: `gnss_frame` と `base_frame` の静的 TF。未提供の場合は警告を出し、恒等変換を仮定する。

## サポートする座標系
- **UTM (0)**: GeographicLib の UTMUPS を用いて東距/北距を算出し、ジオイド高補正を適用。
- **MGRS (1)**: UTM を MGRS に変換。精度は設定可能（既定は 0.1 mm）。
- **PLANE (2)**: `geo_pos_conv` による日本の平面直角座標系変換。`plane_zone` で系を指定。

## フィルタリングと姿勢（ヨー）
- `buff_epoch` 個のサンプルに対して、x/y/z の各軸で中央値フィルタを適用して外れ値を抑制。
- ヨー角は最新の中央値と直前の中央値の差分ベクトルから算出（ロール/ピッチは 0）。

## テスト
gtest による単体テストおよび ROS 2 launch testing を用いた結合テストの計画は `TEST_SPEC.md` を参照してください。

## 既知の制約
- 正高（Orthometric Height）への変換には EGM2008 のジオイドデータが必要。環境に GeographicLib のデータセットをインストールすること。
- `NavSatFix.status` が劣化または NO_FIX の場合、姿勢出力は意図的にスキップされる。下流モジュールは `gnss_fixed` を監視すること。
- GNSS アンテナと車両ベースフレームの静的 TF が不正確/未提供の場合、アンテナ位置基準の姿勢となり、車両原点との空間オフセットが生じうる。

## 参考文献
- Autoware documentation: https://autowarefoundation.github.io/autoware-documentation
- GeographicLib: https://geographiclib.sourceforge.io/
