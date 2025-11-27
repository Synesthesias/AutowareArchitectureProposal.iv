# pose_initializer

## 概要 (Overview)
`pose_initializer` はローカライゼーション系に信頼できる初期姿勢を提供するノードです。RViz などからの `initialpose`、任意の GNSS 姿勢ストリーム、ラッチ配信される地図点群を購読し、各入力の高さを周囲の地図点から補正したうえで `ndt_align_srv` による姿勢合わせを行います。結果は共分散を整形して `initialpose3d` とサービス応答経由で再配信され、下流ノードが最小ドリフトで起動できるようにします。

### 主な特長 (Key Features)
- **高さ補正付きシーディング**: `pointcloud_map` のキャッシュを半径 1 m 以内でサンプリングし、TF 変換を適用して入力姿勢を地図面へスナップしてからパブリッシュします。
- **手動初期姿勢の整列**: `initialpose` で与えられた姿勢に緩めの平面共分散を設定し、`ndt_align_srv` で位置合わせしたあとにタイトな共分散へ置き換えて配信します。
- **GNSS ワンショット初期化**: `use_first_gnss_topic` が true のとき `gnss_pose_cov` を 1 回だけ利用し、整列結果をパブリッシュした直後に購読を停止して不要な再初期化を防ぎます。
- **サービス経路での初期化**: `pose_initializer_srv` でも同じ高さ補正と整列処理を再利用し、サービス応答と `initialpose3d` の双方へ結果を流します。
- **QoS を考慮した地図キャッシュ**: 地図購読は `KeepLast(1).reliable().transient_local()` を使用し、最新の点群と frame 名をミューテックス内に保持することでトピック／サービス双方で共有します。
- **アライン監視**: `ndt_align_srv` が 1 s 待っても現れない場合や応答が 3 s 以内に返らない場合は WARN を出し、高さ補正のみの姿勢をフォールバックとしてパブリッシュします。

## ノード (Nodes)
### pose_initializer::PoseInitializer
- **種別**: コンポーネント (`pose_initializer::PoseInitializer`) とスタンドアロン実行可能ファイル (`pose_initializer_node`) の両方を提供。
- **実行モデル**: センサ／サービス／クライアント用に複数のコールバックグループを用意し、地図受信・サービス処理・整列要求が互いにブロックしないようにしています。

#### 購読トピック (Subscribed Topics)
| 名称             | 型                                            | QoS                                             | 説明                                                                                             |
| ---------------- | --------------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `initialpose`    | `geometry_msgs/msg/PoseWithCovarianceStamped` | `SensorDataQoS()`                               | RViz 等から与えられる手動初期姿勢。高さ補正→整列→再配信フローをトリガします。                    |
| `gnss_pose_cov`  | `geometry_msgs/msg/PoseWithCovarianceStamped` | `SensorDataQoS()`                               | 任意の GNSS シード。`use_first_gnss_topic` が true の場合は 1 回パブリッシュ後に購読解除します。 |
| `pointcloud_map` | `sensor_msgs/msg/PointCloud2`                 | `QoS{KeepLast(1)}.reliable().transient_local()` | 地図フレームと地表高さを提供するラッチ配信の点群。TF 変換と高さ補正に利用されます。              |

#### 公開トピック (Published Topics)
| 名称            | 型                                            | QoS                            | 説明                                                                                        |
| --------------- | --------------------------------------------- | ------------------------------ | ------------------------------------------------------------------------------------------- |
| `initialpose3d` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `QoS{KeepLast(10)}.reliable()` | 高さ補正済み (必要に応じて NDT 整列済み) の姿勢。ローカライザや pose-graph に入力されます。 |

#### サービスクライアント (Service Clients)
| 名称            | 型                                                         | 目的                                                                                                          |
| --------------- | ---------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `ndt_align_srv` | `autoware_localization_srvs/srv/PoseWithCovarianceStamped` | シード姿勢を地図に整列させるサービス。要求は最大 3 s 待機し、レスポンスの共分散をチューニングして使用します。 |

#### 提供サービス (Provided Services)
| 名称                   | 型                                                         | 目的                                                                                                        |
| ---------------------- | ---------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `pose_initializer_srv` | `autoware_localization_srvs/srv/PoseWithCovarianceStamped` | 姿勢＋共分散を受け取り、高さ補正と整列パイプラインを実行した結果を応答および `initialpose3d` で返却します。 |

#### パラメータ (Parameters)
| 名称                   | 型     | 既定値 | 説明                                                                                                                   |
| ---------------------- | ------ | ------ | ---------------------------------------------------------------------------------------------------------------------- |
| `map_frame`            | string | `map`  | 地図点群をサンプリングする際のフレーム。`pointcloud_map` の `frame_id` が入っている場合は自動で上書きされます。        |
| `use_first_gnss_topic` | bool   | `true` | true の場合、最初の GNSS 整列が完了した時点で `gnss_pose_cov` 購読を停止し、ローカライゼーションの再初期化を防ぎます。 |

## 機能とテスト対応表 (Feature-to-Test Traceability)
| 機能 ID | 機能説明                                                                                                | テスト ID | テストで検証する観点                                                                                                     |
| ------- | ------------------------------------------------------------------------------------------------------- | --------- | ------------------------------------------------------------------------------------------------------------------------ |
| F-001   | 地図点群と TF を使って姿勢の高さを地面へスナップする。                                                  | UT_PI_001 | `getHeight` が半径 1 m 以内の最小高さを `z` に反映し、フレーム変換を考慮できているか。                                   |
| F-002   | 手動 `initialpose` を NDT で整列し、共分散を引き締めた上で再配信する。                                  | UT_PI_002 | 整列サービスが呼び出され、共分散対角が `[1,1,0.01,0.01,0.01,0.2]` に上書きされ、`initialpose3d` に整列結果が反映される。 |
| F-003   | GNSS ブートストラップは 1 回だけパブリッシュし、その後サブスクリプションを停止する。                    | UT_PI_003 | 最初の `gnss_pose_cov` のみ `initialpose3d` を生成し、2 件目以降は無視されること。                                       |
| F-004   | `pose_initializer_srv` がトピック経路と同じ処理を行い、成功時は必ずパブリッシュする。                   | UT_PI_004 | サービス応答の姿勢が `initialpose3d` にも出力されること。                                                                |
| F-005   | 地図 frame が変わってもキャッシュと TF lookup が一貫する。                                              | UT_PI_005 | 新しい frame を持つ `pointcloud_map` を受け取った後も、以降の `initialpose` が正しい frame／高さで変換される。           |
| F-006   | `ndt_align_srv` が不在/タイムアウトでも高さ補正のみでフォールバックし、警告を出し続けるウォッチドッグ。 | IT_PI_001 | アラインサービス不在時に WARN が 1 Hz で出力され、高さ補正のみの姿勢がパブリッシュされてノードが落ちない。               |

## テストケース (Test Cases)
| テスト ID | 目的 (Purpose)                                                       | セットアップ (Setup)                                                                                                                             | 入力・操作 (Stimulus)                                             | 期待結果 (Expected Result)                                                                                                                                           |
| --------- | -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| UT_PI_001 | 地図点群を使った高さ補正の妥当性を確認する。                         | `PoseInitializer` 派生フィクスチャで 2 つの高さを持つ `pointcloud_map` を注入し、`map`→`base_link` の TF ツリーを用意する。                      | もっとも高い点より上にある姿勢で `getHeight` を呼び出す。         | 戻り値の `z` が半径 1 m 内で最も低い高さになる。ヘッダ frame も地図ヘッダ由来に更新される。                                                                          |
| UT_PI_002 | 手動 `initialpose` が NDT 整列とパブリッシュをトリガすることを確認。 | テスト executor 上でコンポーネントを動かし、既知の姿勢を返す `ndt_align_srv` モックと `initialpose3d` 購読者を用意する。                         | `initialpose` を 1 件パブリッシュする。                           | モックサーバが 1 リクエストを受け、購読者は整列後の姿勢と `[1,1,0.01,0.01,0.01,0.2]` の共分散を受信し、ヘッダ stamp は入力からコピーされる。                         |
| UT_PI_003 | GNSS 購読が 1 回で停止することを確認。                               | `use_first_gnss_topic:=true` で起動し、整列サービスをモックする。`initialpose3d` を購読しつつ購読者数を監視できるようにする。                    | `gnss_pose_cov` に 2 件のメッセージをパブリッシュ。               | 1 件目のみ `initialpose3d` が出力され、2 件目以降はパブリッシュされない。購読もリセットされる (ログや購読者数で検証)。                                               |
| UT_PI_004 | サービス経路がトピック経路と同一であることを確認。                   | ノードを起動し、整列サービスをモック、`initialpose3d` を購読してサービス応答と比較できるようにする。                                             | `pose_initializer_srv` へ地面より高い pose を送る。               | サービス応答の姿勢が `initialpose3d` でも配信され、共分散は整列応答の値に上書きされる。                                                                              |
| UT_PI_005 | map フレーム変更後も TF が適用されることを確認。                     | `map_aligned` フレームの `pointcloud_map` をパブリッシュし、`map_aligned`↔`base_link` の TF をブロードキャストする。                             | 地図を受け取った後、`base_link` フレームで `initialpose` を送る。 | 高さ補正に `map_aligned` が使われ、TF lookup が成功し、`initialpose3d` の位置が変換後の高さを示す。                                                                  |
| IT_PI_001 | 整列サービスが無い場合のフォールバック挙動を確認。                   | `pose_initializer.launch.py` を `ndt_align_srv` 未起動のまま `launch_testing_ros` から立ち上げ、RViz 相当の `initialpose` と地図点群を供給する。 | `ndt_align_srv` が無い状態で `initialpose` をパブリッシュ。       | ログに "NDT align service is unavailable" が出力され、高さ補正のみの姿勢がパブリッシュされる。ノードはクラッシュせず、後からサービスを起動すれば再度整列を試行する。 |

## 起動ファイル (Launch Files)
### `launch/pose_initializer.launch.py`
`component_container_mt` に `pose_initializer::PoseInitializer` をロードするコンポーネント用 launch。以下のリマップが既定で適用されています。
- `initialpose` ↔ `/initialpose`
- `initialpose3d` ↔ `/initialpose3d`
- `gnss_pose_cov` ↔ `/sensing/gnss/pose_with_covariance`
- `pointcloud_map` ↔ `/map/pointcloud_map`
- `ndt_align_srv` ↔ `/localization/pose_estimator/ndt_align_srv`

使用例:
```bash
ros2 launch pose_initializer pose_initializer.launch.py
```

### `launch/pose_initializer.launch.xml`
同じパラメータ・リマップを備えたスタンドアロンノード用 launch。単体ノードで動かしたい場合に使用します。

使用例:
```bash
ros2 launch pose_initializer pose_initializer.launch.xml
```

## 設定 (Configuration)
- 既定パラメータは `params/pose_initializer.param.yaml` にあり、launch や ROS 2 Parameter API で上書き可能です。
- `pointcloud_map` の `frame_id` が届くたびに `map_frame` を自動で更新します (TF で入力フレームと接続されている前提)。
- 共分散のヒューリスティクス:
   - 手動 `initialpose`: 整列前は diag `[2.0, 2.0, 0.01, 0.01, 0.01, 0.3]`。
   - GNSS `initialpose`: 整列前は diag `[1.0, 1.0, 0.01, 0.01, 0.01, 3.14]`。
   - サービス経路: 整列前は diag `[1.0, 1.0, 0.01, 0.01, 0.01, 1.0]`。
   - 整列応答を受け取った後は diag `[1.0, 1.0, 0.01, 0.01, 0.01, 0.2]` に書き換えます。
- 地図検索半径はノード内部で 1 m に固定されています。より細かい高さ推定が必要な場合は地図点群の密度を高めてください。

## 使い方 (Usage)
1. ワークスペースをビルドして環境を読み込みます。
   ```bash
   colcon build --packages-select pose_initializer
   source install/setup.bash
   ```
2. 上述のコンポーネント or XML launch を起動します。
3. 入力を与えます。
   - `pointcloud_map` をラッチ付きでパブリッシュし、map フレームと入力姿勢フレーム間の TF を用意する。
   - RViz の「2D Pose Estimate」や `/sensing/gnss/pose_with_covariance` へのパブリッシュで初期化を行う。
   - 任意: `ros2 service call /pose_initializer_srv autoware_localization_srvs/srv/PoseWithCovarianceStamped` でサービス経路を利用する。
4. `/initialpose3d` もしくはサービス応答から出力を取得します。

## トラブルシューティング (Troubleshooting)
- **`initialpose3d` が出ない**: `transient_local` QoS で `pointcloud_map` を配信したか、`map_frame` と入力フレームの TF が存在するかを確認。欠如すると TF WARN が記録されます。
- **整列が成功しない**: `/localization/pose_estimator/ndt_align_srv` が起動しているか確認。ノードは 1 Hz で WARN を出しつつ高さ補正結果にフォールバックします。
- **GNSS が繰り返し初期化してしまう**: `use_first_gnss_topic` を false にするか、手動初期化を速やかに行って GNSS 購読が停止する前に姿勢を確定させてください。
- **サービスが応答しない**: `pose_initializer_srv` も内部で同じ align クライアントを使います。`Timed out while waiting for NDT align service response` のログがないか確認し、整列サーバが 3 s 以内に応答することを確かめます。
