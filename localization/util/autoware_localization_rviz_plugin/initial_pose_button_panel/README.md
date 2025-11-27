# initial_pose_button_panel

## 概要 (Overview)
`initial_pose_button_panel` は RViz のパネルとして提供される GUI プラグインで、GNSS などから配信される `geometry_msgs/msg/PoseWithCovarianceStamped` を監視し、最新の推定姿勢を `/localization/util/pose_initializer_srv` に送る初期化ボタンを提供します。オペレータは RViz 上のボタンを押すだけで Pose Initializer サービスへ最新値を転送でき、再ローカライズ時の手順を簡素化できます。

### 特長 (Key Features)
- パネル上で購読トピックを自由に変更し、最新 Pose を常にキャッシュ。
- センサーデータ QoS での購読と GUI 状態連動により、無効な Pose を送信しない安全策を実装。
- `/localization/util/pose_initializer_srv` に対するサービス呼び出しと進捗表示を 1 ボタンに集約。
- サービス不在やタイムアウトを色付きステータスで即時通知。
- RViz のパネル管理に準拠した pluginlib エクスポートで、Autoware の既存レイアウトへ容易に組み込み可能。

## ノード (Nodes)
### autoware_localization_rviz_plugin::InitialPoseButtonPanel
- **型**: `rviz_common::Panel` を継承した GUI プラグイン。`rviz2` の [Panels] から追加可能。
- **説明**: 内部で `rclcpp::Node` を取得し、Pose 購読と Pose Initializer サービスクライアントを管理する。GUI 操作は Qt Widgets で構成され、ROS コールバックとはスレッドセーフにやり取りする。

#### 購読トピック (Subscribed Topics)
| 名称                                             | 型                                            | 説明                                                                                                  |
| ------------------------------------------------ | --------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| 任意 (既定 `/sensing/gnss/pose_with_covariance`) | `geometry_msgs/msg/PoseWithCovarianceStamped` | GNSS や外部ローカライザからの最新推定 Pose。センサーデータ QoS で購読し、受信後にボタンを有効化する。 |

#### サービスクライアント (Service Clients)
| 名称                                      | 型                                                         | 説明                                                                                                |
| ----------------------------------------- | ---------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| `/localization/util/pose_initializer_srv` | `autoware_localization_srvs/srv/PoseWithCovarianceStamped` | キャッシュした Pose を Pose Initializer ノードへ送信して再ローカライズを開始する。待機時間は 5 秒。 |

#### GUI 入力 (GUI Inputs)
| 名称                 | 説明                                                                    |
| -------------------- | ----------------------------------------------------------------------- |
| `topic_edit_`        | 監視対象の Pose トピック名を直接入力。値変更時に再購読する。            |
| `initialize_button_` | 最新 Pose をサービスへ転送するボタン。Pose が未受信の間は無効化される。 |

### 機能とテスト対応 (Feature-to-Test Traceability)
各機能は README に記載された外部仕様と 1:1 で対応しており、詳細な試験手順は `TEST_SPEC.md` を参照してください。

| 機能 ID | 機能説明                                                                                   | テスト ID   | テストで検証する観点                                                                              |
| ------- | ------------------------------------------------------------------------------------------ | ----------- | ------------------------------------------------------------------------------------------------- |
| F-001   | 指定した `PoseWithCovariance` トピックを購読し、最初の受信までボタンを無効化する。         | UT_IPBP_001 | QoS セットアップ後に Pose を publish するとボタンが有効化され、受信前は無効のままである。         |
| F-002   | 最新 Pose をミューテックス付きでキャッシュし、リクエスト生成時に読み出す。                 | UT_IPBP_002 | 異なる Pose を 2 回受信させるとリクエストに常に最後の値が入ること。                               |
| F-003   | 初期化ボタン押下で Pose Initializer サービスへ送信し、成功時にステータスを緑色に更新する。 | UT_IPBP_003 | モックサービスが成功応答を返すとステータスラベルが "OK!!!" へ遷移する。                           |
| F-004   | サービスが 5 秒以内に利用不可なら赤色ステータスと再有効化でフェイルセーフする。            | IT_IPBP_001 | サービスを起動しない環境でボタンを押すと "Service Unavailable" 表示とボタン再有効化が行われる。   |
| F-005   | トピック名を編集すると再購読し、新しいトピックの Pose を即座に使用する。                   | IT_IPBP_002 | GUI 上でトピックを書き換えた後、別トピックから publish するとそのデータでサービス要求が送られる。 |

## 起動ファイル (Launch Files)
本パッケージは RViz パネルとして提供されるため専用の launch ファイルはありません。`rviz2` のレイアウトファイル (`.rviz`) に本パネルを追加して保存すると次回以降は自動読み込みされます。Autoware の RViz 設定に組み込む場合は `Panels -> Add New Panel -> autoware_localization_rviz_plugin/InitialPoseButtonPanel` を選択してください。

## 設定 (Configuration)
- 既定購読トピックは `src/.../initial_pose_button_panel/src/initial_pose_button_panel.cpp` の `kDefaultTopic` 定数で管理されています。リビルドなしで変えたい場合はパネル上のテキストボックスから変更してください。
- サービス名 `kInitializerService` は `/localization/util/pose_initializer_srv` 固定です。別ノードと連携する場合はソースを変更し、`colcon build --packages-select initial_pose_button_panel` で再ビルドします。
- GUI は Qt Widgets で構成されるため、Wayland 環境では `QT_QPA_PLATFORM=xcb` を指定すると安定します。

## 使用方法 (Usage)
1. 依存をビルド:
   ```bash
   colcon build --packages-select initial_pose_button_panel
   source install/setup.bash
   ```
2. RViz を起動し、[Panels] -> [Add New Panel] から `autoware_localization_rviz_plugin/InitialPoseButtonPanel` を追加。
3. `PoseWithCovariance` を出力しているトピック名を入力 (既定の GNSS トピックを流用していれば編集不要)。
4. Pose が購読されボタンが "Pose Initializer   Let's GO!" へ変わったらクリックでサービス呼び出し。
5. ステータスラベルが緑色 "OK!!!" になれば Pose Initializer が受理したことを示します。

## テスト (Testing)
- 試験観点は `TEST_SPEC.md` に明記されています。
- 単体テストは `gtest`、結合テストは `launch_testing_ros` を想定しています。
- 実行例:
  ```bash
  colcon test --packages-select initial_pose_button_panel
  ```

## トラブルシューティング (Troubleshooting)
- ボタンが永遠に有効化されない場合は入力トピック名と QoS を確認し、`ros2 topic echo` で Pose が届いているか検証してください。
- "Service Unavailable" が出る場合は Pose Initializer ノードが起動しているか、`ros2 service list` で `/localization/util/pose_initializer_srv` が存在するか確認してください。
- RViz が Qt プラグイン読み込みエラーを起こす場合は `sudo apt install qtbase5-dev` が入っているか再確認し、`rviz2` を再起動してください。
