# pose2twist

## 概要 (Overview)
`pose2twist` は継続的に受信する `PoseStamped` の差分から時系列で速度情報 (`TwistStamped`) を推定し，下流のローカライゼーションや制御コンポーネントへ出力する．位置の差分を経過時間で割った値を並進速度（ここでは大きさを `linear.x` に格納）とし，ロール・ピッチ・ヨー角の差分を正規化した上で経過時間で割ることで角速度を算出する

## 特長 (Key Features)
- 最初の 1 メッセージ受信時は計算不能なため速度0を出力し，その姿勢をキャッシュして次回の計算に利用する
- 角度差分を [-pi, pi] に正規化し，ヨー角などが ±pi を跨ぐ場合でも不連続で突飛な角度差が出ないようにする
- `geometry_msgs/msg/TwistStamped` と，スカラー値である`linear_x` / `angular_z` を同時に出力する
- スタンドアロン実行（ノード単体）またはコンポーネントコンテナへのロードの両方に対応

## ノード (Nodes)
### `pose2twist`
概要を参照

#### 購読トピック (Subscribed Topics)
- `pose` (`geometry_msgs/msg/PoseStamped`): 入力となる推定姿勢ストリーム

#### 公開トピック (Published Topics)
- `twist` (`geometry_msgs/msg/TwistStamped`): 最新の入力姿勢ヘッダとパラメータで指定したフレーム ID を付与した推定ツイスト
- `linear_x` (`std_msgs/msg/Float32`): 並進速度（今回の実装では位置差分距離/時間）
- `angular_z` (`std_msgs/msg/Float32`): ヨー角速度（角度差分/時間）

#### パラメータ (Parameters)
| 名前 (Name)      | 型 (Type) | 既定値 (Default) | 説明 (Description)                                        |
| ---------------- | --------- | ---------------- | --------------------------------------------------------- |
| `twist_frame_id` | string    | `base_link`      | 出力する `TwistStamped` とスカラー速度トピックの frame_id |

## 起動ファイル (Launch Files)
- `launch/pose2twist.launch.py`: コンポーネントコンテナ上でノードを起動
  ```bash
  ros2 launch pose2twist pose2twist.launch.py \
    input_pose_topic:=/localization/pose_estimator/pose \
    output_twist_topic:=/estimate_twist
  ```
- `launch/pose2twist.launch.xml`: スタンドアロン実行用

## 設定 (Configuration)
- 既定パラメータは `params/pose2twist.param.yaml` に定義
- `twist_frame_id` をスタック内で利用する座標系（例: `odom`, `base_link` など）に合わせて変更

## テスト (Testing)
- 単体テストおよび結合テストの仕様は `TEST_SPEC.md` に記載
- 単体テストは gtest を用いて `pose2twist/test/` 以下に実装し，
- 結合テストは `launch_testing` を組み合わせて実行
```bash
colcon test --packages-select pose2twist
```

## トラブルシューティング (Troubleshooting)
- 受信する `PoseStamped` のタイムスタンプが単調増加しているか確認
- 非正の時間差 (dt <= 0) の場合はゼロ速度を出力し警告をスロットル表示
- クォータニオンが正規化済みか確認
- 異常な姿勢（非正規化クォータニオン）は角速度計算の不整合を引き起こす
