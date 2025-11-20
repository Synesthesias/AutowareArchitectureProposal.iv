# autoware_localization_srvs

Autoware のローカライゼーション機能向けサービスインタフェース（ROS 2 Humble）。

- リポジトリ: `AutowareArchitectureProposal.iv`
- パッケージ: `autoware_localization_srvs`
- ライセンス: Apache-2.0

## 概要
`autoware_localization_srvs` は、ローカライゼーション系ノード間で使用されるサービス定義を提供します。姿勢候補の選択や、共分散付き姿勢の伝搬/更新といった用途を想定しています。本パッケージはランタイムノードを含まず、`.srv` ファイルとそこから生成される型のみを提供します。

### 提供サービス
- `autoware_localization_srvs/srv/PoseArray`
  - リクエスト: `geometry_msgs/PoseArray pose_array`
  - レスポンス: `geometry_msgs/PoseStamped pose`
  - 典型的用途: GNSS/ICP などから得られた複数の姿勢候補の中から選択した姿勢を返す。

- `autoware_localization_srvs/srv/PoseWithCovarianceStamped`
  - リクエスト: `geometry_msgs/PoseWithCovarianceStamped pose_with_cov`
  - レスポンス: `geometry_msgs/PoseWithCovarianceStamped pose_with_cov`
  - 典型的用途: 共分散を保持したまま姿勢をエコー/調整（例: EKF の更新やタイムスタンプ付け）。

## ビルドとインストール
`ament_cmake` と `rosidl_default_generators` を用いた標準的なインタフェースパッケージです。

```bash
# ワークスペースのルートから
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 使い方
本パッケージはサービス定義のみを提供します。下流パッケージから依存し、生成された型をノード内で使用してください。

### 下流パッケージの CMake 設定
```cmake
find_package(autoware_localization_srvs REQUIRED)
ament_target_dependencies(your_target autoware_localization_srvs)
```

### 下流パッケージの package.xml
```xml
<depend>autoware_localization_srvs</depend>
```

### 例: 最小サービスサーバ（C++）
```cpp
#include <rclcpp/rclcpp.hpp>
#include <autoware_localization_srvs/srv/pose_array.hpp>

using PoseArray = autoware_localization_srvs::srv::PoseArray;

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pose_selector_server");
  auto srv = node->create_service<PoseArray>(
    "pose_selector",
    [](const std::shared_ptr<PoseArray::Request> req,
       std::shared_ptr<PoseArray::Response> res) {
      if (!req->pose_array.poses.empty()) {
        res->pose.header = req->pose_array.header;
        res->pose.pose = req->pose_array.poses.front();
        res->pose.header.stamp = rclcpp::Clock().now();
      } else {
        res->pose.header.stamp = rclcpp::Clock().now();
      }
    });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

### 例: 最小サービスクライアント（CLI）
サービスサーバが起動している状態で、ROS 2 CLI から呼び出せます。

```bash
# PoseArray サービスの呼び出し例
ros2 interface show autoware_localization_srvs/srv/PoseArray

ros2 service call /pose_selector autoware_localization_srvs/srv/PoseArray "{pose_array: {header: {frame_id: 'map'}, poses: [{position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}]}}"
```

```bash
# PoseWithCovarianceStamped のエコー例
ros2 interface show autoware_localization_srvs/srv/PoseWithCovarianceStamped

ros2 service call /pose_cov_echo autoware_localization_srvs/srv/PoseWithCovarianceStamped "{pose_with_cov: {header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]}}}"
```

## インタフェース
`.srv` ファイルは `share/${PROJECT_NAME}/srv` にインストールされ、以下のコマンドで内容を確認できます。

```bash
ros2 interface list | grep autoware_localization_srvs
ros2 interface show autoware_localization_srvs/srv/PoseArray
ros2 interface show autoware_localization_srvs/srv/PoseWithCovarianceStamped
```

## ノード
本パッケージはノードを提供しません。

## パラメータ
本パッケージはパラメータを定義しません。

## テスト
詳細は `TEST_SPEC.md` を参照してください。

### 簡易ガイダンス
- 単体テスト: `gtest` を用いて、リクエスト/レスポンス型の生成・シリアライズ/デシリアライズ・デフォルト値・代入の検証を行います。
- 結合テスト: `launch_testing` も活用し、最小のサーバ/クライアントで往復・同時実行・タイムアウトなどを確認します。

テストの実行（ワークスペースルートから）:
```bash
colcon test --event-handlers console_cohesion+ --pytest-args -q
```

## 設計の考え方
- `PoseArray` は複数の仮説からの姿勢候補を選択する用途を想定しています。
- `PoseWithCovarianceStamped` は共分散を保持した姿勢の更新/エコー（タイムスタンプ/フレーム調整）に用います。
- インタフェースを専用パッケージに分離することで、実装パッケージと定義を疎結合に保ちます。

## 要件
- ROS 2 Humble（rclcpp, rosidl, geometry_msgs, std_msgs）

## 既知の制限
- サービスを使用するサーバの具体的な挙動（どの姿勢を選ぶか等）は本パッケージの範囲外です。

## コントリビューション
歓迎します。Autoware のスタイルガイドに従ってください。`.srv` 定義を変更する場合は、テスト/仕様も更新し、マイナーバージョンを更新してください。

## ライセンス
Apache-2.0
