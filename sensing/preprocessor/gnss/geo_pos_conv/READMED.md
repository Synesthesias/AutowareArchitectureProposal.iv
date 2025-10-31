# geo_pos_conv パッケージガイド

## 概要
`geo_pos_conv` は測地系の緯度/経度/高さ（LLH）を，日本の平面直角座標系（JGD2011）に整列した平面 x/y 座標へ変換するライブラリです。
GNSS の測位値を地図フレームへ変換し，自己位置推定やセンサフュージョンの前処理として利用されます。

## 機能
- JGD2011 の 19 平面ゾーンに対応した原点プリセットと手動原点の上書き
- 10 進度または NMEA 形式の度をメートル単位の x/y に変換
- 直近の LLH とデカルト状態（x/y/z）を保持し，下流コンポーネントが再利用可能

## 対応プラットフォーム
ROS 2 Humble（Ubuntu 22.04）と C++14 をサポートするシステムツールチェーンを対象とします。

## 使い方
1. ヘッダをインクルードし，コンバータを生成します：
   ```cpp
   #include <geo_pos_conv/geo_pos_conv.hpp>

   geo_pos_conv converter;
   converter.set_plane(9);  // 東京エリア
   converter.llh_to_xyz(35.681236, 139.767125, 44.0);
   const double east_m = converter.y();
   const double north_m = converter.x();
   ```
2. NMEA 形式の度は `set_llh_nmea_degrees` で入力できます。内部でラジアンへ変換したのち主投影を実行します。
3. `gnss_poser` と組み合わせる際は，同一のプレーン番号を使用して，地図座標系との整合を保証してください。

## API 概要
| メソッド                                                             | 説明                                                               |
| -------------------------------------------------------------------- | ------------------------------------------------------------------ |
| `set_plane(int plane_id)`                                            | 日本の平面直角座標系の定義済みゾーン（1-19）を選択します。         |
| `set_plane(double lat_rad, double lon_rad)`                          | 原点をラジアン指定で手動設定します。                               |
| `llh_to_xyz(double lat_deg, double lon_deg, double h_m)`             | 10 進度の緯度・経度と高度（m）をローカルデカルト座標へ変換します。 |
| `set_llh_nmea_degrees(double lat_ddmm, double lon_ddmm, double h_m)` | NMEA 形式の度（ddmm.mmmm）をパースしてから投影します。             |
| `set_xyz(double x_m, double y_m, double z_m)`                        | 主に結合試験向けにデカルト状態を手動設定します。                   |
| `x()`，`y()`，`z()`                                                  | 投影済み（または手動設定）デカルト成分のアクセサです。             |

> 注意：`conv_xyz2llh()` は現行のコードベースでは未実装です。逆変換が必要な場合は GeographicLib などのライブラリを利用してください。

## 連携ポイント
- **gnss_poser**：`sensor_msgs::msg::NavSatFix` を `NavSatFix2PLANE` で地図フレームへ変換し，内部で `geo_pos_conv` を利用します。
- **map_tf_generator**：平面座標を使用して map と車両フレーム間の TF を公開します。
- **ローカリゼーション系**：`geo_pos_conv` の出力を消費する全モジュールでプレーン原点の一貫性が必要です。

## テスト
テストは gtest を前提としています。詳細は `TEST_SPEC.md` を参照してください。実装後の実行例：

```bash
colcon test --packages-select geo_pos_conv
```

## トラブルシューティング
- **常に 0 が出力される**：LLH 変換の前に `set_plane` を呼んでください。原点未設定のままでは原点（0,0,0）からの変化が出ません。
- **位置が大きくジャンプする**：すべてのモジュールで同じプレーンゾーンを使用しているか，また入力の単位（10 進度 vs NMEA 度）が期待通りか確認してください。
- **ジオイドデータのエラー**：`gnss_poser` などの下流パッケージは EGM2008 データを必要とします。結合テスト前に `GEOGRAPHICLIB_DATA` を適切に設定してください。
