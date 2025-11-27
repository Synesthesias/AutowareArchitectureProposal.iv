# geo_pos_conv

## 概要 (Overview)
- GNSSから得られる測地系の緯度/経度/高さ（LLH）を日本測地系 2011（JGD2011）の平面直角座標へ投影する C++ ライブラリ
- GNSS 測位値を地図フレームへ変換する際に `gnss_poser` 等のTF生成ノードからライブラリとして利用されます

## 特長 (Key Features)
- 19 個の JGD2011 平面ゾーンをプリセットし、手動原点による上書きもサポート
- 10 進度入力と NMEA 度入力の両方をメートル単位の x/y/z へ変換
- 直近の LLH とデカルト値を保持し、下流モジュールがアクセサを介して即座に参照可能
- GeographicLib を利用する下流ノードと組み合わせて map フレームとの整合性を担保

## ライブラリ (Library)
### geo_pos_conv クラス
- **ヘッダ**: `#include <geo_pos_conv/geo_pos_conv.hpp>`
- **役割**: 平面ゾーンの設定、LLH からデカルト座標への前方変換、テスト用の手動座標設定を提供。

| メソッド                                                             | 説明                                                                                             |
| -------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------ |
| `set_plane(int plane_id)`                                            | 国土地理院が定義する平面直角座標系のゾーン（1-19）を選択。基準緯度・経度を内部テーブルから取得。 |
| `set_plane(double lat_rad, double lon_rad)`                          | ラジアンで手動原点を設定し、プリセットに依存しない測域へ対応。                                   |
| `llh_to_xyz(double lat_deg, double lon_deg, double h_m)`             | 10 進度の LLH をメートル単位の x/y/z（北・東・高さ）へ投影。                                     |
| `set_llh_nmea_degrees(double lat_ddmm, double lon_ddmm, double h_m)` | NMEA（ddmm.mmmm）入力をパースして上記と同一の座標変換を実行。                                    |
| `set_xyz(double x_m, double y_m, double z_m)`                        | 結合テストやリセット用途でデカルト座標を直接与え、アクセサ経由で公開。                           |
| `x()`, `y()`, `z()`                                                  | 直近に計算または設定されたデカルト成分を返す。                                                   |

> 注意: `conv_xyz2llh()` はダミー実装で逆変換は行なわないため、必要に応じて GeographicLib 等を併用する必要があります

## 機能とテスト対応 (Feature-to-Test Traceability)
各機能は外部仕様として公開するものに限定し、テストケースと1:1対応で記載しています。詳細な手順は `TEST_SPEC.md` を参照してください。

| 機能 ID | 機能説明                                                                                      | テスト ID  | テストで検証する観点                                                            |
| ------- | --------------------------------------------------------------------------------------------- | ---------- | ------------------------------------------------------------------------------- |
| F-001   | プリセットの平面番号から正しい原点を選び、LLH→XYZ で国土地理院基準値に一致させる。            | UT_GPC_001 | `set_plane(9)` と `llh_to_xyz` の結果がベンチマークと ±1 mm で一致する。        |
| F-002   | 手動ラジアン原点指定でもプリセットと同一の結果を返す。                                        | UT_GPC_002 | `set_plane(lat, lon)` の出力が F-001 と誤差範囲内で一致する。                   |
| F-003   | NMEA 度入力をパースし、10 進度経路と同一の XYZ を得る。                                       | UT_GPC_003 | `set_llh_nmea_degrees` の出力が F-001 と ±1 mm 以内。                           |
| F-004   | 高度をそのまま z に反映し、平面射影と独立させる。                                             | UT_GPC_004 | 高度のみ変更した入力でも x/y は不変で z が入力値と一致。                        |
| F-005   | 手動で設定したデカルト値をアクセサからそのまま取得できる。                                    | UT_GPC_005 | `set_xyz` 後に x/y/z アクセサが設定値を返しクラッシュしない。                   |
| F-006   | `gnss_poser::NavSatFix2PLANE` で map 座標へ変換しても XY が一致し、ジオイド補正が適用される。 | IT_GPC_001 | 出力 `plane.x/y` が F-001 の結果と一致し、`plane.z` が EGM2008 の補正値になる。 |
| F-007   | GeographicLib データ欠如などの異常時に高度を保持しつつエラーログで通知する。                  | IT_GPC_002 | 例外発生時に `plane.z` が入力高度を保持し、`ERROR` ログを 1 回出力。            |
| F-008   | `map_tf_generator` など下流 TF 利用者と座標整合を保つ。                                       | IT_GPC_003 | 生成された TF が期待オフセットと ±5 cm 以内で一致し、一定時間安定する。         |

## 使用方法 (Usage)
1. 依存関係をビルドしてセットアップします。
   ```bash
   colcon build --packages-select geo_pos_conv
   source install/setup.bash
   ```
2. C++ からクラスを利用します。
   ```cpp
   #include <geo_pos_conv/geo_pos_conv.hpp>

   geo_pos_conv conv;
   conv.set_plane(9);  // 東京エリア
   conv.llh_to_xyz(35.681236, 139.767125, 44.0);

   const double northing = conv.x();
   const double easting = conv.y();
   const double height = conv.z();
   ```
3. NMEA 度を扱う場合は `set_llh_nmea_degrees` を呼び出し、`gnss_poser` や TF ノードと同じ平面番号を使用してください。

## テスト (Testing)
`TEST_SPEC.md` に詳細な手順を記載しています。gtest と launch_testing を利用する標準的な実行コマンド:

```bash
colcon test --packages-select geo_pos_conv
```

## トラブルシューティング
- **x/y が常に 0**: `llh_to_xyz` または `set_llh_nmea_degrees` の前に必ず `set_plane` を呼んでください。
- **位置がジャンプする**: すべてのノードで同じプレーン ID を共有し、入力の単位（10 進度 or NMEA 度）を確認してください。
- **ジオイド補正が失敗**: `gnss_poser` 連携時は `GEOGRAPHICLIB_DATA` を正しく設定し、EGM2008 データを展開してください。
