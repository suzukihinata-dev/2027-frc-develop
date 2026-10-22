# 2027-frc-develop

FRC チーム 9494 の開発リポジトリ。WPILib 2026 + YAGSL によるスウェーブドライブの雛形です。

## 構成

```
src/main/java/frc/robot/                  ロボットコード（最小構成）
  Robot.java / RobotContainer.java        Command-Based の標準構成
  subsystems/swervedrive/SwerveSubsystem.java  YAGSL でスウェーブを構築するサブシステム
src/main/deploy/swerve/base/              ★スウェーブのハードウェア設定（YAGSL JSON）
  swervedrive.json                        ジャイロとモジュール一覧
  modules/frontleft.json など             各モジュールのモーターID・エンコーダー・位置
  modules/physicalproperties.json         ギア比・ホイール径・電流制限
  modules/pidfproperties.json             PID / FF ゲイン（実機）
  modules/pidfproperties_sim.json         PID / FF ゲイン（シミュレーター）
vendordeps/                               使用ライブラリ（YAGSL 2026.10.03 など）
```

## ロボットに合わせて編集する場所

基本的には `src/main/deploy/swerve/base/` 配下の JSON を書き換えるだけです。

- モーター / エンコーダーの種類と ID → `modules/*.json`
- ジャイロの種類と ID → `swervedrive.json`
- ギア比・ホイール径 → `modules/physicalproperties.json`
- ゲイン → `modules/pidfproperties.json`

設定ジェネレーター: https://config.yagsl.com /
ドキュメント: https://docs.yagsl.com

## ビルド / デプロイ

WPILib VS Code 拡張機能、または:

```bash
./gradlew build        # ビルド
./gradlew deploy       # roboRIO へデプロイ（チーム番号 9494）
./gradlew simulateJava # シミュレーターで起動
```
