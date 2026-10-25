# 差動二輪ロボット 搬送タスク動画 — ソース一式

## ファイル
- `robot_task.py` … シミュレーション・3D描画・HUD・動画書き出しをすべて含む本体
- `build.sh` … 動画を一括ビルドするスクリプト

## 必要環境
- Python 3.10+、`numpy`、`Pillow`（`pip install numpy pillow`）
- `ffmpeg`（libx264 付き）
- フォント: Noto Sans CJK（`/usr/share/fonts/opentype/noto/`）、DejaVu Sans Mono
  - Ubuntu: `sudo apt install fonts-noto-cjk fonts-dejavu-core ffmpeg`
  - 別の場所にある場合は `robot_task.py` の `FD` と `FM` のパスを変更

## 使い方
```bash
./build.sh                                  # robot_cargo_task.mp4 を生成（2コアで約6分）
python3 robot_task.py sim build/sim.pkl     # シミュレーションのみ（衝突余裕も表示）
python3 robot_task.py still build/sim.pkl build 470 1150 3500   # 指定フレームの静止画を確認
python3 robot_task.py render build/sim.pkl 0 3682 out.mp4       # 範囲を指定してレンダリング
```

## 構成
1. **シミュレーション（`Sim`）** … 記事の DiffDrive 設定（車輪半径 0.033 m、車輪間隔 0.19 m）を使った差動二輪の運動学。
   teleop_node と同じキー割り当て（w/s → linear.x ±1.0、a/d → angular.z ±1.0、スペース → 停止）で、最後の指令を保持します。
   速度応答には時定数 0.12 s の一次遅れを入れ、1/480 s 刻みで積分しています。
   `simulate()` が「キーを押す → 指定時間待つ → 停止」を組み合わせて搬送ルートを自動で操縦します。
2. **3D描画（`render3d`）** … 自前の透視投影とペインタ法、平行光源による陰影と影。2倍で描画してから縮小し、アンチエイリアスをかけています。
3. **HUD（`hud`）** … キー入力、/cmd_vel、車輪の角速度、ROS2 風ログ、ミニマップ、タスクの進み具合。
4. **書き出し** … フレームを ffmpeg に流し込み、H.264（1280×720、30fps）で保存します。

## カスタマイズ
- 障害物は `OBST`、荷物の初期位置は `CUBES0`、置き場所は `SLOTS`、倉庫エリアは `ZONE` で変えられます。
- ルートは `simulate()` 内の `goto` / `approach` / `reverse` の並びで決まります。
  ルートを変えたら、`sim` モードで表示される clear 値（最小クリアランス [m]）を確認してください。
  積み込み時に荷物へ寄せる場面ではマイナスになりますが、これは想定どおりです。

## 成果物
- [docs/robot_cargo_task.mp4](docs/robot_cargo_task.mp4) … 生成した動画（1280×720、30fps、約2分3秒）
- [docs/robot_cargo_task_report.pdf](docs/robot_cargo_task_report.pdf) … 動画とコードの技術レポート

## 参考
- @N622「gazeboとROS2を連携してロボットを操作する」Qiita: https://qiita.com/N622/items/9d89f77d85d9da0af29e
  （ロボットの寸法とキー操作の仕様をこの記事に合わせています。ROS2 や Gazebo は使っていません）
