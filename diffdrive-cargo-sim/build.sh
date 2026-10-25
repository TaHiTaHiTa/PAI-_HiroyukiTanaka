#!/usr/bin/env bash
# 搬送タスク動画のビルド（シミュレーション → 2並列レンダリング → 結合）
set -euo pipefail
cd "$(dirname "$0")"
WORK=build; mkdir -p "$WORK"
python3 robot_task.py sim "$WORK/sim.pkl"
N=$(python3 -c "import pickle;print(len(pickle.load(open('$WORK/sim.pkl','rb'))[0]))")
H=$((N/2))
python3 robot_task.py render "$WORK/sim.pkl" 0 "$H" "$WORK/part0.mp4" &
python3 robot_task.py render "$WORK/sim.pkl" "$H" "$N" "$WORK/part1.mp4" &
wait
printf "file 'part0.mp4'\nfile 'part1.mp4'\n" > "$WORK/list.txt"
ffmpeg -y -loglevel error -f concat -safe 0 -i "$WORK/list.txt" -c copy robot_cargo_task.mp4
echo "done: robot_cargo_task.mp4"
