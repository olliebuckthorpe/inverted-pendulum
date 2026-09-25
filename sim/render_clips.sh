#!/bin/sh
# Render every double-pendulum transition + the tours into the project's media/clips/ (not in git).
# Usage: ./render_clips.sh [date prefix, default today]
cd "$(dirname "$0")"
D=${1:-$(date +%Y-%m-%d)}
CLIPS="$(git rev-parse --path-format=absolute --git-common-dir)/../media/clips"
mkdir -p "$CLIPS"
for pair in DD,UU DD,UD DD,DU UU,UD UU,DU UU,DD UD,UU UD,DU UD,DD DU,UU DU,UD DU,DD; do
  uv run python watch.py tour --legs "$pair" --seconds 1 --video > /dev/null 2>&1 &&
    mv "videos/tour-$(echo $pair | tr , -)-seed0.mp4" "$CLIPS/$D-sim-transition-$(echo $pair | tr , -).mp4" &&
    echo "ok $pair" || echo "FAILED $pair"
done
for legs in DD,UU,UD,DU,DD DD,UU,DU,UD,UU,DD; do
  uv run python watch.py tour --legs "$legs" --seconds 1 --video > /dev/null 2>&1 &&
    mv "videos/tour-$(echo $legs | tr , -)-seed0.mp4" "$CLIPS/$D-sim-tour-$(echo $legs | tr , -).mp4" &&
    echo "ok tour $legs" || echo "FAILED tour $legs"
done
