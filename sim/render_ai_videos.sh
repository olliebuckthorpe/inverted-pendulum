#!/bin/sh
# When a training run finishes (its final .zip appears), render its timelapse and network animation
# into the project's media/clips/. Usage: nohup ./render_ai_videos.sh balance_seed0 swingup_seed0 &
cd "$(dirname "$0")"
CLIPS="$(git rev-parse --path-format=absolute --git-common-dir)/../media/clips"
for tag in "$@"; do
  until [ -f "checkpoints/$tag.zip" ] && [ -f "results/ai/$tag.json" ]; do sleep 30; done
  D=$(date +%Y-%m-%d)
  uv run python ai_timelapse.py "$tag" 8 "$CLIPS/$D-sim-ai-timelapse-$tag.mp4" && echo "ok timelapse $tag" || echo "FAILED timelapse $tag"
  uv run python ai_network_anim.py "$tag" "$CLIPS/$D-sim-ai-network-$tag.mp4" && echo "ok network $tag" || echo "FAILED network $tag"
done
