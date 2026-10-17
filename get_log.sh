#!/usr/bin/env bash
# DragRACE の走行ログを Pi から Chromebook(Linux) に取り込み、動画をブラウザで再生できる形式に変換する
#
# 使い方:
#   ./get_log.sh                       最新の1走行を取り込む
#   ./get_log.sh 5                     最新の5走行を取り込む
#   ./get_log.sh run_20261013_162749   指定した走行を取り込む
#
# 取り込み先: ~/DragRACE_logs  （ファイルアプリでは「Linux ファイル > DragRACE_logs」）
# 別の車両から取るとき:  PI=yoshi@yoshi-rc02.local ./get_log.sh
set -uo pipefail

PI="${PI:-yoshi@yoshi-rc01.local}"
REMOTE_DIR="DragRACE/logs"            # Pi のホームからの相対パス
LOCAL_DIR="${LOCAL_DIR:-$HOME/DragRACE_logs}"
mkdir -p "$LOCAL_DIR"

if ! command -v ffmpeg >/dev/null; then
  echo "ffmpeg をインストールします（初回のみ）"
  sudo apt-get update && sudo apt-get install -y ffmpeg || { echo "ffmpeg のインストールに失敗しました"; exit 1; }
fi

arg="${1:-1}"
if [[ "$arg" =~ ^[0-9]+$ ]]; then
  runs=$(ssh "$PI" "cd $REMOTE_DIR && ls -1t run_*.csv 2>/dev/null | head -n $arg | sed 's/\.csv\$//'")
else
  runs="$arg"
fi
if [ -z "$runs" ]; then
  echo "Pi の ~/$REMOTE_DIR に走行ログが見つかりませんでした"; exit 1
fi

for r in $runs; do
  echo "=== $r ==="
  for f in "${r}.csv" "${r}_settei.json"; do
    scp -q "$PI:$REMOTE_DIR/$f" "$LOCAL_DIR/" && echo "  取得: $f" || echo "  見つからない: $f"
  done
  if [ -f "$LOCAL_DIR/${r}.mp4" ] && [ -f "$LOCAL_DIR/.${r}.converted" ]; then
    echo "  動画は変換済み"; continue
  fi
  if scp -q "$PI:$REMOTE_DIR/${r}.mp4" "$LOCAL_DIR/${r}.raw.mp4"; then
    # Pi の録画(mp4v)はブラウザで再生できないので H.264 に変換して同じ名前で保存
    if ffmpeg -v error -y -i "$LOCAL_DIR/${r}.raw.mp4" -c:v libx264 -preset veryfast -crf 23 \
         -pix_fmt yuv420p -movflags +faststart -an "$LOCAL_DIR/${r}.mp4"; then
      rm -f "$LOCAL_DIR/${r}.raw.mp4"; touch "$LOCAL_DIR/.${r}.converted"
      echo "  取得: ${r}.mp4（H.264 に変換済み）"
    else
      echo "  動画の変換に失敗しました（元ファイル: ${r}.raw.mp4）"
    fi
  else
    echo "  見つからない: ${r}.mp4"
  fi
done
echo
echo "完了: ファイルアプリの「Linux ファイル > DragRACE_logs」に入っています"
