# DragRACE — カメラ黒線追従ドラッグレース

自動車整備専門学校 自動運転実習用のプログラム一式です。
全員が同じプログラムを使います。タイムの差は、ハードウェアの整備とセッティングの判断から生まれます。

## ファイル構成

| ファイル | 役割 |
|---|---|
| `rc_race.py` | 本番走行プログラム |
| `camera_test.py` | カメラの確認・調整 |
| `servo_test.py` | ステアリングの中立出し・切れ角確認 |
| `throttle_test.py` | スロットル（ESC）の学習 |
| `kyotsu_settei.json` | 共通設定（全車共通・教員が管理） |
| `kuruma_settei_sample.json` | 車ごとの設定の見本 |
| `docs/` | 説明書・調整マニュアル |

車ごとの設定 `kuruma_settei.json` と走行記録 `logs/` は、各車の中だけにあり、ここには含まれません。

## はじめて使うとき（ラズパイで）

```bash
cd ~
git clone https://github.com/RC-DragRACE/DragRACE.git
cd DragRACE
```

走行セッティングの見本を、自分の車の設定ファイルとしてコピーします。

```bash
cp kuruma_settei_sample.json kuruma_settei.json
```

その後、次の順で調整します。調整の結果（学習値）は `kuruma_settei.json` に自動で書き込まれます。

1. `python3 camera_test.py` — カメラの向きとライン検出を確認
2. `python3 servo_test.py` — ステアリングの中立を出して保存
3. `python3 throttle_test.py` — スロットルを学習して保存
4. `python3 rc_race.py` — 走行（2と3が済んでいない車は、安全のため走行開始できません）

いずれもブラウザで `http://<車のIPアドレス>:8080` を開いて操作します。

**ESCの電源は、`throttle_test.py` や `rc_race.py` を起動したあとに入れてください。** ESCは電源を入れた瞬間の信号を「停止位置」として覚えます。順番が逆だと、学習した値がずれます。
詳しくは `docs/` の各説明書を読んでください。

## 最新版に更新するとき

```bash
cd ~/DragRACE
git pull
```

自分の車の設定（`kuruma_settei.json`）と走行記録（`logs/`）は、更新しても消えません。

## 注意

- プログラム（`.py`）は書き換えないこと。走りを変えたいときは設定ファイルで調整します
- レース前の車検で `git status` を確認します。プログラムが書き換えられていないことがルールです
