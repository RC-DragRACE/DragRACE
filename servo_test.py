#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   servo_test.py — ステアリングサーボ 動作確認・調整プログラム
#
#   自動車整備専門学校 自動運転実習用
#   詳しい手順は「説明書_servo_test.md」を読んでください。
#
# ==============================================================================
#
#  ● このプログラムでやること
#
#      フェーズ1: 組付け後の合否チェック（動く・向きが正しい・唸らない など）
#      フェーズ2: 「中立＝タイヤがまっすぐ」の調整と、車専用設定への保存
#
#  ● 安全について
#
#      このプログラムは ハンドルの線（GPIO13）にしか信号を出しません。
#      アクセルの線（GPIO12）には何も出さないので、モーターは回りません。
#      ただしタイヤが左右に動くので、タイヤ付近に手や物を置かないこと。
#
#  ● 使い方
#
#      1. ラズパイのターミナルで:   python3 servo_test.py
#      2. ブラウザで:  http://<この車のIPアドレス>:8080
#      3. 終わるときはターミナルで Ctrl と C を同時に押す
#
# ==============================================================================

import os
import json
import time
import signal
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# ==============================================================================
# ブロック1: 設定
# ==============================================================================

STEER_GPIO = 13        # ハンドル(サーボ)がつながっているピン番号。変えないこと
PWM_SHUHASU = 70       # 信号の周波数[Hz]。変えないこと

CHURITSU_KIJUN = 10.50 # 中立の基準値[%]。調整はここから始める
                       # サーボ規格の中立はパルス幅1.5ms＝70Hzで10.50%。
                       # この位置がサーボ可動域の中心（左右対称に働く点）
KIREKAKU_HABA = 1.00   # 中立からハンドルを切る幅[%]（左右いっぱいまで）
                       # 全車共通。機械限界に当たって唸る車があれば教員が下げる

# 安全リミット: どんな操作でもこの範囲の外の信号は絶対に出さない
ANZEN_MIN = 7.50
ANZEN_MAX = 13.00

HANDORU_GYAKU = False  # ハンドルの向き。左ボタンで右に切れてしまう車は True
                       # （個別設定項目: 画面の「左右の向きを反転」ボタンで
                       #   kuruma_settei.json に保存され、全プログラムに反映される）

SWEEP_BYO = 3.0        # スイープ(自動で左右1往復)にかける時間[秒]

SETTEI_FILE = "kuruma_settei.json"   # この車専用の設定ファイル
HTTP_PORT = 8080

# ==============================================================================
# 設定ファイルの読み込み（全プログラム共通の仕組み）
#   優先順位: プログラム内の初期値
#             → kyotsu_settei.json（共通設定・教員が全車に配布）
#             → kuruma_settei.json（個体設定・各テストツールが保存）
#   ファイルが無ければ初期値のまま動く。上と同じ名前の項目だけが上書きされる。
# ==============================================================================
KYOTSU_FILE = "kyotsu_settei.json"
_KAKIKAE_KINSHI = {"PWM_SHUHASU", "STEER_GPIO", "THROTTLE_GPIO",
                   "ANZEN_MIN", "ANZEN_MAX"}   # 安全のため上書き禁止

def settei_yomikomi():
    for fname in (KYOTSU_FILE, SETTEI_FILE):
        if not os.path.exists(fname):
            continue
        try:
            with open(fname) as f:
                yomu = json.load(f)
        except Exception as e:
            print(f"[設定] {fname} が読めません ({e})")
            continue
        for key, atai in yomu.items():
            if key in _KAKIKAE_KINSHI:
                continue
            if key in globals() and key.isupper():
                globals()[key] = atai
                print(f"[設定] {fname}: {key} = {atai}")

settei_yomikomi()


# ==============================================================================
# Raspberry Pi 5 用の信号出力（pigpio が使えない Pi 5 のための代わり）
#   Pi 4 では pigpio が使えるので、この仕組みは使われない（動作は変わらない）。
#   Pi 5 では Linux標準のハードウェアPWMを使う。事前に
#   /boot/firmware/config.txt の最後に次の1行を書いて再起動しておくこと:
#     dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4
# ==============================================================================
class Pi5PWM:
    """pigpio と同じ呼び方 hardware_PWM(GPIO番号, 周波数, デューティ[100万分率]) で使える"""
    CHANNEL = {12: 0, 13: 1}          # GPIO12 = チャンネル0、GPIO13 = チャンネル1

    def __init__(self):
        import glob
        self.chip = None
        kouho = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
        for c in kouho:                                 # Pi 5 の GPIO12/13 用PWMを優先
            if "1f00098000" in os.path.realpath(c):
                self.chip = c
                break
        if self.chip is None:
            for c in kouho:
                try:
                    if int(open(c + "/npwm").read()) >= 2:
                        self.chip = c
                        break
                except Exception:
                    pass
        if self.chip is None:
            raise RuntimeError("ハードウェアPWMが見つかりません（config.txt の設定と再起動を確認）")
        if not os.access(self.chip + "/export", os.W_OK):
            raise RuntimeError("PWMを操作する権限がありません（sudo をつけて起動してください）")
        self.connected = True
        self._shuki = {}

    def _kaku(self, path, atai):
        for _ in range(50):                             # 準備直後は書けるまで少し待つ
            try:
                with open(path, "w") as f:
                    f.write(str(atai))
                return
            except (PermissionError, FileNotFoundError):
                time.sleep(0.05)
        raise RuntimeError(f"書き込めません: {path}")

    def hardware_PWM(self, gpio, shuhasu, duty_man):
        ch = self.CHANNEL[gpio]
        d = f"{self.chip}/pwm{ch}"
        if not os.path.exists(d):
            self._kaku(self.chip + "/export", ch)
        if shuhasu == 0:                                # 0 = 信号を止める
            self._kaku(d + "/enable", 0)
            return
        shuki = int(1_000_000_000 / shuhasu)            # 周期[ナノ秒]
        if self._shuki.get(ch) != shuki:
            try:
                self._kaku(d + "/duty_cycle", 0)        # 周期を変える前にデューティを0に
            except Exception:
                pass
            self._kaku(d + "/period", shuki)
            self._shuki[ch] = shuki
        self._kaku(d + "/duty_cycle", int(shuki * duty_man / 1_000_000))
        self._kaku(d + "/enable", 1)

    def stop(self):
        pass

# ==============================================================================
# ブロック2: サーボへの信号出力（pigpioライブラリを使用）
# ==============================================================================

class Servo:
    def __init__(self):
        self.pi = None
        try:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.pi = None
                raise RuntimeError("pigpiod が動いていません")
            print("[サーボ] pigpio に接続しました（実機モード）")
        except Exception as e:
            print(f"[サーボ] pigpio が使えません ({e})")
            try:
                self.pi = Pi5PWM()
                print("[サーボ] Pi 5 のハードウェアPWMで出力します（実機モード）")
            except Exception as e2:
                self.pi = None
                print(f"[サーボ] Pi 5 用の出力も使えません ({e2})")
                print("[サーボ] 画面表示だけのテストモードで動きます")

    def shingou(self, duty):
        """duty[%] の信号を出す。安全リミットの外は絶対に出さない"""
        duty = max(ANZEN_MIN, min(ANZEN_MAX, duty))
        if self.pi is not None:
            # pigpio では 100% = 1,000,000 なので、% を 10000倍して渡す
            self.pi.hardware_PWM(STEER_GPIO, PWM_SHUHASU, int(duty * 10000))
        return duty

    def teishi(self):
        """信号を止める（サーボは力が抜けた状態になる）"""
        if self.pi is not None:
            self.pi.hardware_PWM(STEER_GPIO, 0, 0)
            self.pi.stop()

# ==============================================================================
# ブロック3: 調整のまとめ役
# ==============================================================================

class App:
    def __init__(self):
        self.lock = threading.Lock()
        self.servo = Servo()

        # 設定ファイルがあれば、保存済みの中立値を読み込む
        self.churitsu = CHURITSU_KIJUN
        if os.path.exists(SETTEI_FILE):
            try:
                with open(SETTEI_FILE) as f:
                    settei = json.load(f)
                self.churitsu = float(settei.get("STEER_CHURITSU", CHURITSU_KIJUN))
                print(f"[設定] {SETTEI_FILE} から中立値 {self.churitsu:.2f}% を読み込みました")
            except Exception as e:
                print(f"[設定] 設定ファイルが読めません ({e})。基準値を使います")

        self.gyaku = bool(HANDORU_GYAKU)   # ハンドルの向き（True=反転）
        self.ichi = "chuuritsu"       # いまのハンドル位置
        self.sweep_chu = False        # スイープ動作中フラグ
        self.ima_duty = self.servo.shingou(self.churitsu)  # 起動時は必ず中立
        print(f"[サーボ] 起動: 中立 {self.churitsu:.2f}% を出力中")

    # ---- 左右の向き（HANDORU_GYAKU 対応） ----
    def _hidari_duty(self):
        muki = -1 if self.gyaku else 1
        return self.churitsu + KIREKAKU_HABA * muki

    def _migi_duty(self):
        muki = -1 if self.gyaku else 1
        return self.churitsu - KIREKAKU_HABA * muki

    # ---- ボタン操作 ----
    def ugokasu(self, ichi):
        with self.lock:
            if self.sweep_chu:
                return
            if ichi == "hidari":
                duty = self._hidari_duty()
            elif ichi == "migi":
                duty = self._migi_duty()
            else:
                ichi = "chuuritsu"
                duty = self.churitsu
            self.ichi = ichi
            self.ima_duty = self.servo.shingou(duty)
            print(f"[操作] {ichi}: {self.ima_duty:.2f}%")

    def shitei(self, duty):
        """任意の%を直接出す（実測実習用）。切れ角の範囲内に制限"""
        with self.lock:
            if self.sweep_chu:
                return
            saitei = self.churitsu - KIREKAKU_HABA
            saidai = self.churitsu + KIREKAKU_HABA
            duty = max(saitei, min(saidai, duty))
            self.ichi = "shitei"
            self.ima_duty = self.servo.shingou(duty)
            print(f"[指定] {self.ima_duty:.2f}%")

    def sweep(self):
        """左いっぱい→右いっぱい→中立 を1回だけゆっくり往復する"""
        with self.lock:
            if self.sweep_chu:
                return
            self.sweep_chu = True
            self.ichi = "sweep"
        threading.Thread(target=self._sweep_ugoki, daemon=True).start()

    def _sweep_ugoki(self):
        try:
            hidari, migi = self._hidari_duty(), self._migi_duty()
            # 経路: 中立→左(1/4) 左→右(2/4) 右→中立(1/4)
            keiro = [(self.churitsu, hidari, SWEEP_BYO * 0.25),
                     (hidari, migi, SWEEP_BYO * 0.50),
                     (migi, self.churitsu, SWEEP_BYO * 0.25)]
            for kara, made, byou in keiro:
                kaisu = max(2, int(byou / 0.05))
                for i in range(1, kaisu + 1):
                    duty = kara + (made - kara) * i / kaisu
                    with self.lock:
                        self.ima_duty = self.servo.shingou(duty)
                    time.sleep(0.05)
        finally:
            with self.lock:
                self.sweep_chu = False
                self.ichi = "chuuritsu"
                self.ima_duty = self.servo.shingou(self.churitsu)
            print("[スイープ] 完了。中立に戻りました")

    # ---- 中立の調整 ----
    def churitsu_bichosei(self, sa):
        """中立値を少しずつ動かす（±0.01% 単位）"""
        with self.lock:
            if self.sweep_chu:
                return
            self.churitsu = round(
                max(ANZEN_MIN + KIREKAKU_HABA,
                    min(ANZEN_MAX - KIREKAKU_HABA, self.churitsu + sa)), 2)
            self.ichi = "chuuritsu"
            self.ima_duty = self.servo.shingou(self.churitsu)
            print(f"[調整] 中立 {self.churitsu:.2f}%")

    def kijun_modosu(self):
        """中立値を基準(10.00%)に戻す。ハード調整をやり直すときに使う"""
        with self.lock:
            if self.sweep_chu:
                return
            self.churitsu = CHURITSU_KIJUN
            self.ichi = "chuuritsu"
            self.ima_duty = self.servo.shingou(self.churitsu)
            print(f"[調整] 中立を基準 {CHURITSU_KIJUN:.2f}% に戻しました")

    def gyaku_hanten(self):
        """ハンドルの左右の向きを反転し、個別設定ファイルに保存する"""
        with self.lock:
            if self.sweep_chu:
                return
            self.gyaku = not self.gyaku
            settei = {}
            if os.path.exists(SETTEI_FILE):
                try:
                    with open(SETTEI_FILE) as f:
                        settei = json.load(f)
                except Exception:
                    settei = {}
            settei["HANDORU_GYAKU"] = self.gyaku
            with open(SETTEI_FILE, "w") as f:
                json.dump(settei, f, indent=2, ensure_ascii=False)
            # いま押している位置に新しい向きで出し直す
            if self.ichi == "hidari":
                self.ima_duty = self.servo.shingou(self._hidari_duty())
            elif self.ichi == "migi":
                self.ima_duty = self.servo.shingou(self._migi_duty())
            print(f"[設定] ハンドルの向きを{'反転' if self.gyaku else '標準'}にして保存しました")

    def hozon(self):
        """中立値を設定ファイルに保存する（他のプログラムもこの値を使う）"""
        with self.lock:
            settei = {}
            if os.path.exists(SETTEI_FILE):
                try:
                    with open(SETTEI_FILE) as f:
                        settei = json.load(f)
                except Exception:
                    settei = {}
            settei["STEER_CHURITSU"] = self.churitsu
            with open(SETTEI_FILE, "w") as f:
                json.dump(settei, f, indent=2, ensure_ascii=False)
            print(f"[保存] {SETTEI_FILE} に中立 {self.churitsu:.2f}% を保存しました")

    def owari(self):
        """終了処理: 必ず中立に戻してから信号を止める"""
        with self.lock:
            self.servo.shingou(self.churitsu)
            time.sleep(0.5)
            self.servo.teishi()
            print("[終了] 中立に戻して信号を停止しました")

    def joutai_json(self):
        with self.lock:
            return json.dumps({
                "churitsu": round(self.churitsu, 2),
                "chosei": round(self.churitsu - CHURITSU_KIJUN, 2),
                "ima_duty": round(self.ima_duty, 2),
                "ichi": self.ichi,
                "gyaku": self.gyaku,
                "sweep": self.sweep_chu,
                "jikki": self.servo.pi is not None,
            })

# ==============================================================================
# ブロック4: ブラウザ画面
# ==============================================================================

GAMEN_HTML = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SERVO TEST</title>
<style>
  :root { --bg:#171a1e; --panel:#21262c; --line:#343b44;
    --text:#e8eaed; --dim:#8a939e; --acc:#5b8dbf; --ok:#43b06a; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text);
    font-family:"Hiragino Sans","Noto Sans JP",sans-serif;
    min-height:100vh; display:flex; flex-direction:column;
    align-items:center; gap:12px; padding:14px; }
  h1 { font-size:22px; letter-spacing:1px; }
  #wrap { display:grid; grid-template-columns:1fr 1fr; gap:12px;
    width:100%; max-width:1100px; align-items:start; }
  .col { display:flex; flex-direction:column; gap:12px; }
  @media (max-width:900px) { #wrap { grid-template-columns:1fr; } }
  #mode { font-size:14px; color:var(--dim); }
  #mode.jikki { color:var(--ok); }
  .card { background:var(--panel); border:1px solid var(--line);
    border-radius:12px; padding:16px; width:100%; }
  .card h2 { font-size:15px; color:var(--dim); margin-bottom:10px; }
  #btns { display:flex; gap:10px; }
  #btns button { flex:1; padding:26px 10px; font-size:22px; font-weight:700;
    border:none; border-radius:10px; color:#fff; cursor:pointer;
    background:#3b526b; }
  #btns button.now { background:var(--acc); outline:3px solid #9dc3e8; }
  #sweep { width:100%; margin-top:10px; padding:14px; font-size:17px;
    font-weight:700; border:none; border-radius:10px; color:#fff;
    background:#6b5b3b; cursor:pointer; }
  #gyaku { width:100%; margin-top:10px; padding:14px; font-size:15px;
    font-weight:700; border:none; border-radius:10px; color:#fff;
    background:#4a4f57; cursor:pointer; }
  #vals { display:grid; grid-template-columns:1fr 1fr 1fr; gap:10px; }
  .v { font-size:30px; font-weight:700; text-align:center;
    font-variant-numeric:tabular-nums; }
  .k { font-size:12px; color:var(--dim); text-align:center; margin-top:2px; }
  #adj { display:flex; gap:8px; }
  #adj button { flex:1; padding:16px 6px; font-size:18px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#4a4f57; cursor:pointer; }
  #saveline { display:flex; gap:8px; margin-top:10px; }
  #save { flex:2; padding:18px; font-size:19px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:var(--ok); cursor:pointer; }
  #reset { flex:1; padding:18px; font-size:15px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#7a4646; cursor:pointer; }
  #shiteiline { display:flex; gap:8px; }
  #shitei-v { flex:2; padding:12px; font-size:20px; border-radius:10px;
    border:1px solid var(--line); background:#14171b; color:var(--text);
    text-align:center; font-variant-numeric:tabular-nums; }
  #shitei-btn { flex:1; padding:12px; font-size:17px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#3b526b; cursor:pointer; }
  .hint { font-size:12px; color:var(--dim); margin-top:8px; }
  button:active { filter:brightness(.85); }
  #msg { text-align:center; color:var(--dim); font-size:14px; min-height:20px; }
</style>
</head>
<body>
  <h1>ステアリングサーボ テスト</h1>
  <div id="mode">--</div>

  <div id="wrap">
    <div class="col">
      <div class="card">
        <h2>ハンドル操作（押した位置に切れます）</h2>
        <div id="btns">
          <button id="b-hidari">◀ 左いっぱい</button>
          <button id="b-chuuritsu">● 中立</button>
          <button id="b-migi">右いっぱい ▶</button>
        </div>
        <button id="sweep">スイープ（左→右→中立を自動でゆっくり1往復）</button>
        <button id="gyaku">--</button>
        <div class="hint">左ボタンを押して右に切れる車は「反転」にする。設定は自動保存され、本番プログラムにも反映されます</div>
      </div>

      <div class="card">
        <h2>任意の信号を出す（切れ角の実測実習用）</h2>
        <div id="shiteiline">
          <input id="shitei-v" type="number" step="0.01" value="10.00">
          <button id="shitei-btn">この%を出す</button>
        </div>
        <div class="hint">出せる範囲: 中立±1.00%（切れ角の範囲内のみ）。分度器で実舵角を測る実習に使います</div>
      </div>
    </div>

    <div class="col">
      <div class="card">
        <h2>いまの値</h2>
        <div id="vals">
          <div><div class="v" id="v-churitsu">--</div><div class="k">中立値 [%]</div></div>
          <div><div class="v" id="v-chosei">--</div><div class="k">基準からの調整量 [%]</div></div>
          <div><div class="v" id="v-duty">--</div><div class="k">出力中の信号 [%]</div></div>
        </div>
        <div class="hint">調整量の目安: 調整式ロッドなら±0.10以内。標準の固定ロッド車はホーンの歯1枚分の残差をソフトで拾うため ±0.30 程度まで正常</div>
      </div>

      <div class="card">
        <h2>中立の微調整（タイヤがまっすぐになるまで）</h2>
        <div id="adj">
          <button data-d="-0.05">◀◀ -0.05</button>
          <button data-d="-0.01">◀ -0.01</button>
          <button data-d="0.01">+0.01 ▶</button>
          <button data-d="0.05">+0.05 ▶▶</button>
        </div>
        <div id="saveline">
          <button id="save">この中立値を保存する</button>
          <button id="reset">基準(10.50)に戻す</button>
        </div>
      </div>
    </div>
  </div>

  <div id="msg"></div>

<script>
  const $ = id => document.getElementById(id);
  const sousa = p => fetch(p, {method:"POST"});
  $("b-hidari").onclick = () => sousa("/hidari");
  $("b-chuuritsu").onclick = () => sousa("/chuuritsu");
  $("b-migi").onclick = () => sousa("/migi");
  $("sweep").onclick = () => sousa("/sweep");
  $("gyaku").onclick = () => {
    if (confirm("ハンドルの左右の向きを切り替えて保存します。よろしいですか？")) {
      sousa("/gyaku");
    }
  };
  document.querySelectorAll("#adj button").forEach(b => {
    b.onclick = () => sousa("/bichosei?d=" + b.dataset.d);
  });
  $("save").onclick = async () => {
    await sousa("/hozon");
    $("msg").textContent = "保存しました。この値をカルテにも記入してください。";
  };
  $("reset").onclick = () => {
    if (confirm("中立値を基準の10.50%（サーボ規格中立）に戻します。調整した値は消えます（保存済みのファイルは保存し直すまで変わりません）。よろしいですか？")) {
      sousa("/kijun");
    }
  };
  $("shitei-btn").onclick = () => {
    const v = parseFloat($("shitei-v").value);
    if (!isNaN(v)) sousa("/shitei?v=" + v.toFixed(2));
  };
  async function poll() {
    try {
      const s = await (await fetch("/status")).json();
      $("v-churitsu").textContent = s.churitsu.toFixed(2);
      $("v-chosei").textContent = (s.chosei >= 0 ? "+" : "") + s.chosei.toFixed(2);
      $("v-duty").textContent = s.ima_duty.toFixed(2);
      const m = $("mode");
      m.textContent = s.jikki ? "実機モード（サーボに信号を出しています）"
                              : "テストモード（画面のみ・信号は出ていません）";
      m.className = s.jikki ? "jikki" : "";
      ["hidari","chuuritsu","migi"].forEach(i => {
        $("b-" + i).className = (s.ichi === i) ? "now" : "";
      });
      $("gyaku").textContent = s.gyaku
        ? "ハンドルの向き: 反転（押すと標準に戻す）"
        : "ハンドルの向き: 標準（押すと反転して保存）";
      if (s.sweep) $("msg").textContent = "スイープ中... タイヤの動きを観察してください";
      else if ($("msg").textContent.startsWith("スイープ中")) $("msg").textContent = "";
    } catch (e) {}
  }
  setInterval(poll, 300);
  poll();
</script>
</body>
</html>
"""

def handler_wo_tsukuru(app):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                body = GAMEN_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/status":
                body = app.joutai_json().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path == "/hidari":
                app.ugokasu("hidari")
            elif self.path == "/chuuritsu":
                app.ugokasu("chuuritsu")
            elif self.path == "/migi":
                app.ugokasu("migi")
            elif self.path == "/sweep":
                app.sweep()
            elif self.path == "/gyaku":
                app.gyaku_hanten()
            elif self.path == "/kijun":
                app.kijun_modosu()
            elif self.path.startswith("/shitei"):
                try:
                    v = self.path.split("v=")[1]
                    # 全角数字・全角ピリオドでも読めるように直してから解釈
                    v = v.translate(str.maketrans("０１２３４５６７８９．", "0123456789."))
                    app.shitei(float(v))
                except Exception:
                    pass
            elif self.path.startswith("/bichosei"):
                try:
                    sa = float(self.path.split("d=")[1])
                    if abs(sa) <= 0.1:          # 一度に大きく動かせない安全策
                        app.churitsu_bichosei(sa)
                except Exception:
                    pass
            elif self.path == "/hozon":
                app.hozon()
            else:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler

# ==============================================================================
# ブロック5: main
# ==============================================================================

def main():
    app = App()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler_wo_tsukuru(app))
    print(f"[web] ブラウザで http://<この車のIPアドレス>:{HTTP_PORT} を開いてください")

    def tomeru(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, tomeru)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        # どんな終わり方でも、必ず中立に戻してから終了する
        print("\n[main] 停止処理中...")
        server.server_close()
        app.owari()
        print("[main] 終了しました")

if __name__ == "__main__":
    main()
