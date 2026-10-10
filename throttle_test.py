#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   throttle_test.py — スロットル（ESC・モーター）接続テスト・学習プログラム
#
#   自動車整備専門学校 自動運転実習用
#   詳しい手順は「説明書_throttle_test.md」を読んでください。
#
# ==============================================================================
#
#  ● このプログラムでやること
#
#      フェーズ1: スロットル学習 —— ATのシフトレンジにたとえると:
#          D点 = 前進し始める境界（ここより前進側で駆動がつながる）
#          R点 = 後退し始める境界（ここより後退側で駆動がつながる）
#          N   = D点とR点のちょうど中間（確実に止まる位置・自動計算）
#          L点 = モーターから「ピー」と音が鳴り始める点（D点の少し手前）。
#                動き出したあとは、ここまで下げても回り続ける。
#                本番プログラムはここをアクセル0%にして低速で走る
#          後退L点 = 後退側で音が鳴り始める点（R点の少し手前）
#      フェーズ2: 動作確認 —— 微速前進・微速後退で「キック→低速」を確かめる
#          キック = 動き出しの一瞬だけ D点(R点) の信号を出すこと。
#          そのあと本番と同じ低速の信号まで下げ、回り続けるかを見る
#      学習した値は車専用の設定ファイルに保存され、本番プログラムが使います。
#
#  ● 安全について【最重要】
#
#      このプログラムはモーターを回します。必ず守ること:
#        1. 車体をスタンドに載せ、4輪すべてを浮かせてから操作する
#        2. 回転部（タイヤ・ギア・シャフト）に手や物を近づけない
#      プログラム側の安全装置:
#        ・前進も後退も微速までしか出せず、10秒で自動的にNへ戻る
#        ・後退の前のブレーキは1秒だけ出して自動でNへ戻る
#        ・学習モードで動かせる範囲はNの±1.5%まで
#        ・どんな終わり方でも必ずNを出してから終了する
#
#  ● 使い方
#
#      1. ラズパイのターミナルで:   python3 throttle_test.py
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

THROTTLE_GPIO = 12     # アクセル(ESC)がつながっているピン番号。変えないこと
PWM_SHUHASU = 70       # 信号の周波数[Hz]。変えないこと

N_KIJUN = 10.48        # 学習前に仮で使うNポジション[%]
                       # 学習後は「(D点+R点)÷2」の計算値に置き換わる

SUROTTORU_GYAKU = False # スロットルの向き（個別設定項目）
                        #   True  = 低い%で前進、高い%で後退（タミヤ TT-02 の実測）
                        #   False = 高い%で前進、低い%で後退（ヨコモ RD2.0 の実測）
                        #   TT-02 で使うときは kuruma_settei.json に "SUROTTORU_GYAKU": true と書く

def zen():
    """前進側の向き: 前進が低い%なら -1、高い%なら +1"""
    return -1 if SUROTTORU_GYAKU else 1

BISOKU_HABA = 0.50     # 学習前の微速前進の張り出し[%]。学習後はD点−0.20を使う
BISOKU_D_OFFSET = 0.20 # 学習後の微速前進: D点からさらに前進側へ出す量[%]
BRAKE_HABA = 1.00      # ブレーキの張り出し[%]（Nから後退側へ）
BRAKE_BYO = 1.0        # ブレーキを出す時間[秒]（出しっぱなしにすると
                       # ESCの仕様で後退に入ることがあるため、必ず短時間）
ZENSHIN_SAIDAI_BYO = 10.0   # 前進をこの秒数で自動的にNへ戻す（安全装置）
GAKUSHU_HANI = 1.50    # 学習モードで動かせる範囲: N±この値[%]（安全装置）
BICHOSEI_SAIDAI = 0.10 # 微調整ボタン1回で動かせる上限[%]（安全策）
RENDA_MUSHI_BYO = 0.7  # 学習の段階が変わった直後、この秒数はボタンを受け付けない
                       # （二度押しで次の点まで記録されてしまうのを防ぐ）

# ---- 低速走行の確認（本番 rc_race.py と同じ信号を出すための値） ----
ACCEL_SAIDAI = 5       # 本番の最大アクセル[%]。kuruma_settei.json の値が入る
KICK_BYO = 0.3         # 発進キックの時間[秒]。kuruma_settei.json の値が入る
ZENKAI_HABA = 2.75     # アクセル100%の信号量[%]。rc_race.py と同じ値。変えないこと
KIRIKAE_BYO = 1.0      # 後退モードへの切替で、Nのまま待つ時間[秒]

# ---- タイヤエンコーダ（基準車のみ搭載） ----
ENCODER_ARI = False    # エンコーダの有無（個別設定項目）。初期値は「無し」。
                       # 搭載車だけ kuruma_settei.json に "ENCODER_ARI": true と書く
ENCODER_GPIO = 22      # エンコーダのパルス入力ピン
ENCODER_PPR = 36       # タイヤ1回転あたりのパルス数

# ---- 全開点探索（教員・基準車専用。ESCの飽和点を実測する） ----
TANSAKU_KAISHI = 2.00  # 探索を始める開度[%]（Nからの張り出し）
TANSAKU_KIZAMI = 0.25  # 1段ごとに増やす開度[%]
TANSAKU_JOGEN = 3.50   # 探索の上限開度[%]（これ以上は出さない）
TANSAKU_HOJI_BYO = 3.0 # 各段の保持時間[秒]。終わると自動でNに戻る

# 安全リミット: どんな操作でもこの範囲の外の信号は絶対に出さない
ANZEN_MIN = 7.50
ANZEN_MAX = 13.00

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
# ブロック2: ESCへの信号出力（pigpioライブラリを使用）
# ==============================================================================

class Esc:
    def __init__(self):
        self.pi = None
        try:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.pi = None
                raise RuntimeError("pigpiod が動いていません")
            print("[ESC] pigpio に接続しました（実機モード）")
        except Exception as e:
            print(f"[ESC] pigpio が使えません ({e})")
            try:
                self.pi = Pi5PWM()
                print("[ESC] Pi 5 のハードウェアPWMで出力します（実機モード）")
            except Exception as e2:
                self.pi = None
                print(f"[ESC] Pi 5 用の出力も使えません ({e2})")
                print("[ESC] 画面表示だけのテストモードで動きます")

    def shingou(self, duty):
        """duty[%] の信号を出す。安全リミットの外は絶対に出さない"""
        duty = max(ANZEN_MIN, min(ANZEN_MAX, duty))
        if self.pi is not None:
            self.pi.hardware_PWM(THROTTLE_GPIO, PWM_SHUHASU, int(duty * 10000))
        return duty

    def teishi(self):
        """信号を止める（ESCは信号なしを検出してピーピー鳴き始める＝安全）"""
        if self.pi is not None:
            self.pi.hardware_PWM(THROTTLE_GPIO, 0, 0)
            self.pi.stop()

class Encoder:
    """タイヤエンコーダのパルスを数えて回転数[rpm]を計算する（搭載車のみ）"""

    def __init__(self, pi):
        self.rpm = None
        self._kaisu = 0
        if not ENCODER_ARI or pi is None:
            return
        try:
            import pigpio
            pi.set_mode(ENCODER_GPIO, pigpio.INPUT)
            pi.set_pull_up_down(ENCODER_GPIO, pigpio.PUD_UP)
            pi.callback(ENCODER_GPIO, pigpio.RISING_EDGE, self._pulse)
            self.rpm = 0.0
            threading.Thread(target=self._keisan, daemon=True).start()
            print(f"[エンコーダ] GPIO{ENCODER_GPIO} で回転数を計測します")
        except Exception as e:
            print(f"[エンコーダ] 初期化できません ({e})")

    def _pulse(self, gpio, level, tick):
        self._kaisu += 1

    def _keisan(self):
        """0.25秒ごとのパルス数から回転数を計算し続ける"""
        mae = 0
        while True:
            time.sleep(0.25)
            ima = self._kaisu
            sa = ima - mae
            mae = ima
            # 0.25秒に sa パルス → 1分あたり sa×240 パルス → ÷PPR で回転数
            self.rpm = round(sa * 240 / ENCODER_PPR, 0)


# ==============================================================================
# ブロック3: テストと学習のまとめ役
# ==============================================================================

class App:
    def __init__(self):
        self.lock = threading.Lock()
        self.esc = Esc()

        # 設定ファイルから学習済みの値を読み込む（あれば）
        self.d_ten = None       # 前進境界（学習で決める）
        self.r_ten = None       # 後退境界（学習で決める）
        self.l_ten = None       # 音が鳴り始める点（学習で決める。無くても動く）
        self.rl_ten = None      # 後退側で音が鳴り始める点（同上）
        self.n = N_KIJUN        # Nポジション
        if os.path.exists(SETTEI_FILE):
            try:
                with open(SETTEI_FILE) as f:
                    settei = json.load(f)
                self.d_ten = settei.get("ZENSHIN_KYOKAI")
                self.r_ten = settei.get("KOTAI_KYOKAI")
                self.n = float(settei.get("THROTTLE_N", N_KIJUN))
                self.l_ten = settei.get("ZENSHIN_SAITEI")
                self.rl_ten = settei.get("KOTAI_SAITEI")
                print(f"[設定] 読込: N={self.n:.2f}% L点={self.l_ten} D点={self.d_ten} "
                      f"後退L点={self.rl_ten} R点={self.r_ten}")
            except Exception as e:
                print(f"[設定] 設定ファイルが読めません ({e})。基準値を使います")

        self.enc = Encoder(self.esc.pi)

        self.mode = "normal"          # normal / jido_gakushu / tansaku
        self.ichi = "n"               # n / zenshin / brake / kirikae / kotai
                                      #   / gakushu / tansaku
        self.kick_chu = False         # いまキックの信号を出しているか
        self._sousa_ban = 0           # 操作の通し番号。ボタンが押されるたびに増え、
                                      # 古い操作の続き（キック後の切替など）を無効にする
        self.gaku_phase = None        # 学習の段階: s(音) / d / kaijo / rs(音) / r
        self._tobashi = False         # 「音が鳴らない」でL点をとばしたか
        self._phase_jikoku = 0.0      # いまの段階に入った時刻（連打の無視用）
        self.gaku_jikko = False       # 学習の実行中フラグ
        self.gakushu_duty = None      # 学習中に人が動かしている出力値
        self._osareta = threading.Event()   # 「回り始めた!」ボタンの合図
        self.tan_kiroku = []          # 全開点探索の記録 [[開度, rpm], ...]
        self.tan_kaido = None         # いま探索中の開度
        self.tan_jikko = False        # 自動探索の実行中フラグ
        self.tan_howa = False         # 飽和を検出したか
        self.tan_howa_kaido = None    # 検出した全開点（飽和直前の開度）
        self.gakushu_duty = self.n    # 学習モードで動かしている値
        self.jido_teishi_yotei = None
        self.ima_duty = self.esc.shingou(self.n)   # 起動時は必ずN
        print(f"[ESC] 起動: N {self.n:.2f}% を出力中")
        print("[ESC] ESCのピーピー音が止まれば、信号が届いています")

        threading.Thread(target=self._mihari, daemon=True).start()

    def _mihari(self):
        """前進しっぱなしを自動で止める見張り"""
        while True:
            time.sleep(0.1)
            with self.lock:
                if (self.ichi in ("zenshin", "kotai") and self.jido_teishi_yotei
                        and time.time() >= self.jido_teishi_yotei):
                    self._sousa_ban += 1
                    self.ichi = "n"
                    self.kick_chu = False
                    self.jido_teishi_yotei = None
                    self.ima_duty = self.esc.shingou(self.n)
                    print(f"[安全] {ZENSHIN_SAIDAI_BYO:.0f}秒経過 → 自動でNに戻しました")

    # ---- 通常モードのボタン ----
    def btn_n(self):
        with self.lock:
            if self.mode != "normal" or self.tan_jikko:
                return
            self._sousa_ban += 1          # 進行中のキック・切替を取り消す
            self.ichi = "n"
            self.kick_chu = False
            self.jido_teishi_yotei = None
            self.ima_duty = self.esc.shingou(self.n)
            print(f"[操作] N: {self.ima_duty:.2f}%")

    # ---- 低速確認で出す信号の計算（本番 rc_race.py と同じ考え方） ----
    @staticmethod
    def _accel():
        """本番の最大アクセル[%]（設定がおかしいときは初期値5）"""
        a = ACCEL_SAIDAI
        if isinstance(a, bool) or not isinstance(a, (int, float)):
            return 5.0
        return max(0.0, min(100.0, float(a)))

    @staticmethod
    def _kick_byo():
        """キックの時間[秒]（設定がおかしいときは初期値0.3。最長2秒）"""
        k = KICK_BYO
        if isinstance(k, bool) or not isinstance(k, (int, float)):
            return 0.3
        return max(0.0, min(2.0, float(k)))

    def _teisoku_mae(self):
        """微速前進で出す信号を返す: (キックの信号 または None, 低速の信号)
           L点を学習済み → キック=D点、低速=L点＋本番のアクセル分
                           （ただし「D点より0.20前進側」を上限とする＝安全策）
           L点が未学習   → キック無しで「D点より0.20前進側」（従来の微速）"""
        if self.d_ten is None:
            return None, self.n + zen() * BISOKU_HABA
        jogen = self.d_ten + zen() * BISOKU_D_OFFSET
        if self.l_ten is None:
            return None, jogen
        teisoku = self.l_ten + zen() * ZENKAI_HABA * self._accel() / 100.0
        if (teisoku - jogen) * zen() > 0:
            teisoku = jogen
        # キック中はD点より弱い信号にしない（低速のほうが強ければそちら）
        kick = self.d_ten if (self.d_ten - teisoku) * zen() > 0 else teisoku
        return kick, teisoku

    def _teisoku_ushiro(self):
        """微速後退で出す信号を返す: (キックの信号 または None, 低速の信号)
           R点が未学習なら (None, None)。考え方は前進と左右対称"""
        if self.r_ten is None:
            return None, None
        jogen = self.r_ten - zen() * BISOKU_D_OFFSET
        if self.rl_ten is None:
            return None, jogen
        teisoku = self.rl_ten - zen() * ZENKAI_HABA * self._accel() / 100.0
        if (jogen - teisoku) * zen() > 0:
            teisoku = jogen
        kick = self.r_ten if (teisoku - self.r_ten) * zen() > 0 else teisoku
        return kick, teisoku

    def btn_zenshin(self):
        """微速前進: 発進キック → 本番と同じ低速の信号（10秒で自動停止）"""
        with self.lock:
            if (self.mode != "normal" or self.tan_jikko
                    or self.ichi not in ("n", "zenshin")):
                return
            self._sousa_ban += 1
            ban = self._sousa_ban
            kick, teisoku = self._teisoku_mae()
            self.ichi = "zenshin"
            self.jido_teishi_yotei = time.time() + ZENSHIN_SAIDAI_BYO
            if kick is None:
                self.kick_chu = False
                self.ima_duty = self.esc.shingou(teisoku)
                print(f"[操作] 微速前進: {self.ima_duty:.2f}% "
                      f"（L点が未学習のためキック無し / {ZENSHIN_SAIDAI_BYO:.0f}秒で自動停止）")
                return
            self.kick_chu = True
            self.ima_duty = self.esc.shingou(kick)
            print(f"[操作] 微速前進: キック {self.ima_duty:.2f}% を {self._kick_byo():.1f}秒 "
                  f"→ 低速 {teisoku:.2f}% （{ZENSHIN_SAIDAI_BYO:.0f}秒で自動停止）")
        threading.Thread(target=self._kick_ato, args=(ban, "zenshin", teisoku),
                         daemon=True).start()

    def _kick_ato(self, ban, ichi, teisoku):
        """キックの時間が過ぎたら低速の信号へ下げる（別スレッド）"""
        time.sleep(self._kick_byo())
        with self.lock:
            if self._sousa_ban != ban or self.ichi != ichi:
                return                    # その間にNなど別の操作がされた
            self.kick_chu = False
            self.ima_duty = self.esc.shingou(teisoku)
            print(f"[操作] キック終了 → 低速 {self.ima_duty:.2f}%")

    def btn_kotai(self):
        """微速後退: ブレーキ → N（後退モードへ切替）→ キック → 低速。
           ESCは「後退側→N→もう一度後退側」の順でないと後退に入らないため、
           最初に必ずブレーキが入る（前進中に押せばブレーキの確認になる）"""
        with self.lock:
            if (self.mode != "normal" or self.tan_jikko
                    or self.ichi not in ("n", "zenshin")):
                return
            self._sousa_ban += 1
            ban = self._sousa_ban
            self.ichi = "brake"
            self.kick_chu = False
            self.jido_teishi_yotei = None
            self.ima_duty = self.esc.shingou(self.n - zen() * BRAKE_HABA)
            print(f"[操作] 微速後退: まずブレーキ {self.ima_duty:.2f}% を {BRAKE_BYO:.0f}秒")
        threading.Thread(target=self._kotai_nagare, args=(ban,), daemon=True).start()

    def _kotai_nagare(self, ban):
        """微速後退の続き（別スレッド）: ブレーキ → N → キック → 低速"""
        time.sleep(BRAKE_BYO)
        with self.lock:
            if self._sousa_ban != ban:
                return
            self.ichi = "kirikae"
            self.ima_duty = self.esc.shingou(self.n)
            print(f"[操作] ブレーキ終了 → N で {KIRIKAE_BYO:.0f}秒待ち（後退モードへ切替）")
        time.sleep(KIRIKAE_BYO)
        with self.lock:
            if self._sousa_ban != ban:
                return
            kick, teisoku = self._teisoku_ushiro()
            if teisoku is None:
                self.ichi = "n"
                print("[操作] R点が未学習のため、後退は行いません（ブレーキの確認のみ）")
                return
            self.ichi = "kotai"
            self.jido_teishi_yotei = time.time() + ZENSHIN_SAIDAI_BYO
            if kick is None:
                self.ima_duty = self.esc.shingou(teisoku)
                print(f"[操作] 微速後退: {self.ima_duty:.2f}% "
                      f"（後退L点が未学習のためキック無し / {ZENSHIN_SAIDAI_BYO:.0f}秒で自動停止）")
                return
            self.kick_chu = True
            self.ima_duty = self.esc.shingou(kick)
            print(f"[操作] 微速後退: キック {self.ima_duty:.2f}% を {self._kick_byo():.1f}秒 "
                  f"→ 低速 {teisoku:.2f}% （{ZENSHIN_SAIDAI_BYO:.0f}秒で自動停止）")
        self._kick_ato(ban, "kotai", teisoku)

    # ---- スロットル自動学習（前進→後退をボタン一つで） ----
    def jido_gakushu_kaishi(self):
        """自動学習を開始。信号を少しずつ動かしてもらい、
           音が鳴り始めたら「音が鳴り始めた!」、タイヤが回り始めたら
           「回り始めた!」を押してもらう
           （L点→D点→切替→後退L点→R点の順に進行）"""
        with self.lock:
            if (self.mode != "normal" or self.tan_jikko
                    or self.ichi in ("brake", "kirikae")):
                return
            self._sousa_ban += 1
            self.kick_chu = False
            self.jido_teishi_yotei = None
            self.mode = "jido_gakushu"
            self.ichi = "gakushu"
            self.gaku_phase = "s"
            self._tobashi = False
            self._phase_jikoku = time.time()
            self.gaku_jikko = True
            self._osareta.clear()
        threading.Thread(target=self._jido_gakushu, daemon=True).start()

    def kaiten_oshita(self):
        """「音が鳴り始めた!」「回り始めた!」ボタンが押された（同じボタン）"""
        if self.mode != "jido_gakushu":
            return
        if time.time() - self._phase_jikoku < RENDA_MUSHI_BYO:
            return          # 段階が変わった直後の二度押しは無視する
        self._osareta.set()

    def oto_tobasu(self):
        """「音が鳴らない」ボタン: 音の点を記録せずに、回り始める点の学習へ進む"""
        with self.lock:
            if self.mode != "jido_gakushu" or self.gaku_phase not in ("s", "rs"):
                return
            self._tobashi = True
        self._osareta.set()

    def jido_gakushu_chushi(self):
        """自動学習を中止する（保存しない）"""
        with self.lock:
            if self.mode != "jido_gakushu":
                return
            self.gaku_jikko = False
        self._osareta.set()

    def gakushu_bichosei(self, sa):
        """学習中の微調整（人がボタンで少しずつ動かす）。N±1.5%の範囲内のみ"""
        with self.lock:
            if self.gaku_phase not in ("s", "d", "rs", "r"):
                return
            self.gakushu_duty = round(
                max(self.n - GAKUSHU_HANI,
                    min(self.n + GAKUSHU_HANI, self.gakushu_duty + sa)), 2)
            self.ima_duty = self.esc.shingou(self.gakushu_duty)
            print(f"[学習] {self.gakushu_duty:.2f}%")

    def _osareru_made_matsu(self, n_kara=True):
        """ボタンが押されるまで待ち、押された時点の値を返す。
           中止されたら None を返す。
           n_kara=False のときは N に戻さず、いまの出力から続ける"""
        if n_kara:
            with self.lock:
                self.gakushu_duty = self.n
                self.ima_duty = self.esc.shingou(self.gakushu_duty)
        self._osareta.clear()
        while True:
            if self._osareta.wait(0.2):
                self._osareta.clear()
                with self.lock:
                    return self.gakushu_duty if self.gaku_jikko else None
            with self.lock:
                if not self.gaku_jikko:
                    return None

    def _jido_gakushu(self):
        """学習の本体: 案内を出し、人の操作と「回り始めた!」を待って進行する"""
        n0 = self.n
        s = d = rs = r = None
        try:
            # --- 段階1: 音が鳴り始める点（L点） ---
            print("[学習] まずは前進の学習を始めます。")
            print("[学習] " + ("◀ボタンで出力を少しずつ下げて" if SUROTTORU_GYAKU else "▶ボタンで出力を少しずつ上げて") + "ください。")
            print("[学習] モーターから「ピー」と音が鳴り始めたら「音が鳴り始めた!」を押してください。")
            print("[学習] （タイヤはまだ回りません。音が鳴らない車は「音が鳴らない」を押す）")
            s = self._osareru_made_matsu()
            with self.lock:
                if not self.gaku_jikko:
                    return
                if self._tobashi:
                    s = None
                    print("[学習] L点はとばしました（この車は低速走行を使いません）。")
                else:
                    print(f"[学習] L点 = {s:.2f}% を記録しました。")
                self.gaku_phase = "d"
                self._phase_jikoku = time.time()

            # --- 段階2: 前進境界（D点）。Nに戻さず、そのまま続ける ---
            print("[学習] そのまま続けて、タイヤが前進方向に回り始めたら「回り始めた!」を押してください。")
            d = self._osareru_made_matsu(n_kara=False)
            with self.lock:
                self.ima_duty = self.esc.shingou(n0)   # いったんNに戻す
            if d is None:
                return
            print(f"[学習] D点 = {d:.2f}% を記録しました。")

            # --- 段階3: 後退モードへの切替（自動） ---
            with self.lock:
                self.gaku_phase = "kaijo"
            print("[学習] 次に後退の学習を始めます。後退モードへ切替中...")
            with self.lock:
                self.ima_duty = self.esc.shingou(n0 - zen() * BRAKE_HABA)   # 後退側へ一瞬
            time.sleep(0.5)
            with self.lock:
                self.ima_duty = self.esc.shingou(n0)
            time.sleep(1.0)
            with self.lock:
                if not self.gaku_jikko:
                    return
                self.gaku_phase = "rs"
                self._tobashi = False
                self._phase_jikoku = time.time()

            # --- 段階4: 後退側で音が鳴り始める点（後退L点） ---
            print("[学習] " + ("▶ボタンで出力を少しずつ上げて" if SUROTTORU_GYAKU else "◀ボタンで出力を少しずつ下げて") + "ください。")
            print("[学習] モーターから「ピー」と音が鳴り始めたら「音が鳴り始めた!」を押してください。")
            rs = self._osareru_made_matsu()
            with self.lock:
                if not self.gaku_jikko:
                    return
                if self._tobashi:
                    rs = None
                    print("[学習] 後退L点はとばしました。")
                else:
                    print(f"[学習] 後退L点 = {rs:.2f}% を記録しました。")
                self.gaku_phase = "r"
                self._phase_jikoku = time.time()

            # --- 段階5: 後退境界（R点）。Nに戻さず、そのまま続ける ---
            print("[学習] そのまま続けて、タイヤが後退方向に回り始めたら「回り始めた!」を押してください。")
            r = self._osareru_made_matsu(n_kara=False)
            with self.lock:
                self.ima_duty = self.esc.shingou(n0)
            if r is None:
                return
            print(f"[学習] R点 = {r:.2f}% を記録しました。")

            # --- 計算・保存 ---
            with self.lock:
                self.d_ten = d
                self.r_ten = r
                self.n = round((d + r) / 2, 2)
                # L点は「NとD点の間」にあるときだけ採用する
                # （押し間違いの値で本番の出力が狂うのを防ぐ）
                l_ok = (s is not None and (s - self.n) * zen() > 0
                        and (d - s) * zen() >= 0)
                self.l_ten = s if l_ok else None
                # 後退L点も同じ（NとR点の間にあるときだけ採用）
                rl_ok = (rs is not None and (self.n - rs) * zen() > 0
                         and (rs - r) * zen() >= 0)
                self.rl_ten = rs if rl_ok else None
                self._settei_kaku()
            print(f"[学習] 完了! N = ({d:.2f} + {r:.2f}) ÷ 2 = {self.n:.2f}%")
            if l_ok:
                print(f"[学習] L点 {s:.2f}% を保存しました（本番のアクセル0%になります）")
            elif s is not None:
                print(f"[学習] 注意: L点 {s:.2f}% がNとD点の間にないため、保存しませんでした。")
                print("[学習] 低速走行を使うには、学習をもう一度行ってください。")
            if rl_ok:
                print(f"[学習] 後退L点 {rs:.2f}% を保存しました（微速後退の確認に使います）")
            elif rs is not None:
                print(f"[学習] 注意: 後退L点 {rs:.2f}% がNとR点の間にないため、保存しませんでした。")
        finally:
            with self.lock:
                self.mode = "normal"
                self.ichi = "n"
                self.gaku_phase = None
                self.gaku_jikko = False
                self.gakushu_duty = None
                self.ima_duty = self.esc.shingou(self.n)
            if d is None or r is None:
                print("[学習] 保存せずに終了しました（中止）")

    def gakushu_risetto(self):
        """学習をやり直す（学習した点をすべて消してNを基準値に戻す）"""
        with self.lock:
            if self.mode != "normal" or self.ichi in ("brake", "kirikae"):
                return
            self._sousa_ban += 1
            self.kick_chu = False
            self.jido_teishi_yotei = None
            self.d_ten = None
            self.r_ten = None
            self.l_ten = None
            self.rl_ten = None
            self.n = N_KIJUN
            self._settei_kaku()
            self.ichi = "n"
            self.ima_duty = self.esc.shingou(self.n)
            print(f"[学習] リセット。N基準値 {N_KIJUN:.2f}% に戻しました")

    def _settei_kaku(self):
        """設定ファイルへ書き込む（self.lock 取得済みで呼ばれる）"""
        settei = {}
        if os.path.exists(SETTEI_FILE):
            try:
                with open(SETTEI_FILE) as f:
                    settei = json.load(f)
            except Exception:
                settei = {}
        settei["ZENSHIN_KYOKAI"] = self.d_ten
        settei["KOTAI_KYOKAI"] = self.r_ten
        settei["THROTTLE_N"] = self.n
        if self.l_ten is None:
            settei.pop("ZENSHIN_SAITEI", None)    # 古い値を残さない
        else:
            settei["ZENSHIN_SAITEI"] = self.l_ten
        if self.rl_ten is None:
            settei.pop("KOTAI_SAITEI", None)
        else:
            settei["KOTAI_SAITEI"] = self.rl_ten
        with open(SETTEI_FILE, "w") as f:
            json.dump(settei, f, indent=2, ensure_ascii=False)
        print(f"[保存] {SETTEI_FILE} に書き込みました")

    # ---- 全開点探索（教員・基準車専用・全自動） ----
    def tansaku_kaishi(self):
        """自動探索: 開度を段階的に上げ、rpmが伸びなくなったら自動停止"""
        with self.lock:
            if self.mode != "normal" or self.ichi in ("brake", "kirikae"):
                return
            self._sousa_ban += 1
            self.kick_chu = False
            self.jido_teishi_yotei = None
            self.mode = "tansaku"
            self.ichi = "tansaku"
            self.tan_kiroku = []
            self.tan_howa = False
            self.tan_howa_kaido = None
            self.tan_kaido = None
            self.tan_jikko = True
        threading.Thread(target=self._tansaku_jikko, daemon=True).start()

    def tansaku_chushi(self):
        """探索を中止する（実行中でも即座に効く）"""
        with self.lock:
            if self.mode != "tansaku":
                return
            self.tan_jikko = False

    def _tansaku_jikko(self):
        """自動探索の本体。各段3秒保持→rpm計測→飽和なら終了"""
        kaido = TANSAKU_KAISHI
        try:
            while True:
                with self.lock:
                    if not self.tan_jikko:
                        print("[探索] 中止されました")
                        break
                    self.tan_kaido = kaido
                    self.ima_duty = self.esc.shingou(self.n + zen() * kaido)  # 前進側へ
                print(f"[探索] 開度 {kaido:.2f}% を保持中...")

                # 保持（0.1秒刻みで中止フラグを確認しながら待つ）
                machi = TANSAKU_HOJI_BYO - 0.5
                while machi > 0:
                    time.sleep(0.1)
                    machi -= 0.1
                    with self.lock:
                        if not self.tan_jikko:
                            break
                with self.lock:
                    if not self.tan_jikko:
                        print("[探索] 中止されました")
                        break

                # 最後の0.5秒でrpmを3回読んで平均
                yomi = []
                for _ in range(3):
                    if self.enc.rpm is not None:
                        yomi.append(self.enc.rpm)
                    time.sleep(0.17)
                rpm = round(sum(yomi) / len(yomi), 0) if yomi else None

                with self.lock:
                    self.tan_kiroku.append([kaido, rpm])
                print(f"[探索] 開度 {kaido:.2f}% → {rpm} rpm")

                # 飽和判定: 前の段からrpmが1%も伸びていない → 自動終了
                with self.lock:
                    if (len(self.tan_kiroku) >= 2 and rpm is not None
                            and self.tan_kiroku[-2][1]
                            and rpm < self.tan_kiroku[-2][1] * 1.01):
                        self.tan_howa = True
                        self.tan_howa_kaido = self.tan_kiroku[-2][0]
                        print(f"[探索] 飽和を検出。全開点はおよそ "
                              f"{self.tan_howa_kaido:.2f}% です")
                        break

                if kaido + TANSAKU_KIZAMI > TANSAKU_JOGEN + 0.001:
                    print("[探索] 上限に達しました（飽和未検出）")
                    break
                kaido = round(kaido + TANSAKU_KIZAMI, 2)
        finally:
            with self.lock:
                self.tan_jikko = False
                self.tan_kaido = None
                self.mode = "normal"
                self.ichi = "n"
                self.ima_duty = self.esc.shingou(self.n)
            print("[探索] 終了。Nに戻りました。記録:", self.tan_kiroku)

    def owari(self):
        """終了処理: 必ずNを出してから信号を止める"""
        with self.lock:
            self.esc.shingou(self.n)
            time.sleep(0.5)
            self.esc.teishi()
            print("[終了] Nに戻して信号を停止しました（ESCがピーピー鳴くのは正常）")

    def joutai_json(self):
        with self.lock:
            nokori = None
            if self.ichi in ("zenshin", "kotai") and self.jido_teishi_yotei:
                nokori = max(0, round(self.jido_teishi_yotei - time.time(), 1))
            mae_kick, mae_teisoku = self._teisoku_mae()
            ushiro_kick, ushiro_teisoku = self._teisoku_ushiro()
            maru = lambda v: None if v is None else round(v, 2)
            # 低速の信号が安全の上限（D点/R点から0.20）で頭打ちになっているか
            mae_jogen = (self.l_ten is not None and self.d_ten is not None
                         and (self.l_ten + zen() * ZENKAI_HABA * self._accel() / 100.0
                              - mae_teisoku) * zen() > 1e-9)
            ushiro_jogen = (self.rl_ten is not None and self.r_ten is not None
                            and (ushiro_teisoku - (self.rl_ten - zen() * ZENKAI_HABA
                                 * self._accel() / 100.0)) * zen() > 1e-9)
            zone = None
            if self.d_ten is not None and self.r_ten is not None:
                zone = round(abs(self.r_ten - self.d_ten), 2)
            return json.dumps({
                "mode": self.mode,
                "ichi": self.ichi,
                "gaku_phase": self.gaku_phase,
                "n": round(self.n, 2),
                "d_ten": self.d_ten,
                "r_ten": self.r_ten,
                "l_ten": self.l_ten,
                "rl_ten": self.rl_ten,
                "gyaku": SUROTTORU_GYAKU,
                "kick_chu": self.kick_chu,
                "kick_byo": self._kick_byo(),
                "accel": self._accel(),
                "mae_kick": maru(mae_kick), "mae_teisoku": maru(mae_teisoku),
                "ushiro_kick": maru(ushiro_kick), "ushiro_teisoku": maru(ushiro_teisoku),
                "mae_jogen": mae_jogen, "ushiro_jogen": ushiro_jogen,
                "zone": zone,
                "ima_duty": round(self.ima_duty, 2),
                "nokori": nokori,
                "rpm": self.enc.rpm,
                "tan_kiroku": self.tan_kiroku,
                "tan_jikko": self.tan_jikko,
                "tan_kaido": self.tan_kaido,
                "tan_howa": self.tan_howa,
                "tan_howa_kaido": self.tan_howa_kaido,
                "jikki": self.esc.pi is not None,
            })

# ==============================================================================
# ブロック4: ブラウザ画面
# ==============================================================================

GAMEN_HTML = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>THROTTLE TEST</title>
<style>
  :root { --bg:#171a1e; --panel:#21262c; --line:#343b44;
    --text:#e8eaed; --dim:#8a939e; --acc:#5b8dbf; --ok:#43b06a;
    --warn:#d9a13b; --stop:#c94f4f; }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text);
    font-family:"Hiragino Sans","Noto Sans JP",sans-serif;
    min-height:100vh; display:flex; flex-direction:column;
    align-items:center; gap:12px; padding:14px; }
  h1 { font-size:22px; letter-spacing:1px; }
  #mode { font-size:14px; color:var(--dim); }
  #mode.jikki { color:var(--ok); }
  #warn { width:100%; max-width:1100px; background:#4a2f2f;
    border:1px solid var(--stop); color:#f0b9b9; border-radius:10px;
    padding:12px; text-align:center; font-weight:700; font-size:16px; }
  #wrap { display:grid; grid-template-columns:1fr 1fr; gap:12px;
    width:100%; max-width:1100px; align-items:start; }
  .col { display:flex; flex-direction:column; gap:12px; }
  @media (max-width:900px) { #wrap { grid-template-columns:1fr; } }
  .card { background:var(--panel); border:1px solid var(--line);
    border-radius:12px; padding:16px; width:100%; }
  .card h2 { font-size:15px; color:var(--dim); margin-bottom:10px; }
  .stack { display:flex; flex-direction:column; gap:10px; }
  .stack button { padding:20px 10px; font-size:20px; font-weight:700;
    border:none; border-radius:10px; color:#fff; cursor:pointer; }
  #b-n { background:#3b526b; }
  #b-zenshin { background:#3b6b46; }
  #b-kotai { background:#6b4a3b; }
  #b-jido { background:var(--acc); }
  #b-tan-kaishi, #b-tan-chushi { background:#4a4f57; }
  .stack button.now { outline:3px solid #9dc3e8; }
  #vals { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  .v { font-size:28px; font-weight:700; text-align:center;
    font-variant-numeric:tabular-nums; }
  .k { font-size:12px; color:var(--dim); text-align:center; margin-top:2px; }
  #adj { display:flex; gap:8px; margin-top:10px; }
  #adj button { flex:1; padding:16px 6px; font-size:18px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#4a4f57; cursor:pointer; }
  #adj { display:flex; gap:8px; margin-top:10px; }
  #adj button { flex:1; padding:16px 6px; font-size:18px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#4a4f57; cursor:pointer; }
  #gakuline { display:flex; gap:8px; }
  #gaku-annai { color:var(--warn); font-size:15px; font-weight:700;
    margin-bottom:10px; min-height:22px; }
  #b-kaiten { flex:2; padding:28px 16px; font-size:24px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:var(--ok); cursor:pointer; }
  #gaku-chushi { flex:1; padding:16px; font-size:15px; font-weight:700;
    border:none; border-radius:10px; color:#fff; background:#7a4646; cursor:pointer; }
  #reset, #b-tobasu { width:100%; margin-top:10px; padding:12px; font-size:14px;
    border:none; border-radius:10px; color:#fff; background:#4a4f57; cursor:pointer; }
  .hint { font-size:12px; color:var(--dim); margin-top:8px; }
  button:active { filter:brightness(.85); }
  button:disabled { opacity:.35; cursor:default; }
  #msg { text-align:center; color:var(--warn); font-size:15px;
    font-weight:700; min-height:22px; }
</style>
</head>
<body>
  <h1>スロットル（ESC・モーター）テスト</h1>
  <div id="mode" style="display:none"></div>
  <div id="warn">必ず車体をスタンドに載せ、4輪を浮かせてから操作すること</div>
  <div id="msg"></div>

  <div id="wrap">
    <div class="col">
      <div class="card">
        <h2>スロットル学習（前進→後退をまとめて自動で）</h2>
        <div class="stack">
          <button id="b-jido">スロットル開度学習開始</button>
        </div>
        <div class="hint">案内に従って、右の◀▶ボタンで出力を自分のペースで動かします。モーターから「ピー」と音が鳴り始めたら「音が鳴り始めた!」、タイヤが回り始めたら「回り始めた!」。これを前進と後退で1回ずつ行います（押すのは全部で4回）</div>
      </div>

      <div class="card">
        <h2>スロットル操作（通常モード）</h2>
        <div class="stack">
          <button id="b-n">● N（停止）</button>
          <button id="b-zenshin">▲ 微速前進（キック→低速・10秒で自動停止）</button>
          <button id="b-kotai">▼ 微速後退（ブレーキ→キック→低速・10秒で自動停止）</button>
        </div>
        <div class="hint" id="teisoku-mae">微速前進: --</div>
        <div class="hint" id="teisoku-ushiro">微速後退: --</div>
        <div class="hint">キック＝動き出しの一瞬だけD点（後退はR点）の信号を出すこと。そのあと本番と同じ低速の信号まで下げます。<b>下げたあともタイヤが回り続ければ合格</b>です。微速後退は、ESCの仕様で最初に必ずブレーキが入ります（前進中に押せばブレーキの確認になります）</div>
      </div>

      <div class="card">
        <h2>全開点探索（教員・基準車専用 / エンコーダ必須）</h2>
        <div class="stack">
          <button id="b-tan-kaishi">自動探索を開始（2.00%→最大3.50%）</button>
          <button id="b-tan-chushi">探索を中止する</button>
        </div>
        <div class="hint" id="tan-kekka">記録: まだありません</div>
      </div>
    </div>

    <div class="col">
      <div class="card">
        <h2>いまの値</h2>
        <div id="vals">
          <div><div class="v" id="v-n">--</div><div class="k">N（停止位置）[%]</div></div>
          <div><div class="v" id="v-duty">--</div><div class="k">出力中の信号 [%]</div></div>
          <div><div class="v" id="v-l">--</div><div class="k">L点（音が鳴り始める点）[%]</div></div>
          <div><div class="v" id="v-d">--</div><div class="k">D点（前進境界）[%]</div></div>
          <div><div class="v" id="v-rl">--</div><div class="k">後退L点（後退で音が鳴り始める点）[%]</div></div>
          <div><div class="v" id="v-r">--</div><div class="k">R点（後退境界）[%]</div></div>
          <div><div class="v" id="v-rpm">--</div><div class="k">タイヤ回転数 [rpm]</div></div>
        </div>
        <div class="hint" id="zone-hint">停止ゾーン幅（R−D）: --</div>
        <div id="adj">
          <button data-d="-0.05" disabled>◀◀ -0.05</button>
          <button data-d="-0.01" disabled>◀ -0.01</button>
          <button data-d="0.01" disabled>+0.01 ▶</button>
          <button data-d="0.05" disabled>+0.05 ▶▶</button>
        </div>
      </div>

      <div class="card" id="gaku-card">
        <h2 id="gaku-title">学習モードの操作（学習開始で有効になります）</h2>
        <div id="gaku-annai"></div>
        <div id="gakuline">
          <button id="b-kaiten" disabled>回り始めた！</button>
          <button id="gaku-chushi" disabled>中止</button>
        </div>
        <button id="b-tobasu" disabled>音が鳴らない車はここを押す（音の点をとばす）</button>
        <button id="reset">学習をやり直す（学習した点をすべて消去）</button>
      </div>
    </div>
  </div>

<script>
  const $ = id => document.getElementById(id);
  const sousa = p => fetch(p, {method:"POST"});
  $("b-n").onclick = () => sousa("/n");
  $("b-zenshin").onclick = () => sousa("/zenshin");
  $("b-kotai").onclick = () => sousa("/kotai");
  $("b-jido").onclick = () => {
    if (confirm("スロットル開度学習を開始します。操作中にタイヤが回ります。4輪が浮いていることを確認しましたか？")) {
      sousa("/jido_gakushu");
    }
  };
  $("b-kaiten").onclick = () => sousa("/kaiten");
  $("b-tobasu").onclick = () => sousa("/oto_tobasu");
  document.querySelectorAll("#adj button").forEach(b => {
    b.onclick = () => sousa("/bichosei?d=" + b.dataset.d);
  });
  $("b-tan-kaishi").onclick = () => {
    if (confirm("自動探索を開始します。開度を段階的に上げ、ホイールが高速回転します。4輪が浮いていることを確認しましたか？")) {
      sousa("/tan_kaishi");
    }
  };
  $("b-tan-chushi").onclick = () => sousa("/tan_chushi");
  $("gaku-chushi").onclick = () => sousa("/gaku_chushi");
  $("reset").onclick = () => {
    if (confirm("学習した点（L点・D点・後退L点・R点）をすべて消去し、Nを基準値(10.48%)に戻します。よろしいですか？")) {
      sousa("/gaku_reset");
    }
  };
  async function poll() {
    try {
      const s = await (await fetch("/status")).json();
      $("v-n").textContent = s.n.toFixed(2);
      $("v-duty").textContent = s.ima_duty.toFixed(2);
      $("v-l").textContent = s.l_ten != null ? s.l_ten.toFixed(2) : "未学習";
      $("v-d").textContent = s.d_ten != null ? s.d_ten.toFixed(2) : "未学習";
      $("v-rl").textContent = s.rl_ten != null ? s.rl_ten.toFixed(2) : "未学習";
      $("v-r").textContent = s.r_ten != null ? s.r_ten.toFixed(2) : "未学習";
      // 微速前進・微速後退で実際に出す信号を表示する
      const kb = s.kick_byo.toFixed(1), ac = s.accel;
      $("teisoku-mae").textContent = "微速前進: " + (s.mae_kick != null
        ? `キック ${s.mae_kick.toFixed(2)}% を${kb}秒 → 低速 ${s.mae_teisoku.toFixed(2)}%` +
          (s.mae_jogen ? "（安全のため上限の「D点＋0.20%」で頭打ち）" : `（L点＋アクセル${ac}%分＝本番と同じ）`)
        : `${s.mae_teisoku.toFixed(2)}%（L点が未学習のためキック無し）`);
      $("teisoku-ushiro").textContent = "微速後退: " + (s.ushiro_teisoku == null
        ? "R点が未学習のため、ブレーキのみ"
        : s.ushiro_kick != null
        ? `キック ${s.ushiro_kick.toFixed(2)}% を${kb}秒 → 低速 ${s.ushiro_teisoku.toFixed(2)}%` +
          (s.ushiro_jogen ? "（安全のため上限の「R点−0.20%」で頭打ち）" : `（後退L点からアクセル${ac}%分）`)
        : `${s.ushiro_teisoku.toFixed(2)}%（後退L点が未学習のためキック無し）`);
      $("zone-hint").textContent = "停止ゾーン幅（R−D）: " +
        (s.zone != null ? s.zone.toFixed(2) + " %" : "--");
      $("v-rpm").textContent = s.rpm != null ? s.rpm.toFixed(0) : "--";
      if (s.tan_kiroku && s.tan_kiroku.length) {
        $("tan-kekka").textContent = "記録: " + s.tan_kiroku.map(
          k => `${k[0].toFixed(2)}%→${k[1] != null ? k[1].toFixed(0) : "?"}rpm`
        ).join("  ") + (s.tan_howa_kaido != null
          ? ` ★全開点 ≈ ${s.tan_howa_kaido.toFixed(2)}%` : "");
      }
      $("b-tan-chushi").disabled = !s.tan_jikko;
      $("b-tan-kaishi").disabled = (s.mode !== "normal");
      const m = $("mode");
      m.style.display = s.jikki ? "none" : "block";
      m.textContent = s.jikki ? "" : "テストモード（画面のみ・信号は出ていません）";
      const gaku = s.mode === "jido_gakushu";
      const oto = s.gaku_phase === "s" || s.gaku_phase === "rs";
      const sousachu = gaku && (oto || s.gaku_phase === "d" || s.gaku_phase === "r");
      document.querySelectorAll("#adj button").forEach(b => b.disabled = !sousachu);
      $("b-kaiten").disabled = !sousachu;
      $("b-kaiten").textContent = oto ? "音が鳴り始めた！" : "回り始めた！";
      $("b-tobasu").disabled = !(gaku && oto);
      $("gaku-chushi").disabled = !gaku;
      $("b-jido").disabled = s.mode !== "normal";
      $("b-n").disabled = s.mode !== "normal";
      $("b-zenshin").disabled = s.mode !== "normal";
      $("b-kotai").disabled = s.mode !== "normal";
      $("gaku-title").textContent = s.gaku_phase === "s"
        ? "学習中 1/4: 前進（音が鳴り始める点）"
        : s.gaku_phase === "d"
        ? "学習中 2/4: 前進（タイヤが回り始める点）"
        : s.gaku_phase === "kaijo"
        ? "学習中: 後退モードへ切替中..."
        : s.gaku_phase === "rs"
        ? "学習中 3/4: 後退（音が鳴り始める点）"
        : s.gaku_phase === "r"
        ? "学習中 4/4: 後退（タイヤが回り始める点）"
        : "学習モードの操作（学習開始で有効になります)";
      // 学習中の案内（◀▶の向きは、この車のスロットルの向きに合わせる）
      const mae = s.gyaku ? "◀" : "▶", ushiro = s.gyaku ? "▶" : "◀";
      $("gaku-annai").textContent = s.gaku_phase === "s"
        ? mae + " で少しずつ動かす → モーターから「ピー」と音が鳴り始めたら押す（タイヤはまだ回りません）"
        : s.gaku_phase === "d"
        ? "そのまま " + mae + " で動かす → タイヤが前進方向に回り始めたら押す"
        : s.gaku_phase === "rs"
        ? ushiro + " で少しずつ動かす → モーターから「ピー」と音が鳴り始めたら押す（タイヤはまだ回りません）"
        : s.gaku_phase === "r"
        ? "そのまま " + ushiro + " で動かす → タイヤが後退方向に回り始めたら押す"
        : "";
      const kotaichu = ["brake","kirikae","kotai"].includes(s.ichi);
      $("b-n").classList.toggle("now", s.ichi === "n" && !gaku);
      $("b-zenshin").classList.toggle("now", s.ichi === "zenshin" && !gaku);
      $("b-kotai").classList.toggle("now", kotaichu && !gaku);
      if (s.tan_jikko)
        $("msg").textContent = "自動探索中: 開度 " +
          (s.tan_kaido != null ? s.tan_kaido.toFixed(2) + "%" : "--") +
          " を計測しています...（中止ボタンでいつでも止められます）";
      else if (s.gaku_phase === "kaijo")
        $("msg").textContent = "後退モードへ切替中... そのまま待ってください";
      else if (sousachu)
        $("msg").textContent = "";
      else if (s.ichi === "brake")
        $("msg").textContent = "微速後退 1/3: ブレーキ中...";
      else if (s.ichi === "kirikae")
        $("msg").textContent = "微速後退 2/3: 後退モードへ切替中...（Nで待機）";
      else if ((s.ichi === "zenshin" || s.ichi === "kotai") && s.kick_chu)
        $("msg").textContent = (s.ichi === "zenshin" ? "微速前進" : "微速後退 3/3") +
          `: キック中（${s.ima_duty.toFixed(2)}%）`;
      else if ((s.ichi === "zenshin" || s.ichi === "kotai") && s.nokori != null)
        $("msg").textContent = (s.ichi === "zenshin" ? "微速前進" : "微速後退 3/3") +
          `: 低速 ${s.ima_duty.toFixed(2)}% で回転中... あと ${s.nokori.toFixed(0)} 秒で自動停止`;
      else
        $("msg").textContent = "";
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
            if self.path == "/n":
                app.btn_n()
            elif self.path == "/zenshin":
                app.btn_zenshin()
            elif self.path == "/kotai":
                app.btn_kotai()
            elif self.path == "/jido_gakushu":
                app.jido_gakushu_kaishi()
            elif self.path.startswith("/bichosei"):
                try:
                    sa = float(self.path.split("d=")[1])
                    if abs(sa) <= BICHOSEI_SAIDAI:
                        app.gakushu_bichosei(sa)
                except Exception:
                    pass
            elif self.path == "/kaiten":
                app.kaiten_oshita()
            elif self.path == "/oto_tobasu":
                app.oto_tobasu()
            elif self.path == "/gaku_chushi":
                app.jido_gakushu_chushi()
            elif self.path == "/gaku_reset":
                app.gakushu_risetto()
            elif self.path == "/tan_kaishi":
                app.tansaku_kaishi()
            elif self.path == "/tan_chushi":
                app.tansaku_chushi()
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
        print("\n[main] 停止処理中...")
        server.server_close()
        app.owari()
        print("[main] 終了しました")

if __name__ == "__main__":
    main()
