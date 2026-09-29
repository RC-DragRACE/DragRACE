#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   camera_test.py — カメラ接続テスト プログラム
#
#   自動車整備専門学校 自動運転実習用
#   本番プログラム rc_race.py から「目」と「画面」だけを取り出したものです。
#   モーターへの出力が無いため、実行しても車は絶対に動きません。
#   カメラの取り付け調整・白線検出・シグナル検出の確認に使います。
#   詳しい説明は「設計書_rc_race.md」を読んでください。
#
# ==============================================================================
#
#  ● このプログラムの仕組み（人間の運転にたとえると）
#
#      カメラ ＝ 目     … 前の白線を見る                → ブロック2
#      判断   ＝ 脳     … 「どっちにズレてる？」を考える → ブロック3
#      出力   ＝ 手足   … ハンドルとアクセルを動かす     → ブロック4
#
#  ● 読むならここだけでOK
#
#      ブロック1: 設定（数値を変えられる場所）
#      ブロック2: 目   （白線・シグナル・ゴールの見つけ方）
#      ブロック3: 脳   （スタート・走行・停止の判断）
#
#      ブロック5〜7 は「画面表示やWi-Fi配信の仕組み」です。
#      車の動きには関係ないので、読まなくてかまいません。
#
#  ● 使い方
#
#      1. ラズパイのターミナルで:   python3 camera_test.py
#      2. クロームブックのブラウザで:  http://rc-car-XX.local:8080
#         （XX は自分の車の番号）
#      3. 終わるときはターミナルで Ctrl と C を同時に押す
#
#  ● 大事なこと
#
#      このプログラムは全員同じものを使います。
#      それでも車ごとに走りが違うのは、ハードウェアの調整の差です。
#      まっすぐ走らないとき、直すのはプログラムではなく車です。
#
# ==============================================================================

import os
import csv
import json
import time
import math
import signal
import threading
import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np   # 数値計算ライブラリ（画像は数字のかたまりとして扱う）
import cv2           # OpenCV: カメラ画像を処理するライブラリ


# ==============================================================================
# ブロック1: 設定
# ------------------------------------------------------------------------------
#   数字を変えるとプログラムの動きが変わります。
#   ★印 … 授業で調整することがあるもの
#   無印 … 先生の指示があるまで変えないこと
# ==============================================================================

# ---- カメラの映像 ----
CAM_WIDTH  = 640      # 画像の横のサイズ（画素数）
CAM_HEIGHT = 480      # 画像の縦のサイズ
CAM_HFLIP  = False    # ★ 画面が左右逆さまのとき True にする
CAM_VFLIP  = False    # ★ 画面が上下逆さまのとき True にする

# ---- ラインの色と見つけ方 ----
LINE_IRO = "kuro"     # ★ ラインの色。 "shiro"＝白線 / "kuro"＝黒線
                      #    光沢のある床は照明や窓の映り込みが白く写り、
                      #    白線より明るくなってしまう。そういう床では
                      #    黒線を使う（映り込みが明るいほど黒線は目立つ）。
SHIRO_SHIKII = 180    # ★ 白線モード: 白と判定する明るさ (0〜255)
                      #    白線を見つけられない → 数字を小さくする
                      #    床まで白と誤解する   → 数字を大きくする
KURO_SHIKII  = 80     # ★ 黒線モード: 黒と判定する暗さ (0〜255)
                      #    黒線を見つけられない → 数字を大きくする
                      #    床の影まで黒と誤解する → 数字を小さくする

# ---- 画像のどこを見るか（ROI＝注目エリア） 0.0が画面の上端、1.0が下端 ----
ROI_CHIKAKU_UE    = 0.78   # 「近く」エリアの上端（車のすぐ前を見る）
ROI_CHIKAKU_SHITA = 0.95   # 「近く」エリアの下端
ROI_TOOKU_UE      = 0.55   # 「遠く」エリアの上端（少し先を見る）
ROI_TOOKU_SHITA   = 0.68   # 「遠く」エリアの下端

# ---- ライン有無の判定 ----
SAITEI_SHIRO_GASU   = 40   # 1つの塊(かたまり)の点がこの数より少なければ無視
                           # （小さなゴミ・汚れの足切り）
ROSUTO_KAKUTEI      = 10   # 連続これだけ「無し」が続いたら停止（走行中）

# ---- ラインの「連続性」判定（追従ゲート） ----
# 帯の中の黒を「つながった塊」ごとに分け、前フレームで追跡していた
# ライン位置の近くにある塊だけを「ラインの続き」として採用する。
# 離れた場所のマーク（ゴールマーカー等）はラインと判断しない。
TSUIZUI_GATE = 100         # ラインの続きとみなす範囲[px]（前回位置から±この幅）
HAJIME_GATE = 160          # 追跡開始時に画面中央からこの範囲[px]で探す

# ---- ゴールマーカー ----
# センターラインから左右20cm離した位置の幅5cm黒ライン（左右対）。
# 追従ゲートの外にあるのでMARK扱いになり、操舵に影響しない。
# 「ラインの左右両側に同時にMARK」＝ゴール署名。
GOAL_M_CHIKAI = 110        # マーカーと認めるラインからの距離[px]の下限
GOAL_M_TOOI = 260          # 同・上限（20cm≒165pxが中央に来る窓）
GOAL_KAKUTEI = 2           # 連続2回見えたらゴール確定

# ---- 緑シグナル（スタート合図） ----
# シグナルモニタはスタートラインの前方1〜3m、コースの右か左に置く。
# そのため探すエリアは「画面の上半分・左右いっぱい」にしてある。
SIG_UE     = 0.00          # エリア上端（0.0＝画面のいちばん上）
SIG_SHITA  = 0.50          # エリア下端（0.5＝画面の半分）
SIG_HIDARI = 0.00          # エリア左端（0.0＝画面のいちばん左）
SIG_MIGI   = 1.00          # エリア右端（1.0＝画面のいちばん右）
SIG_MIDORI_SHITA = (45,  80,  80)    # 「緑色」とみなす範囲の下限
SIG_MIDORI_UE    = (85, 255, 255)    # 上限（HSVという色の表し方）
SIG_WARIAI  = 0.02         # ★ エリアの2%以上が緑なら「点灯」と判定
                           #    シグナルに反応しない → 下げる／近づける
                           #    緑の服や物に誤反応する → 上げる
SIG_KAKUTEI = 3            # 連続3回見えたらスタート（誤発進防止）

# ---- 十字マーク（カメラの注視点。ターゲットボード合わせに使う） ----
JUJI_X = 0.50              # 画面の横位置 (0.5＝中央)
JUJI_Y = 0.55              # 画面の縦位置

# ---- 安全装置 ----
SAIDAI_SOKO_BYO = 15.0     # ★ 最大走行時間[秒]。超えたら強制停止

# ---- Wi-Fi配信（クロームブックで見る画面） ----
HTTP_PORT      = 8080      # ブラウザでアクセスする番号
HAISHIN_HABA   = 480       # 配信映像の横サイズ（小さいほど軽い）
HAISHIN_FPS    = 10        # ★ 配信のコマ数/秒（Wi-Fiが重いときは下げる）
HAISHIN_GASHITSU = 70      # 映像の画質 (1〜100)

# ---- 記録 ----
LOG_FOLDER = "logs"        # 走行データ(CSV)の保存先
SETTEI_FILE = "kuruma_settei.json"  # この車専用の個体設定ファイル

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
# ブロック2: 目 —— カメラ画像から3つのものを見つける
# ------------------------------------------------------------------------------
#   (a) 白線: 車がラインからどれだけズレているか
#   (b) ゴールライン: 進行方向と直角の太い横線
#   (c) 緑シグナル: スタートの合図
#
#   画像は「数字が並んだ表」です。明るい場所ほど大きい数字になっています。
#   なので「白線を探す」＝「大きい数字の場所を探す」という計算になります。
# ==============================================================================

def me_de_miru(gazou, ato_zure=None):
    """1枚の画像を調べて、見つけた結果を辞書（名前付きの入れ物）で返す。
       ato_zure: 前フレームまで追跡していたラインの横位置[px]（None=追跡開始）"""

    takasa, haba = gazou.shape[:2]      # 画像の縦横サイズ
    chuo = haba // 2                    # 画面の中央（この位置にラインが来るのが理想）

    # --- 手順1: 白黒画像にして、「ラインの色の場所だけ」を取り出す ---
    shirokuro = cv2.cvtColor(gazou, cv2.COLOR_BGR2GRAY)          # カラー→白黒
    if LINE_IRO == "kuro":
        _, sen_dake = cv2.threshold(shirokuro, KURO_SHIKII,
                                    255, cv2.THRESH_BINARY_INV)
    else:
        _, sen_dake = cv2.threshold(shirokuro, SHIRO_SHIKII,
                                    255, cv2.THRESH_BINARY)

    # --- 手順2: 帯の中の黒を「つながった塊」ごとに分け、ラインを選ぶ ---
    # ゲート（前回のライン位置±TSUIZUI_GATE）の中にある塊だけがラインの続き。
    # ゲートの外の塊は「マーク」として別に記録する（操舵に使わない）。
    def obi_wo_shiraberu(ue, shita, gate_chuo, gate_haba):
        y0, y1 = int(takasa * ue), int(takasa * shita)
        obi = sen_dake[y0:y1, :]
        kosu, labels, stats, centers = cv2.connectedComponentsWithStats(obi)
        chikai = None
        marks = []
        for i in range(1, kosu):                     # 0は背景
            if stats[i, cv2.CC_STAT_AREA] < SAITEI_SHIRO_GASU:
                continue                             # 小さなゴミは無視
            zure = centers[i][0] - chuo
            if abs(zure - gate_chuo) <= gate_haba:   # ゲート内 → ライン候補
                if chikai is None or abs(zure - gate_chuo) < abs(chikai[0] - gate_chuo):
                    chikai = (zure, stats[i, cv2.CC_STAT_WIDTH] / haba)
            else:                                    # ゲート外 → マーク
                marks.append(round(float(zure), 1))
        if chikai is None:
            return None, 0.0, marks
        return chikai[0], chikai[1], marks

    if ato_zure is None:
        gate_chuo, gate_haba = 0.0, HAJIME_GATE      # 追跡開始: 中央から探す
    else:
        gate_chuo, gate_haba = ato_zure, TSUIZUI_GATE

    zure_chikaku, hirogari_chikaku, marks = obi_wo_shiraberu(
        ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, gate_chuo, gate_haba)
    zure_tooku, _, _ = obi_wo_shiraberu(
        ROI_TOOKU_UE, ROI_TOOKU_SHITA, gate_chuo, gate_haba * 1.5)

    # --- 手順3: ラインの傾きを計算する ---
    katamuki = None
    if zure_chikaku is not None and zure_tooku is not None:
        y_chikaku = takasa * (ROI_CHIKAKU_UE + ROI_CHIKAKU_SHITA) / 2
        y_tooku = takasa * (ROI_TOOKU_UE + ROI_TOOKU_SHITA) / 2
        katamuki = math.degrees(
            math.atan2(zure_tooku - zure_chikaku, y_chikaku - y_tooku))

    # --- 手順4: ゴールマーカーの判定 ---
    # ラインから見て左右両側の決まった距離にMARKがあればゴール
    goal_hidari = goal_migi = False
    if zure_chikaku is not None:
        for m in marks:
            sa = m - zure_chikaku                    # ラインから見たMARKの位置
            if -GOAL_M_TOOI <= sa <= -GOAL_M_CHIKAI:
                goal_hidari = True
            elif GOAL_M_CHIKAI <= sa <= GOAL_M_TOOI:
                goal_migi = True
    goal_mieta = goal_hidari and goal_migi

    # --- 手順5: 緑シグナルの判定 ---
    y0, y1 = int(takasa * SIG_UE), int(takasa * SIG_SHITA)
    x0, x1 = int(haba * SIG_HIDARI), int(haba * SIG_MIGI)
    sig_area = gazou[y0:y1, x0:x1]
    hsv = cv2.cvtColor(sig_area, cv2.COLOR_BGR2HSV)
    midori = cv2.inRange(hsv, np.array(SIG_MIDORI_SHITA),
                         np.array(SIG_MIDORI_UE))
    midori_wariai = cv2.countNonZero(midori) / midori.size
    midori_tento = midori_wariai >= SIG_WARIAI

    # --- 見つけた結果をまとめて返す ---
    return {
        "zure": None if zure_chikaku is None else round(float(zure_chikaku), 1),
        "zure_tooku": None if zure_tooku is None else round(float(zure_tooku), 1),
        "katamuki": None if katamuki is None else round(katamuki, 1),
        "line_mieru": zure_chikaku is not None,
        "marks": marks,
        "goal": bool(goal_mieta),
        "goal_hidari": bool(goal_hidari),
        "goal_migi": bool(goal_migi),
        "hirogari": round(hirogari_chikaku, 2),
        "midori": bool(midori_tento),
        "midori_wariai": round(midori_wariai, 3),
    }


def nou_de_handan(app, kekka, ima):
    """状態を確認し、必要なら次の状態に切り替える
       app  : プログラム全体の情報が入った入れ物
       kekka: ブロック2「目」が見つけた結果
       ima  : 現在時刻
    """

    # ---------- ARMED: 緑シグナルを待っている ----------
    if app.state == "ARMED":
        if kekka["midori"]:
            app.midori_kaisu += 1        # 緑が見えた回数を数える
        else:
            app.midori_kaisu = 0         # 途切れたらリセット
        # 1回だけの誤検出で発進しないよう、連続で見えたときだけスタート
        if app.midori_kaisu >= SIG_KAKUTEI:
            app.state = "RUNNING"
            app.start_jikoku = ima
            app.goal_kaisu = 0
            app.rosuto_kaisu = 0
            app.kiroku_kaishi()
            print("[脳] スタート!")

    # ---------- RUNNING: 走行中 ----------
    elif app.state == "RUNNING":
        keika = ima - app.start_jikoku            # スタートからの経過時間
        app.kiroku_tsuika(keika, kekka)           # 走行データを1行記録

        # ※第2段階ではここに「ズレに応じてハンドルを切る」計算が入ります。
        #    handoru = -K * zure  のような、ズレを打ち消す向きの計算です。

        # --- 終わりの判定 ---
        app.goal_kaisu = app.goal_kaisu + 1 if kekka["goal"] else 0
        app.rosuto_kaisu = app.rosuto_kaisu + 1 if not kekka["line_mieru"] else 0

        riyu = None
        if app.goal_kaisu >= GOAL_KAKUTEI:
            riyu = "goal"          # ゴールラインを通過した
        elif app.rosuto_kaisu >= ROSUTO_KAKUTEI:
            riyu = "line_lost"     # 白線を見失った
        elif keika >= SAIDAI_SOKO_BYO:
            riyu = "timeout"       # 時間切れ（安全装置）

        if riyu:
            app.state = "FINISHED"
            app.goal_time = keika
            app.owari_riyu = riyu
            app.te_wo_tomeru()
            app.kiroku_shuryo()
            print(f"[脳] 終了({riyu}) タイム {keika:.2f} 秒")

    # IDLE と FINISHED のときは何もしない（ボタン操作を待つだけ）


# ==============================================================================
# ブロック4: 手足 —— ハンドルとアクセルを動かす
# ------------------------------------------------------------------------------
#   【第1段階】このブロックはまだ空っぽです。車は絶対に動きません。
#   第2段階で、ここに pigpio というライブラリを使った
#   サーボ（ハンドル）とESC（アクセル）への信号出力が入ります。
# ==============================================================================

class TeAshi:
    def __init__(self):
        print("[手足] 第1段階: モーター出力は無効です（車は動きません）")

    def neutral(self):
        """ハンドルまっすぐ・アクセルゼロにする（第2段階で実装）"""
        pass

    def close(self):
        pass


# ==============================================================================
# ==============================================================================
#
#   ここから下は「仕組み」の部分です。
#
#   カメラを動かす・映像をWi-Fiで配信する・ブラウザの画面を作る、といった
#   処理が書かれています。車の走り方には関係ないので、読まなくてOKです。
#   （興味がある人は読んでみてください。質問も歓迎です）
#
# ==============================================================================
# ==============================================================================


# ==============================================================================
# ブロック5: カメラ（picamera2 の準備と画像の取り込み）
# ==============================================================================

class Kamera:
    def __init__(self):
        self.picam2 = None
        try:
            from picamera2 import Picamera2
            from libcamera import Transform
            self.picam2 = Picamera2()
            settei = self.picam2.create_video_configuration(
                main={"size": (CAM_WIDTH, CAM_HEIGHT), "format": "RGB888"},
                transform=Transform(hflip=int(CAM_HFLIP), vflip=int(CAM_VFLIP)),
            )
            self.picam2.configure(settei)
            self.picam2.start()
            time.sleep(0.5)   # カメラの明るさ自動調整が落ち着くのを待つ
            print("[カメラ] 起動しました")
        except Exception as e:
            print(f"[カメラ] 起動できません ({e})")
            print("[カメラ] テスト映像で動きます（カメラ無しの開発用）")

    def toru(self):
        """1枚撮影して返す"""
        if self.picam2 is not None:
            return self.picam2.capture_array()
        # ---- カメラが無いときのテスト映像（中央に白線を描いた絵） ----
        img = np.full((CAM_HEIGHT, CAM_WIDTH, 3), 60, np.uint8)
        cv2.line(img, (CAM_WIDTH // 2 + 15, CAM_HEIGHT),
                 (CAM_WIDTH // 2 - 10, int(CAM_HEIGHT * 0.4)), (255, 255, 255), 12)
        time.sleep(0.033)
        return img

    def close(self):
        if self.picam2 is not None:
            self.picam2.stop()


def gamen_ni_kaku(gazou, kekka, state, ato=None):
    """検出結果を映像に描き込む（ブラウザで見る映像を作る）
       ato: 追跡中のライン位置[px]（ゲート枠の中心。None=中央から探索中）"""
    img = gazou.copy()
    takasa, haba = img.shape[:2]
    chuo = haba // 2

    def obi(ue, shita, iro):
        y0, y1 = int(takasa * ue), int(takasa * shita)
        kasane = img.copy()
        cv2.rectangle(kasane, (0, y0), (haba, y1), iro, -1)
        cv2.addWeighted(kasane, 0.25, img, 0.75, 0, img)
        cv2.rectangle(img, (0, y0), (haba, y1), iro, 1)
        return (y0 + y1) // 2

    y_chikaku = obi(ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, (255, 128, 0))   # 青の帯
    y_tooku = obi(ROI_TOOKU_UE, ROI_TOOKU_SHITA, (255, 220, 0))         # 水色の帯

    # 追従ゲート（ラインの続きを探す枠）を黄色で描く
    def gate_waku(ue, shita, g_haba):
        g_chuo = chuo + int(ato if ato is not None else 0)
        gy0, gy1 = int(takasa * ue), int(takasa * shita)
        gx0 = max(0, g_chuo - g_haba)
        gx1 = min(haba - 1, g_chuo + g_haba)
        cv2.rectangle(img, (gx0, gy0 + 2), (gx1, gy1 - 2), (0, 255, 255), 1)
        cv2.line(img, (g_chuo, gy0 + 2), (g_chuo, gy0 + 12), (0, 255, 255), 2)

    gate_chikaku = TSUIZUI_GATE if ato is not None else HAJIME_GATE
    gate_waku(ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, gate_chikaku)
    gate_waku(ROI_TOOKU_UE, ROI_TOOKU_SHITA, int(gate_chikaku * 1.5))

    # ゲートの外の塊＝MARK（紫の丸）。ゴールマーカーはここに映る
    for m in kekka.get("marks", []):
        cv2.circle(img, (int(chuo + m), y_chikaku), 8, (255, 0, 255), 2)
        cv2.putText(img, "MARK", (int(chuo + m) - 22, y_chikaku - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)

    # シグナル判定エリア（黄色の枠。緑が点くと緑の枠になる）
    y0, y1 = int(takasa * SIG_UE), int(takasa * SIG_SHITA)
    x0, x1 = int(haba * SIG_HIDARI), int(haba * SIG_MIGI)
    waku_iro = (0, 255, 0) if kekka["midori"] else (0, 200, 255)
    cv2.rectangle(img, (x0, y0), (x1, y1), waku_iro, 2)

    # 十字マーク（カメラの注視点）
    jx, jy = int(haba * JUJI_X), int(takasa * JUJI_Y)
    cv2.line(img, (jx - 20, jy), (jx + 20, jy), (0, 255, 0), 2)
    cv2.line(img, (jx, jy - 20), (jx, jy + 20), (0, 255, 0), 2)

    # 見つけたラインの位置（オレンジの点と線）
    if kekka["zure"] is not None:
        cv2.circle(img, (int(chuo + kekka["zure"]), y_chikaku), 8, (0, 128, 255), -1)
    if kekka["zure_tooku"] is not None:
        cv2.circle(img, (int(chuo + kekka["zure_tooku"]), y_tooku), 8, (0, 200, 255), -1)
    if kekka["zure"] is not None and kekka["zure_tooku"] is not None:
        cv2.line(img, (int(chuo + kekka["zure"]), y_chikaku),
                 (int(chuo + kekka["zure_tooku"]), y_tooku), (0, 128, 255), 2)

    cv2.putText(img, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (255, 255, 255), 2, cv2.LINE_AA)
    return img


# ==============================================================================
# ブロック6: プログラム全体のまとめ役 と Webサーバ
# ==============================================================================

class RaceApp:
    """プログラム全体の情報（状態・検出結果・記録）をまとめて持つ入れ物"""

    def __init__(self):
        self.kamera = Kamera()
        self.teashi = TeAshi()
        self.lock = threading.Lock()   # 複数の処理が同時に書き換えないための鍵
        self.ugoiteru = True

        self.state = "IDLE"
        self.kekka = {}
        self.fps = 0.0
        self.start_jikoku = None
        self.goal_time = None
        self.owari_riyu = ""
        self.haishin_jpeg = None

        self.midori_kaisu = 0
        self.goal_kaisu = 0
        self.rosuto_kaisu = 0
        self.line_ato = None      # 追跡中のライン位置[px]（None=次フレームで再取得）
        self._csv = None
        self._csv_file = None

    # ---- ブラウザのボタンから呼ばれる ----
    def botan_arm(self):
        with self.lock:
            if self.state == "IDLE":
                self.state = "ARMED"
                self.midori_kaisu = 0
                self.line_ato = None      # 追跡をリセット（中央から探し直す）
                print("[ボタン] ARM: 緑シグナル待ち")

    def botan_stop(self):
        with self.lock:
            mae = self.state
            self.state = "IDLE"
            self.teashi.neutral()
            self.kiroku_shuryo()
            if mae != "IDLE":
                print(f"[ボタン] STOP: {mae} → IDLE")

    def te_wo_tomeru(self):
        self.teashi.neutral()

    # ---- 走行データの記録（あとで採点・分析に使う） ----
    def kiroku_kaishi(self):
        os.makedirs(LOG_FOLDER, exist_ok=True)
        namae = datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S.csv")
        path = os.path.join(LOG_FOLDER, namae)
        self._csv_file = open(path, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["時間[秒]", "横ズレ[px]", "遠くのズレ[px]",
                            "傾き[度]", "ライン", "ゴール"])
        print(f"[記録] {path} に保存します")

    def kiroku_tsuika(self, keika, kekka):
        if self._csv:
            self._csv.writerow([f"{keika:.3f}", kekka["zure"], kekka["zure_tooku"],
                                kekka["katamuki"], int(kekka["line_mieru"]),
                                int(kekka["goal"])])

    def kiroku_shuryo(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv = None

    # ---- メインループ: 撮る → 見る → 判断する を繰り返す ----
    def mawasu(self):
        mae_jikoku = time.time()
        fps_goukei, fps_kaisu = 0.0, 0
        while self.ugoiteru:
            gazou = self.kamera.toru()          # 1. 撮る
            with self.lock:
                ato = self.line_ato
            kekka = me_de_miru(gazou, ato)      # 2. 見る（ブロック2）

            ima = time.time()
            dt = ima - mae_jikoku
            mae_jikoku = ima
            if dt > 0:
                fps_goukei += 1.0 / dt
                fps_kaisu += 1
                if fps_kaisu >= 10:
                    self.fps = fps_goukei / fps_kaisu
                    fps_goukei, fps_kaisu = 0.0, 0

            with self.lock:
                if kekka["line_mieru"]:
                    self.line_ato = kekka["zure"]   # 追跡位置を更新
                nou_de_handan(self, kekka, ima)  # 3. 判断する（ブロック3）
                self.kekka = kekka
                state = self.state

            # 4. ブラウザ用の映像を作る（縮小して軽くする）
            e = gamen_ni_kaku(gazou, kekka, state, ato)
            h2 = int(e.shape[0] * HAISHIN_HABA / e.shape[1])
            chiisai = cv2.resize(e, (HAISHIN_HABA, h2))
            ok, jpeg = cv2.imencode(".jpg", chiisai,
                                    [cv2.IMWRITE_JPEG_QUALITY, HAISHIN_GASHITSU])
            if ok:
                with self.lock:
                    self.haishin_jpeg = jpeg.tobytes()

        self.kamera.close()
        self.teashi.close()

    # ---- ブラウザに送る「いまの状態」データ ----
    def joutai_json(self):
        with self.lock:
            keika = None
            if self.state == "RUNNING" and self.start_jikoku:
                keika = round(time.time() - self.start_jikoku, 2)
            return json.dumps({
                "state": self.state,
                "zure": self.kekka.get("zure"),
                "katamuki": self.kekka.get("katamuki"),
                "line_mieru": bool(self.kekka.get("line_mieru")),
                "midori": bool(self.kekka.get("midori")),
                "goal_mieru": bool(self.kekka.get("goal")),
                "goal_hidari": bool(self.kekka.get("goal_hidari")),
                "goal_migi": bool(self.kekka.get("goal_migi")),
                "fps": round(self.fps, 1),
                "keika": keika,
                "goal_time": None if self.goal_time is None else round(self.goal_time, 2),
                "owari_riyu": self.owari_riyu,
            })


# ---- ブラウザに表示する画面（HTML） ----
GAMEN_HTML = """<!DOCTYPE html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>RC RACE</title>
<style>
  :root {
    --bg:#171a1e; --panel:#21262c; --line:#343b44; --text:#e8eaed; --dim:#8a939e;
    --idle:#5b8dbf; --armed:#d9a13b; --run:#43b06a; --fin:#b06ad9;
  }
  * { box-sizing:border-box; margin:0; padding:0; }
  body { background:var(--bg); color:var(--text);
    font-family:"Hiragino Sans","Noto Sans JP",sans-serif;
    height:100vh; padding:12px; display:grid;
    grid-template-columns:1fr 360px; gap:12px; }
  /* --- 左: カメラ映像 --- */
  #left { min-width:0; display:flex; align-items:center; justify-content:center; }
  #video { max-width:100%; max-height:calc(100vh - 24px);
    border-radius:10px; border:1px solid var(--line); display:block; }
  /* --- 右: 状態・数値・ボタン --- */
  #right { display:flex; flex-direction:column; gap:10px; min-height:0; }
  #statebar { padding:16px; border-radius:10px; text-align:center;
    font-size:30px; font-weight:700; letter-spacing:2px;
    background:var(--idle); color:#fff; transition:background .2s; }
  #statebar.ARMED { background:var(--armed); }
  #statebar.RUNNING { background:var(--run); }
  #statebar.FINISHED { background:var(--fin); }
  #metrics { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .metric { background:var(--panel); border:1px solid var(--line);
    border-radius:10px; padding:14px 10px; text-align:center; }
  .metric .v { font-size:30px; font-weight:700; font-variant-numeric:tabular-nums; }
  .metric .k { font-size:13px; color:var(--dim); margin-top:4px; }
  .ok { color:var(--run); } .ng { color:#d95b5b; }
  #buttons { display:flex; flex-direction:column; gap:10px; margin-top:auto; }
  button { padding:22px; font-size:24px; font-weight:700;
    border:none; border-radius:10px; color:#fff; cursor:pointer; }
  #btn-arm { background:var(--armed); }
  #btn-stop { background:#c94f4f; }
  button:active { filter:brightness(.85); }
  #result { text-align:center; font-size:17px; color:var(--dim); min-height:22px; }
  /* --- 縦長画面（スマホ等）では上下に並べ替える --- */
  @media (max-width:820px) {
    body { grid-template-columns:1fr; height:auto; }
    #video { max-height:none; width:100%; }
    #buttons { flex-direction:row; margin-top:0; }
    button { flex:1; }
  }
</style>
</head>
<body>
  <div id="left"><img id="video" src="/stream" alt="camera"></div>
  <div id="right">
    <div id="statebar">IDLE</div>
    <div id="metrics">
      <div class="metric"><div class="v" id="m-zure">--</div><div class="k">横ズレ [px]</div></div>
      <div class="metric"><div class="v" id="m-katamuki">--</div><div class="k">ライン傾き [°]</div></div>
      <div class="metric"><div class="v" id="m-line">--</div><div class="k">ライン検出</div></div>
      <div class="metric"><div class="v" id="m-sig">--</div><div class="k">シグナル</div></div>
      <div class="metric"><div class="v" id="m-goal">--</div><div class="k">ゴールマーカー</div></div>
      <div class="metric"><div class="v" id="m-fps">--</div><div class="k">fps</div></div>
    </div>
    <div id="result"></div>
    <div id="buttons">
      <button id="btn-arm">ARM — スタート待機</button>
      <button id="btn-stop">STOP — 停止</button>
    </div>
  </div>
<script>
  const $ = id => document.getElementById(id);
  $("btn-arm").onclick = () => fetch("/arm", {method:"POST"});
  $("btn-stop").onclick = () => fetch("/stop", {method:"POST"});
  const riyu = { goal:"ゴール!", line_lost:"ラインロスト", timeout:"タイムアウト" };
  async function poll() {
    try {
      const s = await (await fetch("/status")).json();
      const bar = $("statebar");
      bar.className = s.state;
      bar.textContent = (s.state === "RUNNING" && s.keika != null)
        ? `RUNNING ${s.keika.toFixed(2)} s` : s.state;
      $("m-zure").textContent = s.zure ?? "--";
      $("m-katamuki").textContent = s.katamuki ?? "--";
      $("m-line").textContent = s.line_mieru ? "OK" : "NG";
      $("m-line").className = "v " + (s.line_mieru ? "ok" : "ng");
      $("m-sig").textContent = s.midori ? "GREEN" : "--";
      $("m-sig").className = "v " + (s.midori ? "ok" : "");
      $("m-goal").textContent = s.goal_mieru ? "OK"
        : (s.goal_hidari ? "左" : (s.goal_migi ? "右" : "--"));
      $("m-goal").className = "v " + (s.goal_mieru ? "ok" : "");
      $("m-fps").textContent = s.fps ?? "--";
      $("result").textContent = (s.state === "FINISHED")
        ? `${riyu[s.owari_riyu] ?? s.owari_riyu}  タイム ${s.goal_time ?? "--"} 秒`
        : "";
    } catch (e) { /* 通信が切れても次の更新で復帰する */ }
  }
  setInterval(poll, 300);
  poll();
</script>
</body>
</html>
"""


def handler_wo_tsukuru(app):
    """ブラウザからのアクセスに応答する処理"""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass   # アクセスのたびに出るログを止める

        def do_GET(self):
            if self.path == "/":                       # 操作画面
                body = GAMEN_HTML.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            elif self.path == "/status":               # 状態データ(JSON)
                body = app.joutai_json().encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            elif self.path == "/stream":               # カメラ映像(MJPEG)
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                kankaku = 1.0 / HAISHIN_FPS
                try:
                    while app.ugoiteru:
                        with app.lock:
                            jpeg = app.haishin_jpeg
                        if jpeg:
                            self.wfile.write(b"--frame\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                            self.wfile.write(jpeg)
                            self.wfile.write(b"\r\n")
                        time.sleep(kankaku)
                except (BrokenPipeError, ConnectionResetError):
                    pass   # ブラウザが閉じられた（正常）
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            if self.path == "/arm":
                app.botan_arm()
            elif self.path == "/stop":
                app.botan_stop()
            else:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Length", "0")
            self.end_headers()

    return Handler


# ==============================================================================
# ブロック7: main —— プログラムの入口
# ==============================================================================

def main():
    app = RaceApp()

    # Webサーバを別スレッド（並行処理）で動かす
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler_wo_tsukuru(app))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[web] ブラウザで http://<この車の名前>.local:{HTTP_PORT} を開いてください")

    # Ctrl+C が押されたら安全に止める
    def tomeru(*_):
        print("\n[main] 停止処理中...")
        app.ugoiteru = False
        app.botan_stop()

    signal.signal(signal.SIGINT, tomeru)
    signal.signal(signal.SIGTERM, tomeru)

    app.mawasu()          # メインループ開始（Ctrl+Cで抜ける）
    server.shutdown()
    print("[main] 終了しました")


if __name__ == "__main__":
    main()
