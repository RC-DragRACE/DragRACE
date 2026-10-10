#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
#
#   rc_race.py — カメラ黒線検出 ドラッグレース 本番プログラム
#
#   自動車整備専門学校 自動運転実習用
#   詳しい説明は「設計書_rc_race.md」を読んでください。
#
# ==============================================================================
#
#  ● このプログラムの仕組み（人間の運転にたとえると）
#
#      カメラ ＝ 目     … 前の黒線を見る                → ブロック2
#      判断   ＝ 脳     … ズレの修正・発進・停止を考える → ブロック3
#      出力   ＝ 手足   … ハンドルとアクセルを動かす     → ブロック4
#
#  ● 走らせる前に必要なこと
#
#      servo_test.py と throttle_test.py での調整・学習が済んでいて、
#      kuruma_settei.json がこの車のフォルダにあること。
#      無い車は安全のため、走行開始（ARM）できません。
#
#  ● 使い方
#
#      1. ラズパイのターミナルで:   python3 rc_race.py
#      2. ブラウザで:  http://<この車のIPアドレス>:8080
#      3. 車をスタートラインに置き、ARMボタン → 緑シグナルで自動発進
#      4. 止めたいときはいつでも STOPボタン（またはターミナルで Ctrl+C）
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

import numpy as np
import cv2

# ==============================================================================
# ブロック1: 設定
#   ★印 … 授業で調整することがあるもの（教員の指示に従うこと）
#   無印 … 変えないこと
#   各パラメータに「意味」と「変えるとどうなるか」を書いてあります。
# ==============================================================================

# ------------------------------------------------------------------------------
# カメラの映像
# ------------------------------------------------------------------------------
CAM_WIDTH  = 640      # 検出に使う画像の横サイズ[画素]。大きくすると細かく見える
CAM_HEIGHT = 480      #   が処理が重くなりfpsが落ちる。変えないこと
CAM_HFLIP  = False    # ★ 映像の左右反転。画面が左右逆さまのとき True にする
CAM_VFLIP  = False    # ★ 映像の上下反転。画面が上下逆さまのとき True にする

# ------------------------------------------------------------------------------
# ライン検出（黒線をどう見つけるか）
# ------------------------------------------------------------------------------
LINE_IRO = "kuro"     # ラインの色。"kuro"＝黒線 / "shiro"＝白線。
                      # この実習は黒線（光沢床の映り込みは白く写るため）
SHIRO_SHIKII = 180    # 白線モード用のしきい値。黒線モードでは使われない
KURO_SHIKII  = 80     # ★ 「これより暗い画素＝ライン」とみなす境目 (0=真っ黒〜255=真っ白)
                      #   大きくする → 薄暗い色まで拾う。ラインを見失いにくいが、
                      #                床の影や汚れを誤ってラインと認識しやすくなる
                      #   小さくする → 真っ黒だけを拾う。誤検出は減るが、
                      #                照明の当たり方次第で本物のラインまで見失う

ROI_CHIKAKU_UE    = 0.78   # 「近くの帯」の上端（0.0=画面の上端〜1.0=下端）
ROI_CHIKAKU_SHITA = 0.95   # 「近くの帯」の下端。ハンドル修正の基準になる帯
ROI_TOOKU_UE      = 0.55   # 「遠くの帯」の上端。ラインの傾き計測用
ROI_TOOKU_SHITA   = 0.68   # 「遠くの帯」の下端
                           #   帯を上（数値を小さく）へ動かす → より遠くを見る。
                           #   早めに反応できるが、遠くはブレの影響が大きい。
                           #   通常は変えないこと（カメラ取付角とセットの値）

SAITEI_SHIRO_GASU = 40     # 1つの塊(かたまり)の点がこの数より少なければ無視
                           # （小さなゴミ・汚れの足切り）

# --- ラインの「連続性」判定（追従ゲート） ---
# 帯の中の黒を「つながった塊」ごとに分け、前フレームで追跡していた
# ライン位置の近くにある塊だけを「ラインの続き」として採用する。
# 離れた場所のマークはラインと判断せず、操舵に影響させない。
TSUIZUI_GATE = 100         # ★ ラインの続きとみなす範囲[px]（前回位置から±この幅）
                           #   大きくする → 急なズレにも追従するが、近くのマークを
                           #                ラインと誤認しやすくなる
                           #   小さくする → マークに強いが、大きく蛇行したとき
                           #                本物のラインを見失いやすい
HAJIME_GATE = 160          # 追跡開始時（スタート時）に画面中央からこの範囲[px]で
                           # ラインを探す。車はライン上に置かれている前提
                           #   大きくする → 小さなゴミを無視できるが、細いラインも無視
                           #   小さくする → 敏感になるがノイズを拾いやすい
ROSUTO_KAKUTEI = 10        # 「ライン無し」がこのフレーム数続いたら停止（約0.33秒@30fps）
                           #   大きくする → 一瞬の見失いで止まらないが、
                           #                コースアウト時の停止が遅れて危険
                           #   小さくする → すぐ止まるが、ちらつきで誤停止しやすい

# ------------------------------------------------------------------------------
# ゴールの考え方（重要）
#   ゴールは「ゴールマーカー」で判定する。
#   マーカー = センターラインから左右20cm離した位置に置く幅5cmの黒ライン（左右対）。
#   追従ゲートの外にあるのでラインとは区別され（MARK扱い）、操舵を乱さない。
#   「ラインの左右両側に同時にMARKが見える」= ゴール署名 → ブレーキ開始。
#   ルール: マーカー通過後 5m以内に停止すること。
#   センターラインはゴール後も5m以上続け、その先で切る。
#   → 万一マーカーを見逃しても、ライン切断でロスト停止する（バックアップ）。
#   公式タイムはコース側の計測装置が測る（車のタイムは参考値）。
# ------------------------------------------------------------------------------
GOAL_M_CHIKAI = 110   # マーカーと認めるラインからの距離[px]の下限
                      #   追従ゲート(±100px)より外であること。近すぎる汚れを除外
GOAL_M_TOOI = 260     # 同・上限。20cm≒165px が中央に来る窓
                      #   遠すぎる（コース外の）黒を除外
GOAL_KAKUTEI = 2      # 左右同時MARKが連続このフレーム数でゴール確定
                      #   大きくする → 誤判定は減るが、高速通過で見逃すリスク

# ------------------------------------------------------------------------------
# 計測サーバへの通過通知（集計の自動化用。無くても走行に影響しない）
# ------------------------------------------------------------------------------
KEISOKU_URL = ""      # ★ 計測サーバのURL（例: "http://192.168.18.250:9000/goal"）
                      #   空欄なら通知しない。走行ロジックには一切影響しない
KURUMA_NAMAE = ""     # ★ この車の名前（例: "rc-car-07"）。空欄ならホスト名を使う

# ------------------------------------------------------------------------------
# 緑シグナル（スタート合図・画面の上半分全体で探す）
# ------------------------------------------------------------------------------
SIG_UE, SIG_SHITA = 0.00, 0.50       # 探すエリアの上端・下端（上半分）
SIG_HIDARI, SIG_MIGI = 0.00, 1.00    # 探すエリアの左端・右端（全幅）
SIG_MIDORI_SHITA = (45, 80, 80)      # 「緑色」とみなすHSV範囲の下限
SIG_MIDORI_UE    = (85, 255, 255)    # 上限。通常は変えないこと
SIG_WARIAI  = 0.02    # ★ エリアの何割が緑なら「点灯」か (0.02＝2%)
                      #   大きくする → 誤反応しにくいが、遠く・小さいモニタに
                      #                反応しなくなる（モニタを近づけて対処）
                      #   小さくする → 遠くでも反応するが、緑の服・物で誤発進の恐れ
SIG_KAKUTEI = 3       # 連続このフレーム数で発進確定（一瞬のノイズでのフライング防止）

# ------------------------------------------------------------------------------
# 十字マーク（カメラ注視点。ターゲットボード合わせ用）
# ------------------------------------------------------------------------------
JUJI_X, JUJI_Y = 0.50, 0.55   # 画面上の位置（0.5＝中央）

# ------------------------------------------------------------------------------
# 走行 —— アクセル（0〜100%で考える。ここがレースの核心）
#
#   アクセル%
#   ACCEL_SAIDAI ┤            ┌──────────────  ← 上限
#                │          ／
#                │        ／  ← 坂の急さ＝ACCEL_RAMPU
#   ACCEL_HASSHIN┤──────／
#         0      ┼──┬─────────────→ 時間
#                 発進
#
#   【アクセル%の目盛り】
#     0%   = L点（モーターから音が鳴り始める点）。throttle_test で学習する。
#            動き出した車が回り続けられる、いちばん弱い信号
#     約8% = D点（止まった状態からタイヤが回り始める点）。車ごとに少し違う
#     100% = L点から ZENKAI_HABA だけ前進側（モーター回転が頭打ちになる点）
#     L点を学習していない車は、D点が0%になる（低速では走れない）
#
#   【発進キック】モーターは動き出しに一番力が要る。発進から KICK_BYO 秒間は
#     最低でもD点の信号を出して動き出させ、そのあと設定したアクセルへ下げる。
#
#   【初期値について】ここの初期値は「はじめて走らせても安全な、慎重な走り」
#   になるように低めにしてある。kuruma_settei.json に書かなければこの値で走る。
#   まずこの値で完走できることを確認し、そこから少しずつ上げていくこと。
# ------------------------------------------------------------------------------
ACCEL_SAIDAI  = 5     # ★ 走行中の最大アクセル[%]（個別セッティング）
                      #   上げる → 最高速が上がる。上限をどこに置くかも戦略
                      #   上げたらハンドルのゲインとブレーキも合わせ直すこと
                      #   初期値5は慎重走行用（D点より弱い、ゆっくりした走り）。
                      #   途中で止まってしまう車は1ずつ上げる。
                      #   完走を確認してから少しずつ上げていく
                      #   発進アクセル(ACCEL_HASSHIN)がこれより大きくても、
                      #   この値で頭打ちになる（最大を超えることはない）
ACCEL_HASSHIN = 20    # ★ 発進の瞬間に踏むアクセル[%]
                      #   初期値20は慎重走行用。発進しない車は5ずつ上げる
                      #   大きくする → 発進が鋭いが、後輪が空転して
                      #                横を向きやすい（グリップ次第）
                      #   小さくする → 確実に発進するがタイムをロスする
ACCEL_RAMPU   = 20    # ★ アクセルの踏み増し速度[%/秒]
                      #   大きくする → 早く全開に達する（空転・蛇行のリスク増）
                      #   小さくする → じわっと加速（安定するが遅い）
                      #   例: 発進30% + 毎秒50% → 約1.4秒で100%に到達
                      #   （初期値では最大5%なので、発進した瞬間から5%で一定）

ZENKAI_HABA = 2.75    # アクセル100%のとき、0%の点から前進側へ出す信号量[%]。
                      # 基準車のエンコーダ実測（モーター回転の飽和点）で決めた値。
                      # これ以上増やしても速くならないことを確認済み。変えないこと

ZENSHIN_SAITEI = None # L点: アクセル0%のときの信号[%]（学習値・個別）
                      #   throttle_test の学習で kuruma_settei.json に保存される。
                      #   手で書き換えないこと。
                      #   無い(None) → D点（回り始める点）が0%になる
                      #   NとD点の間の値だけ有効。それ以外は無視してD点を使う
KICK_BYO = 0.3        # ★ 発進キックの時間[秒]。L点を学習した車だけ働く
                      #   発進からこの秒数は、最低でもD点の信号を出す
                      #   （モーターは動き出しに一番力が要るため）
                      #   長くする → 確実に動き出すが、出だしが速くなる
                      #   短くする → 出だしが穏やかだが、動き出せないことがある

# ------------------------------------------------------------------------------
# ステアリング制御
# ------------------------------------------------------------------------------
STEER_P_GAIN = 0.007  # ★ Pゲイン（比例）: 「いまのズレ」に対するハンドルの強さ
                      #   初期値0.007は基準車の実走で安定した値
                      #   意味: 下段帯で測った横ズレ1pxあたり、何%ハンドルを切るか
                      #   役割: ラインの中心に車を引き戻す力。操舵の基本
                      #   大きくする → ズレをすぐ戻す。ただし効きすぎると戻りの
                      #                勢いが余って反対へ行き過ぎ、左右に振れ続ける
                      #                「蛇行」になる
                      #   小さくする → 直進は落ち着くが、ズレからの復帰が遅く、
                      #                大きく外れたとき戻りきれない
STEER_D_GAIN = 0.035  # ★ Dゲイン（微分）: 「ズレの勢い」に対するハンドルの強さ
                      #   意味: 横ズレが1フレームで1px変化するごとに、何%ハンドルを
                      #         足すか。0.0なら勢いを使わない（P制御のみ）
                      #   役割: ラインへ近づく・離れる「勢い」を読んで、行き過ぎる前に
                      #         カウンターを当てる。Pの戻しすぎ（蛇行）を抑えるブレーキ役
                      #   大きくする → 蛇行が収まり高速でも安定。ただし効きすぎると
                      #                カメラの振動などのノイズに反応してハンドルが
                      #                小刻みに震える
                      #   小さくする(0へ) → 動きは素直だが、速度を上げると
                      #                修正が後手に回り蛇行しやすい
                      #   目安: 蛇行が出たら 0.01 刻みで上げ下げして試す
                      #         （0.01〜0.05 の範囲で調整）
                      #   初期値0.035は基準車の実走で蛇行が収まった値
KIREKAKU_HABA = 1.00  # ハンドルの最大値[%]（機械限界の保護）。servo_testと同じ値
HANDORU_GYAKU = False # ハンドルの向き（個別設定項目）。ラインから「逃げる」方向に
                      # 切ってしまう車は True。servo_test の「左右の向きを反転」
                      # ボタンで kuruma_settei.json に保存された値が反映される
STEER_HOJI_HIROGARI = 0.35  # ラインの色が横方向に画面のこの割合以上広がったら、
                            # ハンドルを直前の値のまま保持する。
                            # ゴールライン進入時に計算が狂って誤操舵するのを防ぐガード

# ------------------------------------------------------------------------------
# スロットルの向きとブレーキ
# ------------------------------------------------------------------------------
SUROTTORU_GYAKU = False # スロットルの向き（個別設定項目）
                        #   True  = 低い%で前進（タミヤ TT-02 の実測）
                        #   False = 高い%で前進（ヨコモ RD2.0 の実測）
                        #   throttle_test と同じ値を kuruma_settei.json に書くこと
BRAKE = 50              # ★ ゴール後ブレーキの強さ（0〜100%）
                        #   （個別セッティング項目）
                        #   ルール「マーカー通過後5m以内に停止」を守るために調整する。
                        #   速いセッティング（ACCEL_SAIDAI高め）ほど強いブレーキが必要
                        #   大きくする → 強く止まるが、姿勢を乱すことがある
                        #   小さくする → 滑らかだが制動距離が伸びる（5mを超えたら失格級）
BRAKE_BYO = 1.0         # ブレーキを出す時間[秒]。長くしすぎるとESCの仕様で
                        # 後退に入ることがあるため、大きく変えないこと

# ------------------------------------------------------------------------------
# 安全装置
# ------------------------------------------------------------------------------
SAIDAI_SOKO_BYO = 30.0  # ★ 最大走行時間[秒]。超えたら強制停止する安全装置
                        #   （個別セッティング項目）
                        #   なぜ車ごとの設定なのか: 走行時間はアクセル設定で決まる
                        #   ため。アクセルを控えめにした車は完走に時間がかかるので、
                        #   その分この値も伸ばす必要がある（一律だと、遅いが正常に
                        #   走っている車を安全装置が止めてしまう）
                        #   決め方の目安: 自分の完走タイム + 余裕5秒
                        #   タイムアウトで止まった場合は「自分の完走時間を
                        #   見積もれていなかった」ということ。見積もりもセッティング
                        #   長すぎる値は暴走時の被害を広げるため、60秒を超える
                        #   設定は無効（プログラムが60秒に切り詰める）
SOKO_BYO_JOGEN = 60.0   # SAIDAI_SOKO_BYOの上限。変更禁止（安全装置の底）
                        # 最徐行でも30mを走り切れる長さとして設定してある
HEARTBEAT_YUKO = True   # 走行中、ブラウザとの通信が切れたら止める機能のON/OFF
HEARTBEAT_BYO = 2.0     # 通信断とみなすまでの秒数
                        #   短くする → 素早く止まるが、Wi-Fiの一瞬の乱れで誤停止
ANZEN_MIN, ANZEN_MAX = 7.50, 13.00  # 信号の絶対リミット。どんな計算結果でも
                                    # この外の信号は出さない。変更禁止

# ------------------------------------------------------------------------------
# ハードウェア（変更禁止）
# ------------------------------------------------------------------------------
STEER_GPIO, THROTTLE_GPIO = 13, 12  # 配線先のピン番号
PWM_SHUHASU = 70                    # 信号の周波数[Hz]。全ての%値はこの周波数が
                                    # 前提。変えると全開・中立の意味が変わり暴走する

# ------------------------------------------------------------------------------
# Wi-Fi配信・記録
# ------------------------------------------------------------------------------
HTTP_PORT = 8080        # ブラウザでアクセスする番号
HAISHIN_HABA = 480      # 配信映像の横サイズ[画素]。小さいほどWi-Fiが軽い
HAISHIN_FPS = 10        # ★ 配信のコマ数/秒。Wi-Fiが重い日は下げる
                        #   （検出は常に最高速で動く。これは「見る側」だけの設定）
HAISHIN_GASHITSU = 70   # 配信映像の画質(1〜100)。下げるとWi-Fiが軽くなる
LOG_FOLDER = "logs"     # 走行記録の保存先フォルダ。走行ごとに3点セットが残る:
                        #   run_日時.csv         … 走行データ(1フレーム1行)
                        #   run_日時.mp4         … 検出結果つき映像
                        #   run_日時_settei.json … そのときのセッティング一式
DOUGA_KIROKU = True     # ★ 走行映像の記録 ON/OFF。SDカードが苦しければ False
SETTEI_FILE = "kuruma_settei.json"

# ==============================================================================
KYOTSU_FILE = "kyotsu_settei.json"
_KAKIKAE_KINSHI = {"PWM_SHUHASU", "STEER_GPIO", "THROTTLE_GPIO",
                   "ANZEN_MIN", "ANZEN_MAX", "SOKO_BYO_JOGEN"}   # 安全のため上書き禁止

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
#   （camera_test.py と同じ仕組み。詳しい解説は設計書へ）
# ==============================================================================

def me_de_miru(gazou, ato_zure=None):
    """1枚の画像を調べて、見つけた結果を辞書で返す。
       ato_zure: 前フレームまで追跡していたラインの横位置[px]（None=追跡開始）"""
    takasa, haba = gazou.shape[:2]
    chuo = haba // 2

    shirokuro = cv2.cvtColor(gazou, cv2.COLOR_BGR2GRAY)
    if LINE_IRO == "kuro":
        _, sen_dake = cv2.threshold(shirokuro, KURO_SHIKII, 255, cv2.THRESH_BINARY_INV)
    else:
        _, sen_dake = cv2.threshold(shirokuro, SHIRO_SHIKII, 255, cv2.THRESH_BINARY)

    def obi_wo_shiraberu(ue, shita, gate_chuo, gate_haba):
        """帯の中の黒を塊ごとに分け、ゲート内で最も近い塊をラインとして返す。
           返り値: (ズレ, 広がり, ゲート外の塊の位置リスト)"""
        y0, y1 = int(takasa * ue), int(takasa * shita)
        obi = sen_dake[y0:y1, :]
        kosu, labels, stats, centers = cv2.connectedComponentsWithStats(obi)
        line_zure, line_hirogari = None, 0.0
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
        if chikai is not None:
            line_zure, line_hirogari = chikai
        return line_zure, line_hirogari, marks

    # 追跡の中心: 前回位置（無ければ画面中央から探し始める）
    if ato_zure is None:
        gate_chuo, gate_haba = 0.0, HAJIME_GATE
    else:
        gate_chuo, gate_haba = ato_zure, TSUIZUI_GATE

    zure_chikaku, hirogari_chikaku, marks = obi_wo_shiraberu(
        ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, gate_chuo, gate_haba)
    zure_tooku, _, _ = obi_wo_shiraberu(
        ROI_TOOKU_UE, ROI_TOOKU_SHITA, gate_chuo, gate_haba * 1.5)

    katamuki = None
    if zure_chikaku is not None and zure_tooku is not None:
        y_c = takasa * (ROI_CHIKAKU_UE + ROI_CHIKAKU_SHITA) / 2
        y_t = takasa * (ROI_TOOKU_UE + ROI_TOOKU_SHITA) / 2
        katamuki = math.degrees(math.atan2(zure_tooku - zure_chikaku, y_c - y_t))

    # 緑シグナル
    y0, y1 = int(takasa * SIG_UE), int(takasa * SIG_SHITA)
    x0, x1 = int(haba * SIG_HIDARI), int(haba * SIG_MIGI)
    hsv = cv2.cvtColor(gazou[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    midori = cv2.inRange(hsv, np.array(SIG_MIDORI_SHITA), np.array(SIG_MIDORI_UE))
    midori_wariai = cv2.countNonZero(midori) / midori.size

    return {
        "zure": None if zure_chikaku is None else round(float(zure_chikaku), 1),
        "zure_tooku": None if zure_tooku is None else round(float(zure_tooku), 1),
        "katamuki": None if katamuki is None else round(katamuki, 1),
        "line_mieru": zure_chikaku is not None,
        "hirogari": round(hirogari_chikaku, 2),
        "marks": marks,
        "midori": bool(midori_wariai >= SIG_WARIAI),
        "midori_wariai": round(midori_wariai, 3),
    }


# ==============================================================================
# ブロック3: 脳 —— 状態に応じて判断し、ハンドルとアクセルを決める
#
#   IDLE     : 待機。キャリブレーションはこの状態で（出力はすべて中立）
#   ARMED    : スタート待ち。緑シグナルが3回連続で見えたら発進
#   RUNNING  : 走行中。ズレを打ち消すハンドル + 発進プロファイルのアクセル
#   FINISHED : ゴール後。自動でブレーキ→N。結果を表示
#
#   IDLE --[ARM]--> ARMED --[緑]--> RUNNING --+--> FINISHED
#     ^                                        |
#     +---------[STOP(いつでも)]<--------------+
#
#   RUNNING が終わる条件: ゴール / ラインロスト / 時間切れ / 通信断
# ==============================================================================

def nou_de_handan(app, kekka, ima):
    """状態遷移と、走行中のハンドル・アクセル計算（app.lock 取得済みで呼ばれる）"""

    if app.state == "ARMED":
        app.midori_kaisu = app.midori_kaisu + 1 if kekka["midori"] else 0
        if app.midori_kaisu >= SIG_KAKUTEI:
            app.state = "RUNNING"
            app.start_jikoku = ima
            app.saigo_tsushin = ima      # 発進時点から通信監視をやり直す
            app.goal_kaisu = 0
            app.rosuto_kaisu = 0
            app.kiroku_kaishi()
            print("[脳] スタート!")

    elif app.state == "RUNNING":
        keika = ima - app.start_jikoku

        # --- アクセル: 発進プロファイル（発進%からランプで踏み増す） ---
        accel = min(ACCEL_SAIDAI, ACCEL_HASSHIN + ACCEL_RAMPU * keika)

        # --- ハンドル: P制御(いまのズレ) + D制御(ズレの勢い=この先の予告) ---
        # ラインの色が横に広い（＝横線進入中など）ときは、
        # 計算がくるうのでハンドルを直前の値のまま保持する
        if kekka["zure"] is not None and kekka["hirogari"] < STEER_HOJI_HIROGARI:
            muki = -1 if HANDORU_GYAKU else 1
            # D項: 横ズレの前フレーム差分[px/フレーム] = ズレの勢い。
            # 直前フレームのズレが無いとき（発進直後・ロスト明け・保持明け）は
            # 差分が作れないので、そのフレームは D=0 で走る
            if app.zure_zenkai is None:
                sabun = 0.0
            else:
                sabun = kekka["zure"] - app.zure_zenkai
            app.zure_zenkai = kekka["zure"]
            sa = -(STEER_P_GAIN * kekka["zure"] + STEER_D_GAIN * sabun) * muki
            app.steer_sa = max(-KIREKAKU_HABA, min(KIREKAKU_HABA, sa))
        else:
            # ライン無し・保持中は差分の記憶を捨てる
            # （欠測をまたいで差分を取ると巨大なD値が出てハンドルが跳ねるため）
            app.zure_zenkai = None
        # (ハンドル自体は app.steer_sa を保持 = 前回の値のまま)

        app.teashi.hashiru(app.steer_sa, accel, keika < KICK_BYO)
        app.accel = accel
        app.kiroku_tsuika(keika, kekka)

        # --- 終わりの判定 ---
        # ゴール = ラインの左右両側に同時にMARK（ゴールマーカー）が見えること。
        # 最初に見えたフレームの時刻を控え、連続で確定したらその時刻をタイムに
        # する（確定待ちの遅れを補正）
        marker_ryou = False
        if kekka["zure"] is not None:
            hidari = migi = False
            for m in kekka.get("marks", []):
                sa = m - kekka["zure"]              # ラインから見たMARKの位置
                if -GOAL_M_TOOI <= sa <= -GOAL_M_CHIKAI:
                    hidari = True
                elif GOAL_M_CHIKAI <= sa <= GOAL_M_TOOI:
                    migi = True
            marker_ryou = hidari and migi
        app.marker_mieru = marker_ryou
        if marker_ryou:
            if app.goal_kaisu == 0:
                app.goal_hajime = keika            # 最初に見えた時刻
            app.goal_kaisu += 1
        else:
            app.goal_kaisu = 0

        # ライン切断はバックアップ停止（マーカー見逃し・コースアウト時）
        if not kekka["line_mieru"]:
            if app.rosuto_kaisu == 0:
                app.rosuto_hajime = keika
            app.rosuto_kaisu += 1
        else:
            app.rosuto_kaisu = 0

        riyu = None
        if app.goal_kaisu >= GOAL_KAKUTEI:
            riyu = "goal"                          # ゴールマーカー検出
        elif app.rosuto_kaisu >= ROSUTO_KAKUTEI:
            riyu = "line_lost"                     # バックアップ停止
        elif keika >= min(SAIDAI_SOKO_BYO, SOKO_BYO_JOGEN):
            riyu = "timeout"
        elif (HEARTBEAT_YUKO and app.saigo_tsushin
              and ima - app.saigo_tsushin > HEARTBEAT_BYO):
            riyu = "tsushin_lost"    # ブラウザとの通信が途絶えた

        if riyu:
            app.state = "FINISHED"
            # ゴールはマーカー初認時刻をタイムにする（確定遅れを除去）
            app.goal_time = app.goal_hajime if riyu == "goal" else keika
            app.owari_riyu = riyu
            app.accel = 0
            app.kiroku_shuryo()
            app.teashi.tomaru()      # ブレーキ→N を自動実行
            print(f"[脳] 終了({riyu}) タイム {app.goal_time:.2f} 秒")
            app.keisoku_tsuchi()     # 計測サーバへ通過を通知（設定時のみ）


# ==============================================================================
# ブロック4: 手足 —— ハンドルとアクセルへの信号出力
#   servo_test / throttle_test で調整・学習した kuruma_settei.json を使う
# ==============================================================================

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

class TeAshi:
    def __init__(self):
        self.pi = None
        self.chosei_zumi = False

        # --- 学習済みの個体値を読み込む ---
        self.steer_churitsu = None
        self.d_ten = None
        self.r_ten = None
        self.n = None
        try:
            with open(SETTEI_FILE) as f:
                settei = json.load(f)
            self.steer_churitsu = settei.get("STEER_CHURITSU")
            self.d_ten = settei.get("ZENSHIN_KYOKAI")
            self.r_ten = settei.get("KOTAI_KYOKAI")
            self.n = settei.get("THROTTLE_N")
            self.chosei_zumi = all(v is not None for v in
                                   (self.steer_churitsu, self.d_ten,
                                    self.r_ten, self.n))
        except Exception:
            pass

        if self.chosei_zumi:
            print(f"[手足] 個体値: ステア中立={self.steer_churitsu:.2f}% "
                  f"D点={self.d_ten:.2f}% R点={self.r_ten:.2f}% N={self.n:.2f}%")
            if ZENSHIN_SAITEI is None:
                print("[手足] L点が未学習です。アクセル0% = D点で走ります（低速では走れません）。")
        else:
            print(f"[手足] 警告: {SETTEI_FILE} が無いか不完全です。")
            print("[手足] servo_test.py と throttle_test.py の調整・学習を先に行ってください。")
            print("[手足] 安全のため、走行開始(ARM)はできません。")

        # --- アクセル0%の点: L点（ZENSHIN_SAITEI）。未学習ならD点 ---
        self.zero_ten = self.d_ten
        if self.chosei_zumi and ZENSHIN_SAITEI is not None:
            muki = -1 if SUROTTORU_GYAKU else 1
            ok = (isinstance(ZENSHIN_SAITEI, (int, float))
                  and not isinstance(ZENSHIN_SAITEI, bool)
                  and 0 < (ZENSHIN_SAITEI - self.n) * muki
                        <= (self.d_ten - self.n) * muki)
            if ok:
                self.zero_ten = float(ZENSHIN_SAITEI)
                print(f"[手足] 低速走行: アクセル0% = L点 {self.zero_ten:.2f}% "
                      f"(D点 {self.d_ten:.2f}%) / 発進キック {KICK_BYO}秒")
            else:
                print(f"[手足] 警告: L点(ZENSHIN_SAITEI)={ZENSHIN_SAITEI} は "
                      f"N({self.n:.2f}%)とD点({self.d_ten:.2f}%)の間にありません。")
                print("[手足] 無視してD点を0%にします。throttle_test で学習し直してください。")

        # --- pigpio ---
        try:
            import pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                self.pi = None
                raise RuntimeError("pigpiod が動いていません")
            print("[手足] pigpio に接続しました（実機モード）")
        except Exception as e:
            print(f"[手足] pigpio が使えません ({e})")
            try:
                self.pi = Pi5PWM()
                print("[手足] Pi 5 のハードウェアPWMで出力します（実機モード）")
            except Exception as e2:
                self.pi = None
                print(f"[手足] Pi 5 用の出力も使えません ({e2}) — 画面のみのテストモード")

        self._brake_chu = False
        self.neutral()

    # --- 低レベル出力（安全リミット付き） ---
    def _dasu(self, gpio, duty):
        duty = max(ANZEN_MIN, min(ANZEN_MAX, duty))
        if self.pi is not None:
            self.pi.hardware_PWM(gpio, PWM_SHUHASU, int(duty * 10000))
        return duty

    # --- 走行出力: ハンドル差分[%]とアクセル[0-100%]を受け取る ---
    def hashiru(self, steer_sa, accel, kick=False):
        if not self.chosei_zumi or self._brake_chu:
            return
        self.ima_steer = self._dasu(STEER_GPIO, self.steer_churitsu + steer_sa)
        muki = -1 if SUROTTORU_GYAKU else 1
        duty = self.zero_ten + muki * ZENKAI_HABA * (accel / 100.0)
        if kick:        # 発進キック中: D点より弱い信号にはしない
            duty = max(duty, self.d_ten) if muki > 0 else min(duty, self.d_ten)
        self.ima_throttle = self._dasu(THROTTLE_GPIO, duty)

    # --- 停止: ブレーキ→N（別スレッドで自動実行） ---
    def tomaru(self):
        if self._brake_chu:
            return
        self._brake_chu = True
        threading.Thread(target=self._brake_kara_n, daemon=True).start()

    def _brake_kara_n(self):
        try:
            if self.chosei_zumi:
                # ハンドルは中立へ、アクセルは後退側（＝ブレーキ）へ
                self.ima_steer = self._dasu(STEER_GPIO, self.steer_churitsu)
                muki = -1 if SUROTTORU_GYAKU else 1
                brake = self.r_ten - muki * ZENKAI_HABA * (BRAKE / 100.0)
                self.ima_throttle = self._dasu(THROTTLE_GPIO, brake)
                time.sleep(BRAKE_BYO)
        finally:
            self._brake_chu = False
            self.neutral()
            print("[手足] 停止完了（N）")

    # --- 中立: ハンドルまっすぐ・アクセルN ---
    def neutral(self):
        steer = self.steer_churitsu if self.steer_churitsu else 10.00
        n = self.n if self.n else 10.48
        self.ima_steer = self._dasu(STEER_GPIO, steer)
        self.ima_throttle = self._dasu(THROTTLE_GPIO, n)

    def close(self):
        self.neutral()
        time.sleep(0.5)
        if self.pi is not None:
            self.pi.hardware_PWM(STEER_GPIO, 0, 0)
            self.pi.hardware_PWM(THROTTLE_GPIO, 0, 0)
            self.pi.stop()


# ==============================================================================
# ==============================================================================
#   ここから下は「仕組み」の部分です（カメラ・画面・配信）。
#   車の走り方には関係ないので、読まなくてOKです。
# ==============================================================================
# ==============================================================================

# ==============================================================================
# ブロック5: カメラ
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
            time.sleep(0.5)
            print("[カメラ] 起動しました")
        except Exception as e:
            print(f"[カメラ] 起動できません ({e}) — テスト映像で動きます")

    def toru(self):
        if self.picam2 is not None:
            return self.picam2.capture_array()
        img = np.full((CAM_HEIGHT, CAM_WIDTH, 3), 170, np.uint8)
        cv2.line(img, (CAM_WIDTH // 2 + 15, CAM_HEIGHT),
                 (CAM_WIDTH // 2 - 10, int(CAM_HEIGHT * 0.4)), (30, 30, 30), 14)
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

    y_chikaku = obi(ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, (255, 128, 0))
    y_tooku = obi(ROI_TOOKU_UE, ROI_TOOKU_SHITA, (255, 220, 0))

    # 追従ゲート（ラインの続きを探す枠）を黄色で描く
    def gate_waku(ue, shita, g_haba):
        g_chuo = chuo + int(ato if ato is not None else 0)
        y0, y1 = int(takasa * ue), int(takasa * shita)
        x0 = max(0, g_chuo - g_haba)
        x1 = min(haba - 1, g_chuo + g_haba)
        cv2.rectangle(img, (x0, y0 + 2), (x1, y1 - 2), (0, 255, 255), 1)
        cv2.line(img, (g_chuo, y0 + 2), (g_chuo, y0 + 12), (0, 255, 255), 2)

    gate_chikaku = TSUIZUI_GATE if ato is not None else HAJIME_GATE
    gate_waku(ROI_CHIKAKU_UE, ROI_CHIKAKU_SHITA, gate_chikaku)
    gate_waku(ROI_TOOKU_UE, ROI_TOOKU_SHITA, int(gate_chikaku * 1.5))

    y0, y1 = int(takasa * SIG_UE), int(takasa * SIG_SHITA)
    x0, x1 = int(haba * SIG_HIDARI), int(haba * SIG_MIGI)
    cv2.rectangle(img, (x0, y0), (x1, y1),
                  (0, 255, 0) if kekka["midori"] else (0, 200, 255), 2)

    jx, jy = int(haba * JUJI_X), int(takasa * JUJI_Y)
    cv2.line(img, (jx - 20, jy), (jx + 20, jy), (0, 255, 0), 2)
    cv2.line(img, (jx, jy - 20), (jx, jy + 20), (0, 255, 0), 2)

    for m in kekka.get("marks", []):
        cv2.circle(img, (int(chuo + m), y_chikaku), 8, (255, 0, 255), 2)
        cv2.putText(img, "MARK", (int(chuo + m) - 22, y_chikaku - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)
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
    def __init__(self):
        self.kamera = Kamera()
        self.teashi = TeAshi()
        self.lock = threading.Lock()
        self.ugoiteru = True

        self.state = "IDLE"
        self.kekka = {}
        self.fps = 0.0
        self.start_jikoku = None
        self.goal_time = None
        self.owari_riyu = ""
        self.haishin_jpeg = None
        self.accel = 0
        self.steer_sa = 0.0
        self.saigo_tsushin = None    # 最後にブラウザから通信があった時刻

        self.midori_kaisu = 0
        self.goal_kaisu = 0
        self.rosuto_kaisu = 0
        self.rosuto_hajime = 0.0
        self.goal_hajime = 0.0
        self.marker_mieru = False
        self.line_ato = None      # 追跡中のライン位置[px]（None=次フレームで再取得）
        self.zure_zenkai = None   # 前フレームの横ズレ[px]（D項の差分用。None=差分なし）
        self._csv = None
        self._csv_file = None
        self._kiroku_base = None      # 今回の走行の記録ファイル名（拡張子なし）
        self._douga = None            # 動画の書き込み係（映像ループが管理する）

    # ---- ボタン ----
    def botan_arm(self):
        with self.lock:
            if self.state != "IDLE":
                return
            if not self.teashi.chosei_zumi and self.teashi.pi is not None:
                print("[ボタン] ARM拒否: 個体値が未調整です（kuruma_settei.json）")
                return
            self.state = "ARMED"
            self.midori_kaisu = 0
            self.steer_sa = 0.0
            self.line_ato = None      # 追跡をリセット（中央から探し直す）
            self.zure_zenkai = None   # D項の差分もリセット
            print("[ボタン] ARM: 緑シグナル待ち")

    def botan_stop(self):
        with self.lock:
            mae = self.state
            self.state = "IDLE"
            self.accel = 0
            self.kiroku_shuryo()
            if mae == "RUNNING":
                self.teashi.tomaru()        # 走行中ならブレーキ→N
            else:
                self.teashi.neutral()
            if mae != "IDLE":
                print(f"[ボタン] STOP: {mae} → IDLE")

    # ---- 計測サーバへの通過通知（別スレッドで送る。失敗しても走行に影響なし） ----
    def keisoku_tsuchi(self):
        if not KEISOKU_URL:
            return
        threading.Thread(target=self._tsuchi_okuru, daemon=True).start()

    def _tsuchi_okuru(self):
        try:
            import socket
            import urllib.request
            namae = KURUMA_NAMAE or socket.gethostname()
            data = json.dumps({
                "kuruma": namae,
                "time": None if self.goal_time is None else round(self.goal_time, 3),
                "riyu": self.owari_riyu,
            }).encode("utf-8")
            req = urllib.request.Request(
                KEISOKU_URL, data=data,
                headers={"Content-Type": "application/json"})
            urllib.request.urlopen(req, timeout=2.0)
            print(f"[計測] 通過を通知しました: {namae} {self.goal_time}秒")
        except Exception as e:
            print(f"[計測] 通知できませんでした ({e}) — 走行には影響ありません")

    # ---- 走行データの記録 ----
    def kiroku_kaishi(self):
        os.makedirs(LOG_FOLDER, exist_ok=True)
        self._kiroku_base = os.path.join(
            LOG_FOLDER, datetime.datetime.now().strftime("run_%Y%m%d_%H%M%S"))
        # --- 走行データ(CSV) ---
        self._csv_file = open(self._kiroku_base + ".csv", "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["時間[秒]", "横ズレ[px]", "遠くのズレ[px]", "傾き[度]",
                            "ライン", "マーク数", "ハンドル[%]", "アクセル[%]"])
        # --- 設定スナップショット(この走りがどんなセッティングだったかの記録) ---
        snap = {
            "nichiji": datetime.datetime.now().isoformat(timespec="seconds"),
            "ACCEL_SAIDAI": ACCEL_SAIDAI, "ACCEL_HASSHIN": ACCEL_HASSHIN,
            "ACCEL_RAMPU": ACCEL_RAMPU,
            "STEER_P_GAIN": STEER_P_GAIN, "STEER_D_GAIN": STEER_D_GAIN,
            "BRAKE": BRAKE, "SAIDAI_SOKO_BYO": SAIDAI_SOKO_BYO,
            "HANDORU_GYAKU": HANDORU_GYAKU,
            "LINE_IRO": LINE_IRO,
            "KURO_SHIKII": KURO_SHIKII, "SHIRO_SHIKII": SHIRO_SHIKII,
            "TSUIZUI_GATE": TSUIZUI_GATE,
            "STEER_CHURITSU": self.teashi.steer_churitsu,
            "ZENSHIN_KYOKAI": self.teashi.d_ten,
            "KOTAI_KYOKAI": self.teashi.r_ten,
            "THROTTLE_N": self.teashi.n,
            "ZENSHIN_SAITEI": ZENSHIN_SAITEI, "KICK_BYO": KICK_BYO,
        }
        with open(self._kiroku_base + "_settei.json", "w") as f:
            json.dump(snap, f, indent=2, ensure_ascii=False)
        print(f"[記録] {self._kiroku_base}.csv / .mp4 / _settei.json に保存します")

    def kiroku_tsuika(self, keika, kekka):
        if self._csv:
            self._csv.writerow([f"{keika:.3f}", kekka["zure"], kekka["zure_tooku"],
                                kekka["katamuki"], int(kekka["line_mieru"]),
                                len(kekka.get("marks", [])),
                                f"{self.steer_sa:+.2f}", f"{self.accel:.0f}"])

    def kiroku_shuryo(self):
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv = None

    # ---- メインループ: 撮る → 見る → 判断する ----
    def mawasu(self):
        mae_jikoku = time.time()
        fps_goukei, fps_kaisu = 0.0, 0
        while self.ugoiteru:
            gazou = self.kamera.toru()
            with self.lock:
                ato = self.line_ato
            kekka = me_de_miru(gazou, ato)

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
                nou_de_handan(self, kekka, ima)
                self.kekka = kekka
                state = self.state

            e = gamen_ni_kaku(gazou, kekka, state, ato)
            h2 = int(e.shape[0] * HAISHIN_HABA / e.shape[1])
            chiisai = cv2.resize(e, (HAISHIN_HABA, h2))

            # --- 走行映像の記録（RUNNINGの間だけ。開け閉めもこのループが行う） ---
            if DOUGA_KIROKU:
                with self.lock:
                    base = self._kiroku_base if state == "RUNNING" else None
                if base and self._douga is None:
                    self._douga = cv2.VideoWriter(
                        base + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"),
                        30, (chiisai.shape[1], chiisai.shape[0]))
                    if not self._douga.isOpened():          # mp4が無理な環境用の控え
                        self._douga = cv2.VideoWriter(
                            base + ".avi", cv2.VideoWriter_fourcc(*"MJPG"),
                            30, (chiisai.shape[1], chiisai.shape[0]))
                if self._douga is not None:
                    if state == "RUNNING":
                        self._douga.write(chiisai)
                    else:                                   # 走行が終わったら閉じる
                        self._douga.release()
                        self._douga = None
                        print("[記録] 走行映像を保存しました")

            ok, jpeg = cv2.imencode(".jpg", chiisai,
                                    [cv2.IMWRITE_JPEG_QUALITY, HAISHIN_GASHITSU])
            if ok:
                with self.lock:
                    self.haishin_jpeg = jpeg.tobytes()

        if self._douga is not None:
            self._douga.release()
        self.kamera.close()
        self.teashi.close()

    # ---- ブラウザに送る状態 ----
    def joutai_json(self):
        with self.lock:
            self.saigo_tsushin = time.time()     # ハートビート更新
            keika = None
            if self.state == "RUNNING" and self.start_jikoku:
                keika = round(time.time() - self.start_jikoku, 2)
            return json.dumps({
                "state": self.state,
                "zure": self.kekka.get("zure"),
                "katamuki": self.kekka.get("katamuki"),
                "line_mieru": bool(self.kekka.get("line_mieru")),
                "midori": bool(self.kekka.get("midori")),
                "marks": len(self.kekka.get("marks", [])),
                "goal_mieru": bool(self.marker_mieru),
                "accel": round(self.accel),
                "steer": round(self.steer_sa, 2),
                "fps": round(self.fps, 1),
                "keika": keika,
                "goal_time": None if self.goal_time is None else round(self.goal_time, 2),
                "owari_riyu": self.owari_riyu,
                "chosei_zumi": self.teashi.chosei_zumi,
            })


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
  #left { min-width:0; display:flex; align-items:center; justify-content:center; }
  #video { max-width:100%; max-height:calc(100vh - 24px);
    border-radius:10px; border:1px solid var(--line); display:block; }
  #right { display:flex; flex-direction:column; gap:10px; min-height:0; }
  #statebar { padding:14px; border-radius:10px; text-align:center;
    font-size:28px; font-weight:700; letter-spacing:2px;
    background:var(--idle); color:#fff; transition:background .2s; }
  #statebar.ARMED { background:var(--armed); }
  #statebar.RUNNING { background:var(--run); }
  #statebar.FINISHED { background:var(--fin); }
  #metrics { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
  .metric { background:var(--panel); border:1px solid var(--line);
    border-radius:10px; padding:9px 8px; text-align:center; }
  .metric .v { font-size:24px; font-weight:700; font-variant-numeric:tabular-nums; }
  .metric .k { font-size:11px; color:var(--dim); margin-top:2px; }
  .ok { color:var(--run); } .ng { color:#d95b5b; }
  #buttons { display:flex; flex-direction:column; gap:10px; margin-top:auto; }
  button { padding:20px; font-size:23px; font-weight:700;
    border:none; border-radius:10px; color:#fff; cursor:pointer; }
  #btn-arm { background:var(--armed); }
  #btn-stop { background:#c94f4f; }
  button:active { filter:brightness(.85); }
  button:disabled { opacity:.4; cursor:default; }
  #result { text-align:center; font-size:16px; color:var(--dim); min-height:22px; }
  #result.warn { color:#e0b45c; font-weight:700; }
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
      <div class="metric"><div class="v" id="m-accel">--</div><div class="k">アクセル [%]</div></div>
      <div class="metric"><div class="v" id="m-steer">--</div><div class="k">ハンドル [%]</div></div>
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
  const riyu = { goal:"ゴール!", line_lost:"ラインロスト", timeout:"タイムアウト",
                 tsushin_lost:"通信断のため停止" };
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
        : (s.marks > 0 ? "MARK:" + s.marks : "--");
      $("m-goal").className = "v " + (s.goal_mieru ? "ok" : "");
      $("m-fps").textContent = s.fps ?? "--";
      $("m-accel").textContent = s.accel ?? "--";
      $("m-steer").textContent = (s.steer >= 0 ? "+" : "") + s.steer.toFixed(2);
      $("btn-arm").disabled = !s.chosei_zumi && s.state === "IDLE" ? true : (s.state !== "IDLE");
      const r = $("result");
      if (!s.chosei_zumi) {
        r.textContent = "未調整: servo_test / throttle_test の学習を先に行ってください";
        r.className = "warn";
      } else if (s.state === "FINISHED") {
        r.textContent = `${riyu[s.owari_riyu] ?? s.owari_riyu}  タイム ${s.goal_time ?? "--"} 秒`;
        r.className = "";
      } else {
        r.textContent = "";
        r.className = "";
      }
    } catch (e) { /* 通信断は次のポーリングで復帰 */ }
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
            elif self.path == "/stream":
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
                    pass
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
# ブロック7: main
# ==============================================================================

def main():
    app = RaceApp()
    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler_wo_tsukuru(app))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[web] ブラウザで http://<この車のIPアドレス>:{HTTP_PORT} を開いてください")

    def tomeru(*_):
        print("\n[main] 停止処理中...")
        app.ugoiteru = False
        app.botan_stop()

    signal.signal(signal.SIGINT, tomeru)
    signal.signal(signal.SIGTERM, tomeru)

    app.mawasu()
    server.shutdown()
    print("[main] 終了しました")


if __name__ == "__main__":
    main()
