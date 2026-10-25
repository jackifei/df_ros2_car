#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
前轮转向查表工具（独立调试用）

正解:  steering_deg = interp(motor_pulses, TBL_PULSES, TBL_DEG)
逆解:  motor_pulses = interp(steering_deg, TBL_DEG,    TBL_PULSES)

约定: 脉冲 0 = 左转向极限，脉冲增大朝右。

== 查询入口 ==
直接修改下方 MARK_PULSES 列表（要查几个脉冲就填几个），
然后运行本脚本即可：程序会打印查表结果并在图上用红色星号标出。

命令行 --mark-pulse 仍可用作临时覆盖，例如：
    python3 steer_table_tool.py --mark-pulse 800,1600,2200
"""
import argparse
import sys

import numpy as np
import matplotlib.pyplot as plt


# ============================================================
# ★ 在这里输入要查询的脉冲（可单个或多个）
# ============================================================
MARK_PULSES = [1800.0]
# 例:
# MARK_PULSES = [800.0, 1600.0, 2200.0]
# MARK_PULSES = []          # 空列表 = 不标记任何点


# ---------------- 标定表 ----------------
TBL_PULSES = [0.0, 400.0, 800.0, 1200.0, 1600.0, 2000.0, 2400.0]
TBL_DEG    = [-30.0, -26.0, -12.0,  0.0,  12.0,   26.0,   30.0]


# ==================== 插值 / 正解 / 逆解 ====================
def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            t = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]


def forward(motor_pulses):
    """正解: 绝对脉冲 -> 车轮转向角 (°)。"""
    return interp(motor_pulses, TBL_PULSES, TBL_DEG)


def reverse(steering_deg):
    """逆解: 车轮转向角 (°) -> 绝对脉冲。"""
    return interp(steering_deg, TBL_DEG, TBL_PULSES)


def _linear_slope():
    return (TBL_DEG[-1] - TBL_DEG[0]) / (TBL_PULSES[-1] - TBL_PULSES[0])


def linear_forward(motor_pulses):
    return TBL_DEG[0] + (motor_pulses - TBL_PULSES[0]) * _linear_slope()


def linear_reverse(steering_deg):
    return TBL_PULSES[0] + (steering_deg - TBL_DEG[0]) / _linear_slope()


# ==================== 自检 ====================
def check_monotonic():
    ok = True
    if len(TBL_PULSES) != len(TBL_DEG):
        print('[!] 两表长度不一致'); ok = False
    if any(TBL_PULSES[i] >= TBL_PULSES[i + 1]
           for i in range(len(TBL_PULSES) - 1)):
        print('[!] TBL_PULSES 不是严格递增'); ok = False
    if any(TBL_DEG[i] >= TBL_DEG[i + 1]
           for i in range(len(TBL_DEG) - 1)):
        print('[!] TBL_DEG 不是严格递增'); ok = False
    return ok


def roundtrip_check(n=201, tol=1e-6):
    print('\n== 往返一致性检查 deg -> pulse -> deg ==')
    worst = max(abs(forward(reverse(d)) - d)
                for d in np.linspace(TBL_DEG[0], TBL_DEG[-1], n))
    print(f'   最大误差 {worst:.6f}°  '
          f'{"OK" if worst < tol else "FAIL"}')

    print('== 往返一致性检查 pulse -> deg -> pulse ==')
    worst = max(abs(reverse(forward(p)) - p)
                for p in np.linspace(TBL_PULSES[0], TBL_PULSES[-1], n))
    print(f'   最大误差 {worst:.6f} pulse  '
          f'{"OK" if worst < tol else "FAIL"}')


# ==================== 打印 ====================
def print_table():
    print('=== 标定点 (绝对脉冲 <-> 车轮角) ===')
    print(f'{"绝对脉冲":>12} {"车轮角(°)":>12} {"反解验证":>12}')
    for p, d in zip(TBL_PULSES, TBL_DEG):
        print(f'{p:>12.1f} {d:>12.2f} {forward(p):>12.4f}')


def print_mark(mark_pulses):
    print('\n=== 查询点查表结果 (MARK_PULSES) ===')
    print(f'{"输入脉冲":>12} {"查表角度(°)":>14} {"线性参考(°)":>14}'
          f' {"Δ=查表−线性(°)":>16}')
    for p in mark_pulses:
        d = forward(p)
        d_lin = linear_forward(p)
        print(f'{p:>12.1f} {d:>+14.3f} {d_lin:>+14.3f} '
              f'{d - d_lin:>+16.3f}')


# ==================== 画图 ====================
def plot(mark_pulses):
    plt.rcParams['font.sans-serif'] = [
        'SimHei', 'PingFang SC', 'Microsoft YaHei',
        'Noto Sans CJK SC', 'WenQuanYi Micro Hei', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    degs = np.linspace(TBL_DEG[0] - 5, TBL_DEG[-1] + 5, 401)
    p_tbl = np.array([reverse(d) for d in degs])
    p_lin = np.array([linear_reverse(d) for d in degs])

    pulses = np.linspace(TBL_PULSES[0] - 200.0,
                         TBL_PULSES[-1] + 200.0, 401)
    s_tbl = np.array([forward(p) for p in pulses])
    s_lin = np.array([linear_forward(p) for p in pulses])

    fig, axes = plt.subplots(1, 2, figsize=(15, 6))
    mark_degs = [forward(p) for p in mark_pulses]

    # ---------- 左: 逆解 ----------
    ax = axes[0]
    xs = list(TBL_DEG) + mark_degs
    ys = list(TBL_PULSES) + list(mark_pulses)
    x_min, x_max = min(xs) - 5, max(xs) + 5
    y_min, y_max = min(ys) - 200, max(ys) + 200
    ax.set_xlim(x_min, x_max); ax.set_ylim(y_min, y_max)

    ax.plot(degs, p_tbl, 'b-', lw=2, label='查表（绝对脉冲）')
    ax.plot(degs, p_lin, 'r--', lw=1.2, label='首尾线性参考')
    ax.scatter(TBL_DEG, TBL_PULSES, c='k', s=40,
               zorder=5, label='标定点')
    ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)

    x_mid = (x_min + x_max) / 2
    for p in mark_pulses:
        d = forward(p)
        ax.plot([d, d], [y_min, p], 'k:', lw=0.8, alpha=0.55)
        ax.plot([x_min, d], [p, p], 'k:', lw=0.8, alpha=0.55)
        ax.scatter([d], [p], c='red', s=200, marker='*',
                   zorder=9, edgecolors='black', linewidths=0.8,
                   label=f'标记: {p:.0f} pulse')
        dy = -55 if d > x_mid else 20
        ax.annotate(f'{d:+.2f}°\n← p={p:.0f}',
                    xy=(d, p), xytext=(20, dy),
                    textcoords='offset points', fontsize=9,
                    bbox=dict(boxstyle='round,pad=0.35',
                              fc='#ffffaa', ec='red', alpha=0.9),
                    arrowprops=dict(arrowstyle='->', color='red', lw=1.2))

    ax.set_xlabel('车轮转向角 (deg)')
    ax.set_ylabel('电机绝对脉冲 (pulse)')
    ax.set_title('逆解: 车轮角 → 绝对脉冲')
    ax.grid(True, alpha=0.3); ax.legend(loc='best', fontsize=9)

    # ---------- 右: 正解 ----------
    ax = axes[1]
    xs = list(TBL_PULSES) + list(mark_pulses)
    ys = list(TBL_DEG) + mark_degs
    x_min, x_max = min(xs) - 200, max(xs) + 200
    y_min, y_max = min(ys) - 3, max(ys) + 3
    ax.set_xlim(x_min, x_max); ax.set_ylim(y_min, y_max)

    ax.plot(pulses, s_tbl, 'b-', lw=2, label='查表')
    ax.plot(pulses, s_lin, 'r--', lw=1.2, label='首尾线性参考')
    ax.scatter(TBL_PULSES, TBL_DEG, c='k', s=40,
               zorder=5, label='标定点')
    ax.axhline(0, color='gray', lw=0.5); ax.axvline(0, color='gray', lw=0.5)

    y_mid = (y_min + y_max) / 2
    for p in mark_pulses:
        d = forward(p)
        d_lin = linear_forward(p)
        delta = d - d_lin

        ax.plot([p, p], [y_min, d], 'k:', lw=0.8, alpha=0.55)
        ax.plot([x_min, p], [d, d], 'k:', lw=0.8, alpha=0.55)
        ax.scatter([p], [d_lin], s=90, marker='o',
                   facecolors='none', edgecolors='gray',
                   linewidths=1.2, zorder=8)
        ax.scatter([p], [d], c='red', s=200, marker='*',
                   zorder=9, edgecolors='black', linewidths=0.8,
                   label=f'标记: {p:.0f} pulse')

        label = f'p={p:.0f}\n→ {d:+.2f}°'
        if abs(delta) > 1e-3:
            label += f'\n线性 {d_lin:+.2f}°\nΔ={delta:+.2f}°'
        dy = -75 if d > y_mid else 20
        ax.annotate(label,
                    xy=(p, d), xytext=(20, dy),
                    textcoords='offset points', fontsize=9,
                    bbox=dict(boxstyle='round,pad=0.35',
                              fc='#ffffaa', ec='red', alpha=0.9),
                    arrowprops=dict(arrowstyle='->', color='red', lw=1.2))

    ax.set_xlabel('电机绝对脉冲 (pulse)')
    ax.set_ylabel('车轮转向角 (deg)')
    ax.set_title('正解: 绝对脉冲 → 车轮角')
    ax.grid(True, alpha=0.3); ax.legend(loc='best', fontsize=9)

    fig.tight_layout()
    plt.show()


# ==================== CLI ====================
def _parse_mark(text):
    text = text.replace('，', ',').replace(' ', ',')
    return [float(s) for s in text.split(',') if s.strip()]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--test', action='store_true',
                    help='只做往返一致性自检')
    ap.add_argument('--mark-pulse', type=str, default=None,
                    help='临时覆盖 MARK_PULSES，如 800,1600,2200')
    ap.add_argument('--query', type=float,
                    help='反查: 车轮角(deg) -> 脉冲（纯打印，不画图）')
    ap.add_argument('--query-pulse', type=float,
                    help='正查: 脉冲 -> 车轮角(deg)（纯打印，不画图）')
    ap.add_argument('--no-plot', action='store_true', help='不弹窗画图')
    args = ap.parse_args()

    if not check_monotonic():
        print('标定表不满足单调性，插值结果不可信，请先修正。')
        return 1

    # ---- 纯打印模式 ----
    if args.query is not None:
        d = args.query
        if d < TBL_DEG[0] or d > TBL_DEG[-1]:
            print(f'[!] {d}° 超出表格范围，将被钳位')
        p = reverse(d)
        print(f'车轮角 {d:+.3f}° -> 电机绝对脉冲 {p:.1f}')
        print(f'反解验证: {forward(p):+.4f}°')
        return 0

    if args.query_pulse is not None:
        p = args.query_pulse
        if p < TBL_PULSES[0] or p > TBL_PULSES[-1]:
            print(f'[!] 脉冲 {p} 超出表格范围，将被钳位')
        d = forward(p)
        print(f'电机绝对脉冲 {p:.1f} -> 车轮角 {d:+.4f}°')
        print(f'反解验证: {reverse(d):.1f} pulse')
        return 0

    # ---- 决定本轮要标记的脉冲 ----
    if args.mark_pulse is not None:
        try:
            mark_pulses = _parse_mark(args.mark_pulse)
        except ValueError:
            print('[!] --mark-pulse 解析失败')
            return 1
    else:
        mark_pulses = list(MARK_PULSES)   # ← 默认用代码里的变量

    # ---- 常规输出 ----
    print_table()
    roundtrip_check()

    if mark_pulses:
        print_mark(mark_pulses)
    else:
        print('\n(未设置 MARK_PULSES，跳过查询点标记)')

    if args.test:
        return 0

    if not args.no_plot:
        plot(mark_pulses)
    return 0


if __name__ == '__main__':
    sys.exit(main())