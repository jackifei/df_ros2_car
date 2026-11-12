import numpy as np
import matplotlib.pyplot as plt

L = 1.0

# 图1：固定 ω=0.3，δ 随 v 变化（展示低速爆炸）
v = np.linspace(0.02, 1.2, 500)
omega = 0.3
delta_deg = np.degrees(np.arctan(L * omega / v))

plt.figure(figsize=(7, 4.5))
plt.plot(v, delta_deg, lw=2)
plt.axhline(35, color='r', ls='--', label='δ_max = 35°')
plt.axhline(45, color='orange', ls='--', label='δ_max = 45°')
plt.xlabel('linear.x  v (m/s)')
plt.ylabel('前轮等效转角 δ (deg)')
plt.title('低速时 δ 急剧增大（L=1m, ω=0.3 rad/s）')
plt.grid(True); plt.legend()
plt.show()

# 图2：(v, ω) 平面可达域楔形
v2 = np.linspace(0, 1.5, 500)
plt.figure(figsize=(7, 4.5))
for dmax in [30, 35, 45, 60]:
    Rmin = L / np.tan(np.radians(dmax))
    wmax = v2 / Rmin
    line, = plt.plot(v2, wmax, label=f'δ_max={dmax}°, Rmin={Rmin:.2f}m')
    plt.plot(v2, -wmax, ls='--', color=line.get_color())

plt.axhline(0.3, color='k', ls=':', label='ω_cmd = 0.3')
plt.xlabel('v (m/s)')
plt.ylabel('ω (rad/s)')
plt.title('(v, ω) 合法可达域：楔形内部')
plt.grid(True); plt.legend()
plt.show()

# 图3：软缩放效果，ω_eff vs ω_cmd，在不同 v 下
w_cmd = np.linspace(-1, 1, 500)
plt.figure(figsize=(7, 4.5))
for vv in [0.1, 0.3, 0.6]:
    Rmin = L / np.tan(np.radians(35))
    w_limit = vv / Rmin
    w_eff = np.clip(w_cmd, -w_limit, w_limit)
    plt.plot(w_cmd, w_eff, label=f'v={vv} m/s, ω_max={w_limit:.2f}')
plt.plot(w_cmd, w_cmd, 'k:', lw=1, label='理想未限幅')
plt.xlabel('指令角速度 ω_cmd (rad/s)')
plt.ylabel('实际角速度 ω_eff (rad/s)')
plt.title('按 v 动态缩放角速度（δ_max=35°, L=1m）')
plt.grid(True); plt.legend()
plt.show()