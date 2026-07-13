# 变更记录 (Changelog)

## RVIZ2显示参数配置解释
🔹 Reliability（可靠性）
Best Effort：类似 UDP，发了就发了，丢就丢了，不重传。延迟低。
Reliable：类似 TCP，保证送达，丢了会重传。延迟高一点，但稳。

🔹 Durability（持久性）
Volatile：只发"此刻之后"的新消息，后加入的订阅者收不到历史数据。
Transient Local：发布者会把最后几条存下来，后加入的订阅者一上来就能拿到（ROS1 里的 latch就是这个）。

🔹 History（历史缓存）
Keep Last (depth=N)：只留最近 N 条，旧的覆盖掉。99% 场景用这个。
Keep All：全留，处理不过来内存会爆，慎用。