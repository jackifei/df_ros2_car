# 手柄桥接后的状态消息
std_msgs/Header header

# ---- 前 10 个通道映射后的微秒值（默认 1000~2000）----
# 说明：ch06/ch07 是开关通道，这里的 ch06/ch07 是它们的微秒值，
#       真正的开关状态请看下面的 switch06 / switch07。
float32  ch01
float32  ch02
float32  ch03
float32  ch04
float32  ch05
float32  ch06
float32  ch07
float32  ch08
float32  ch09
float32  ch10

# ---- 开关量状态（0/1）----
uint8 switch06
uint8 switch07
