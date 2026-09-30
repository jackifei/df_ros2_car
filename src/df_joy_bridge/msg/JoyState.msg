# 手柄桥接后的状态消息
std_msgs/Header header

# ---- 前 10 个通道映射后的微秒值（默认 1000~2000）----
# 说明：ch06/ch07 是开关通道，这里的 ch06/ch07 是它们的微秒值，
#       真正的开关状态请看下面的 switch06 / switch07。
uint16 ch01
uint16 ch02
uint16 ch03
uint16 ch04
uint16 ch05
uint16 ch06
uint16 ch07
uint16 ch08
uint16 ch09
uint16 ch10

# ---- 开关量状态（0/1）----
uint8 switch06
uint8 switch07
