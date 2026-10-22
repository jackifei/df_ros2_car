# 这是一个示例 Python 脚本。

# 按 Shift+F10 执行或将其替换为您的代码。
# 按 双击 Shift 在所有地方搜索类、文件、工具窗口、操作和设置。
import math
import threading
import time

import pymodbus
from pymodbus.client import ModbusSerialClient

import serial


# 按装订区域中的绿色按钮以运行脚本。
if __name__ == '__main__':
    ser = ModbusSerialClient(
        port="/dev/ttyUSB0",
        baudrate=115200,
        parity='N',
        stopbits=1,
        bytesize=8
    )
    ser.connect()
    time.sleep(0.1)
    state = 'open'


    # 所有串口写入都通过请求标志交给通讯线程，保证串口单线程访问
    resp = ser.read_holding_registers(address=58, count=1, device_id=1)

    print(resp.registers)

    ser.close()
    time.sleep(0.1)
    print("qiut")

# 访问 https://www.jetbrains.com/help/pycharm/ 获取 PyCharm 帮助
