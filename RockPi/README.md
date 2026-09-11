# ROCK Pi ROS1 串口桥接

这个脚本把 ROS 的 `/cmd_vel` 速度消息转成 STM32 能识别的串口命令。

整体关系：

```text
ROS /cmd_vel
    ↓
ROCK Pi serial_car_bridge.py
    ↓ 串口
STM32 RemoteControl.c
    ↓
Motor.c / Car.c
    ↓
电机驱动
```

## 接线

如果用 ROCK Pi GPIO 串口直连 STM32 USART3：

```text
ROCK Pi TX  -> STM32 PB11 / USART3_RX
ROCK Pi RX  -> STM32 PB10 / USART3_TX
ROCK Pi GND -> STM32 GND
```

注意：两边必须共地，只用 3.3V 串口电平，不要接 5V。

如果用 USB-TTL 或 STM32 USB 虚拟串口，ROCK Pi 上一般会出现：

```bash
/dev/ttyUSB0
/dev/ttyACM0
```

查看串口设备：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

## 安装依赖

```bash
sudo apt install -y python3-pip
python3 -m pip install pyserial
```

## 直接运行

```bash
python3 serial_car_bridge.py _port:=/dev/ttyUSB0
```

如果你的串口是 `/dev/ttyACM0`，就改成：

```bash
python3 serial_car_bridge.py _port:=/dev/ttyACM0
```

## ROS 中运行

把 `serial_car_bridge.py` 放到 ROS 包的 `scripts/` 目录后：

```bash
chmod +x serial_car_bridge.py
rosrun your_package serial_car_bridge.py _port:=/dev/ttyUSB0
```

## 速度映射

脚本会把 `/cmd_vel` 映射成 STM32 命令：

```text
linear.x  -> vx，前进/后退
linear.y  -> vy，麦轮左移/右移
angular.z -> omega，左转/右转
```

发送给 STM32 的格式：

```text
M vx vy omega
```

例如：

```text
M 30 0 0      前进
M 0 30 0      左移
M 0 0 30      左转
S             停车
PING          通信测试
```

所有速度都会限制在 `-100..100`。STM32 端 500ms 收不到完整命令会自动停车。
