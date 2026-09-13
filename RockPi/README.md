# ROCK Pi ROS Bridge

This folder contains the ROS-side code for the ROCK Pi / STM32 mecanum car.

## Files

- `serial_car_bridge.py`
  - Subscribes `/cmd_vel` and converts it to STM32 `M vx vy wz` commands.
  - Subscribes `/car/raw_command` and forwards short commands such as `?`, `D`, `S`, `YAW0`, `HOLD 1`, `T 90`.
  - Publishes STM32 responses to `/car/serial_rx`.

- `car_controller.h`
  - Reusable C++ control helper for mission nodes.
  - Provides `ping`, `stop`, `resetYaw`, `setHeadingHold`, `requestData`, `turnTo`, `moveForward`, `strafeLeft`, etc.

- `car_mission_node.cpp`
  - Example mission flow using `CarController`.

- `serial_test.py`
  - Pure Python serial test, no ROS required.

- `cmd_vel_test.py`
  - Publishes one short `/cmd_vel` test command.

## Copy To Catkin Package

Copy these two files into your ROS package source folder:

```bash
cp ~/Desktop/RockPi/car_controller.h ~/catkin_ws/src/car_demo/src/
cp ~/Desktop/RockPi/car_mission_node.cpp ~/catkin_ws/src/car_demo/src/
```

Then build:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Run

Terminal 1:

```bash
roscore
```

Terminal 2:

```bash
cd ~/Desktop/RockPi
python3 serial_car_bridge.py _port:=/dev/ttyUSB0 _baudrate:=9600 _speed_scale:=100 _angular_scale:=100
```

Terminal 3:

```bash
rostopic echo /car/serial_rx
```

Terminal 4:

```bash
rosrun car_demo car_mission_node
```
