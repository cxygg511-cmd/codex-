#!/usr/bin/env python3
"""ROS1 bridge from /cmd_vel and raw string commands to STM32 USART3.

Topics:
  subscribe /cmd_vel             geometry_msgs/Twist, converted to M vx vy omega
  subscribe /car/raw_command     std_msgs/String, sent to STM32 as-is
  publish   /car/serial_rx       std_msgs/String, every STM32 response line

STM32 protocol examples:
  M 20 0 0
  S
  ?
  YAW0
  HOLD 1
  T 90
  D
"""

import threading

import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class SerialCarBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 9600)
        self.command_period = rospy.get_param("~command_period", 0.1)
        self.speed_scale = rospy.get_param("~speed_scale", 100.0)
        self.angular_scale = rospy.get_param("~angular_scale", 100.0)
        self.hold_cmd_timeout = rospy.get_param("~hold_cmd_timeout", 0.6)

        self.serial = serial.Serial(port, baudrate, timeout=0.2)
        self.lock = threading.Lock()
        self.last_command = (0, 0, 0)
        self.last_send_time = rospy.Time(0)
        self.last_cmd_vel_time = rospy.Time(0)
        self.has_cmd_vel = False
        self.pause_cmd_vel = False
        self.reader_running = True

        self.rx_pub = rospy.Publisher("/car/serial_rx", String, queue_size=50)
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd_vel, queue_size=1)
        rospy.Subscriber("/car/raw_command", String, self.on_raw_command, queue_size=10)
        rospy.on_shutdown(self.stop)

        self.reader_thread = threading.Thread(target=self.read_serial_loop)
        self.reader_thread.daemon = True
        self.reader_thread.start()

        rospy.loginfo("Connected to STM32 on %s at %d baud", port, baudrate)

    @staticmethod
    def clamp(value, low=-100, high=100):
        return max(low, min(high, int(round(value))))

    @staticmethod
    def is_turn_command(line):
        return line == "T" or line.startswith("T ") or line.startswith("TURN")

    def on_cmd_vel(self, message):
        vx = self.clamp(message.linear.x * self.speed_scale)
        vy = self.clamp(message.linear.y * self.speed_scale)
        omega = self.clamp(message.angular.z * self.angular_scale)
        self.last_command = (vx, vy, omega)
        self.last_cmd_vel_time = rospy.Time.now()
        self.has_cmd_vel = True
        self.send_current_command(force=True)

    def on_raw_command(self, message):
        line = message.data.strip()
        if not line:
            return

        if self.is_turn_command(line):
            self.pause_cmd_vel = True
            self.has_cmd_vel = False
            self.last_command = (0, 0, 0)
        elif line == "S" or line == "STOP":
            self.pause_cmd_vel = False
            self.has_cmd_vel = False
            self.last_command = (0, 0, 0)

        rospy.loginfo("STM32 raw command: %s", line)
        self.send_line(line)

    def publish_rx(self, line):
        msg = String()
        msg.data = line
        self.rx_pub.publish(msg)
        rospy.loginfo("STM32 rx: %s", line)

        if (line.startswith("OK TURN_DONE") or line.startswith("ERR TURN") or
                line.startswith("OK STOP")):
            self.pause_cmd_vel = False
            self.has_cmd_vel = False
            self.last_command = (0, 0, 0)

    def read_serial_loop(self):
        while self.reader_running and not rospy.is_shutdown():
            try:
                raw = self.serial.readline()
            except serial.SerialException as exc:
                rospy.logerr("Serial read failed: %s", exc)
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.publish_rx(line)

    def send_line(self, line):
        with self.lock:
            self.serial.write((line + "\r\n").encode("ascii", errors="ignore"))

    def send_current_command(self, force=False):
        if self.pause_cmd_vel or not self.has_cmd_vel:
            return

        now = rospy.Time.now()
        if (now - self.last_cmd_vel_time).to_sec() > self.hold_cmd_timeout:
            self.has_cmd_vel = False
            self.last_command = (0, 0, 0)
            self.send_line("S")
            return

        if not force and (now - self.last_send_time).to_sec() < self.command_period:
            return

        vx, vy, omega = self.last_command
        self.send_line("M {} {} {}".format(vx, vy, omega))
        self.last_send_time = now

    def stop(self):
        self.reader_running = False
        try:
            self.pause_cmd_vel = False
            self.has_cmd_vel = False
            self.send_line("S")
            self.serial.close()
        except serial.SerialException:
            pass

    def spin(self):
        rate = rospy.Rate(1.0 / self.command_period)
        while not rospy.is_shutdown():
            self.send_current_command()
            rate.sleep()


def main():
    rospy.init_node("stm32_serial_bridge")
    bridge = SerialCarBridge()
    bridge.spin()


if __name__ == "__main__":
    main()

