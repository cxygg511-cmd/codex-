#!/usr/bin/env python3
"""ROS1 /cmd_vel to STM32 USART3 bridge.

STM32 protocol:
  M vx vy omega\n   vx    forward speed, -100..100
   vy    left speed, -100..100
   omega left turn speed, -100..100
  S\n       stop
  PING\n   PONG
"""

import threading

import rospy
import serial
from geometry_msgs.msg import Twist


class SerialCarBridge:
    def __init__(self):
        port = rospy.get_param("~port", "/dev/ttyUSB0")
        baudrate = rospy.get_param("~baudrate", 115200)
        self.command_period = rospy.get_param("~command_period", 0.1)
        self.speed_scale = rospy.get_param("~speed_scale", 100.0)
        self.angular_scale = rospy.get_param("~angular_scale", 100.0)

        self.serial = serial.Serial(port, baudrate, timeout=0.2)
        self.lock = threading.Lock()
        self.last_command = (0, 0, 0)
        self.last_send_time = rospy.Time(0)

        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd_vel, queue_size=1)
        rospy.on_shutdown(self.stop)

        rospy.loginfo("Connected to STM32 on %s at %d baud", port, baudrate)
        self.send_line("PING")

    @staticmethod
    def clamp(value, low=-100, high=100):
        return max(low, min(high, int(round(value))))

    def on_cmd_vel(self, message):
        vx = self.clamp(message.linear.x * self.speed_scale)
        vy = self.clamp(message.linear.y * self.speed_scale)
        omega = self.clamp(message.angular.z * self.angular_scale)
        self.last_command = (vx, vy, omega)
        self.send_current_command(force=True)

    def send_line(self, line):
        with self.lock:
            self.serial.write((line + "\n").encode("ascii"))

    def send_current_command(self, force=False):
        now = rospy.Time.now()
        if not force and (now - self.last_send_time).to_sec() < self.command_period:
            return

        vx, vy, omega = self.last_command
        self.send_line("M {} {} {}".format(vx, vy, omega))
        self.last_send_time = now

    def stop(self):
        try:
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
