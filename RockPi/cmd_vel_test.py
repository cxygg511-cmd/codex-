#!/usr/bin/env python3
"""Publish one /cmd_vel command for testing the STM32 serial bridge.

Examples:
  python3 cmd_vel_test.py --x 0.2 --duration 2
  python3 cmd_vel_test.py --y 0.2 --duration 2
  python3 cmd_vel_test.py --yaw 0.2 --duration 2
"""

import argparse

import rospy
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser(description="Publish a fixed /cmd_vel for a short test")
    parser.add_argument("--x", type=float, default=0.0, help="forward speed, m/s style value")
    parser.add_argument("--y", type=float, default=0.0, help="left strafe speed, m/s style value")
    parser.add_argument("--yaw", type=float, default=0.0, help="turn speed, rad/s style value")
    parser.add_argument("--duration", type=float, default=1.0, help="seconds to keep publishing")
    parser.add_argument("--rate", type=float, default=10.0, help="publish rate Hz")
    args = parser.parse_args()

    rospy.init_node("cmd_vel_test", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(args.rate)

    move = Twist()
    move.linear.x = args.x
    move.linear.y = args.y
    move.angular.z = args.yaw

    stop = Twist()
    end_time = rospy.Time.now() + rospy.Duration(args.duration)

    rospy.sleep(0.5)
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        pub.publish(move)
        rate.sleep()

    for _ in range(5):
        pub.publish(stop)
        rate.sleep()


if __name__ == "__main__":
    main()
