#ifndef CAR_CONTROLLER_H
#define CAR_CONTROLLER_H

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <cstddef>
#include <std_msgs/String.h>
#include <deque>
#include <string>

class CarController {
public:
    explicit CarController(ros::NodeHandle &nh)
        : nh_(nh), publish_rate_hz_(20.0) {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        raw_pub_ = nh_.advertise<std_msgs::String>("/car/raw_command", 10);
        serial_sub_ = nh_.subscribe("/car/serial_rx", 50, &CarController::onSerialRx, this);
    }

    void waitForBridge(void) {
        ROS_INFO("Waiting for serial bridge subscribers...");
        ros::Rate rate(10);
        while (ros::ok() && !bridgeReady()) {
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Serial bridge connected, ready to move");
    }

    bool ping(void) {
        return sendCommandWithRetry("?", "OK PONG", 4.0);
    }

    bool stop(double hold_seconds = 0.5) {
        publishVelocity(0.0, 0.0, 0.0, hold_seconds);
        return sendCommandWithRetry("S", "OK STOP", 4.0);
    }

    bool resetYaw(void) {
        ROS_INFO("Reset yaw");
        return sendCommandWithRetry("YAW0", "OK YAW0", 4.0);
    }

    bool setHeadingHold(bool enabled) {
        ROS_INFO("Heading hold: %s", enabled ? "on" : "off");
        return sendCommandWithRetry(enabled ? "HOLD 1" : "HOLD 0",
                                    enabled ? "OK HOLD_ON" : "OK HOLD_OFF", 4.0);
    }

    bool requestData(void) {
        return sendCommandWithRetry("D", "DATA", 5.0);
    }

    bool turnTo(double angle_deg, double timeout_sec = 20.0) {
        const std::string command = "T " + std::to_string(static_cast<int>(angle_deg));
        ROS_INFO("Turn to %.1f deg", angle_deg);

        if (!sendCommandOnce(command, "OK TURN", 4.0)) {
            stop(0.2);
            return false;
        }

        if (!waitForResponse("OK TURN_DONE", timeout_sec)) {
            ROS_ERROR("Turn timeout, sending emergency stop");
            stop(0.2);
            return false;
        }
        return true;
    }

    void moveForward(double speed, double seconds) {
        moveByVelocity(speed, 0.0, 0.0, seconds, "Move forward");
    }

    void moveBackward(double speed, double seconds) {
        moveByVelocity(-speed, 0.0, 0.0, seconds, "Move backward");
    }

    void strafeLeft(double speed, double seconds) {
        moveByVelocity(0.0, speed, 0.0, seconds, "Strafe left");
    }

    void strafeRight(double speed, double seconds) {
        moveByVelocity(0.0, -speed, 0.0, seconds, "Strafe right");
    }

    void rotateLeft(double speed, double seconds) {
        moveByVelocity(0.0, 0.0, speed, seconds, "Rotate left by speed");
    }

    void rotateRight(double speed, double seconds) {
        moveByVelocity(0.0, 0.0, -speed, seconds, "Rotate right by speed");
    }

private:
    bool bridgeReady(void) const {
        return cmd_pub_.getNumSubscribers() > 0 && raw_pub_.getNumSubscribers() > 0 &&
               serial_sub_.getNumPublishers() > 0;
    }

    void onSerialRx(const std_msgs::String::ConstPtr &msg) {
        serial_lines_.push_back(msg->data);
        while (serial_lines_.size() > kMaxCachedSerialLines) {
            serial_lines_.pop_front();
        }
    }

    bool sendCommandOnce(const std::string &command, const std::string &expected_prefix,
                         double timeout_sec) {
        std_msgs::String msg;
        msg.data = command;
        serial_lines_.clear();
        raw_pub_.publish(msg);
        ros::spinOnce();
        return waitForResponse(expected_prefix, timeout_sec);
    }

    bool sendCommandWithRetry(const std::string &command, const std::string &expected_prefix,
                              double timeout_sec) {
        const double retry_sec = 0.8;
        std_msgs::String msg;
        ros::Rate rate(50);
        ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
        ros::Time next_send = ros::Time(0);

        msg.data = command;
        serial_lines_.clear();

        while (ros::ok() && ros::Time::now() < deadline) {
            const ros::Time now = ros::Time::now();
            if (now >= next_send) {
                raw_pub_.publish(msg);
                next_send = now + ros::Duration(retry_sec);
            }

            ros::spinOnce();
            if (hasResponse(expected_prefix)) {
                return true;
            }
            if (hasError()) {
                return false;
            }
            rate.sleep();
        }

        ROS_ERROR("Timeout waiting for response prefix: %s", expected_prefix.c_str());
        return false;
    }

    bool waitForResponse(const std::string &expected_prefix, double timeout_sec) {
        ros::Rate rate(50);
        ros::Time deadline = ros::Time::now() + ros::Duration(timeout_sec);
        serial_lines_.clear();

        while (ros::ok() && ros::Time::now() < deadline) {
            ros::spinOnce();
            if (hasResponse(expected_prefix)) {
                return true;
            }
            if (hasError()) {
                return false;
            }
            rate.sleep();
        }

        ROS_ERROR("Timeout waiting for response prefix: %s", expected_prefix.c_str());
        return false;
    }

    bool hasResponse(const std::string &expected_prefix) {
        for (std::deque<std::string>::const_iterator it = serial_lines_.begin();
             it != serial_lines_.end(); ++it) {
            if (startsWith(*it, expected_prefix)) {
                ROS_INFO("Got response: %s", it->c_str());
                serial_lines_.clear();
                return true;
            }
        }
        return false;
    }

    bool hasError(void) {
        for (std::deque<std::string>::const_iterator it = serial_lines_.begin();
             it != serial_lines_.end(); ++it) {
            if (startsWith(*it, "ERR")) {
                ROS_ERROR("STM32 error: %s", it->c_str());
                serial_lines_.clear();
                return true;
            }
        }
        return false;
    }

    static bool startsWith(const std::string &text, const std::string &prefix) {
        return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
    }

    void moveByVelocity(double vx, double vy, double wz, double seconds, const char *label) {
        ROS_INFO("%s: vx=%.2f vy=%.2f wz=%.2f duration=%.2f", label, vx, vy, wz, seconds);
        publishVelocity(vx, vy, wz, seconds);
        stop();
    }

    void publishVelocity(double vx, double vy, double wz, double seconds) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.angular.z = wz;

        ros::Rate rate(publish_rate_hz_);
        const ros::Time end_time = ros::Time::now() + ros::Duration(seconds);
        while (ros::ok() && ros::Time::now() < end_time) {
            cmd_pub_.publish(cmd);
            ros::spinOnce();
            rate.sleep();
        }
    }

    static const size_t kMaxCachedSerialLines = 20;

    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher raw_pub_;
    ros::Subscriber serial_sub_;
    double publish_rate_hz_;
    std::deque<std::string> serial_lines_;
};

#endif

