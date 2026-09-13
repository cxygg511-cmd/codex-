#include "car_controller.h"
#include "ros/ros.h"

static bool runBasicMotionMission(CarController &car) {
    if (!car.ping()) {
        ROS_ERROR("STM32 is not responding, abort mission");
        return false;
    }

    car.stop(1.0);
    car.resetYaw();
    car.requestData();

    car.setHeadingHold(true);
    car.moveForward(0.20, 2.0);
    car.moveBackward(0.20, 2.0);
    car.strafeLeft(0.20, 2.0);
    car.strafeRight(0.20, 2.0);

    car.setHeadingHold(false);
    if (!car.turnTo(90.0)) {
        ROS_ERROR("Mission aborted during T 90");
        return false;
    }
    car.requestData();

    if (!car.turnTo(0.0)) {
        ROS_ERROR("Mission aborted during T 0");
        return false;
    }
    car.requestData();
    car.stop(1.0);

    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "car_mission_node");
    ros::NodeHandle nh;

    CarController car(nh);
    car.waitForBridge();

    ROS_INFO("Mission start");
    if (!runBasicMotionMission(car)) {
        return 1;
    }

    ROS_INFO("Mission complete");
    return 0;
}
