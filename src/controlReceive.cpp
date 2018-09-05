/*
 * controlWrapper.cpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <comp3431_starter/common.hpp>
#include <comp3431_starter/controlReceive.hpp>

#include <std_msgs/String.h>


#define LOG_START   "ControlReceiver ::"

namespace comp3431 {

class _CommandCallbackReceiver : public crosbot::CrosbotCommandCallback {
private:
    ControlReceiverPtr receiver;

public:
    _CommandCallbackReceiver(ControlReceiverPtr receiver) : receiver(receiver) {};
    virtual ~_CommandCallbackReceiver() {};
    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
        if (receiver != NULL) receiver->callback_receivedCommand(command);
    };
};

ControlReceiver::ControlReceiver() {
}

ControlReceiver::~ControlReceiver() {
}

void ControlReceiver::configure() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");
}

void ControlReceiver::startup() {
    // Use "local" namespace for config
    ros::NodeHandle nh("~");

    crosbotCommand = new crosbot::CrosbotCommand(TURTLEBOT_CONTROL_NAMESPACE, true,
            new _CommandCallbackReceiver(this));
    crosbotStatus = new crosbot::CrosbotStatus(TURTLEBOT_CONTROL_NAMESPACE);
}

void ControlReceiver::shutdown() {
    crosbotCommand = NULL;
    crosbotStatus = NULL;
}

void ControlReceiver::callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
    if (command->command == crosbot_msgs::ControlCommand::CMD_RESET) {
        crosbotStatus->sendStatus(crosbot_msgs::ControlStatus::LEVEL_INFO,
                "Completed reset command");
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_START) {
        crosbotStatus->sendStatus(crosbot_msgs::ControlStatus::LEVEL_INFO,
                "Completed start command");
    } else if (command->command == crosbot_msgs::ControlCommand::CMD_STOP) {
        crosbotStatus->sendStatus(crosbot_msgs::ControlStatus::LEVEL_INFO,
                "Completed stop command");
    }
}

} // namespace comp3431


