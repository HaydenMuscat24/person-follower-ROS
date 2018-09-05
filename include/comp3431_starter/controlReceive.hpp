/*
 * controlWrapper.hpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#ifndef COMP3431_STARTER_CONTROLWRAPPER_HPP_
#define COMP3431_STARTER_CONTROLWRAPPER_HPP_

#include <crosbot/handle.hpp>
#include <crosbot/controls/command.hpp>
#include <crosbot/controls/status.hpp>

#include <ros/publisher.h>
#include <ros/service.h>

namespace comp3431 {

class ControlReceiver : public crosbot::HandledObject {
public:
    ControlReceiver();
    virtual ~ControlReceiver();

    void configure();
    void startup();
    void shutdown();

    // Command callbacks
    void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command);

private:
    crosbot::CrosbotCommandPtr crosbotCommand;
    crosbot::CrosbotStatusPtr crosbotStatus;
};
typedef crosbot::Handle<ControlReceiver> ControlReceiverPtr;

} // namespace comp3431

#endif /* COMP3431_STARTER_CONTROLWRAPPER_HPP_ */
