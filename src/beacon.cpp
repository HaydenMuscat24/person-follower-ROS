/*
 * comp3431.cpp
 *
 *  Created on: 13/07/2016
 *      Author: Timothy Wiley
 */


#include <ros/ros.h>

#include <comp3431_starter/beacon.hpp>

#include <XmlRpcException.h>

#define LOG_START       "COMP3431 ::"

namespace comp3431 {

bool checkBeaconColour(std::string beaconColour, std::vector<std::string>& validColours) {
    bool valid = false;
    for (std::string& colour : validColours) {
        if (beaconColour == colour) {
            valid = true;
        }
    }
    return valid;
};

std::vector<Beacon> Beacon::parseBeacons(ros::NodeHandle& paramNh) {
    std::vector<std::string> validColours;
    validColours.push_back(BEACON_BLUE);
    validColours.push_back(BEACON_GREEN);
    validColours.push_back(BEACON_PINK);
    validColours.push_back(BEACON_YELLOW);

    XmlRpc::XmlRpcValue beaconsConfig;
    paramNh.getParam("beacons", beaconsConfig);

    std::vector<Beacon> beacons;
    try {
        // Iterate through config members
        for(int i = 0; i != beaconsConfig.size(); ++i) {
            XmlRpc::XmlRpcValue beaconConfig = beaconsConfig[i];
            
            // Check type
            if (beaconConfig.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                 if (beaconConfig.hasMember("id")
                         && beaconConfig.hasMember("top")
                         && beaconConfig.hasMember("bottom")) {
                     Beacon b;
                     b.id = static_cast<int>(beaconConfig["id"]);
                     b.top = static_cast<std::string>(beaconConfig["top"]);
                     b.bottom = static_cast<std::string>(beaconConfig["bottom"]);

                     bool topValid = checkBeaconColour(b.top, validColours);
                     bool bottomValid = checkBeaconColour(b.bottom, validColours);
                     if (!topValid) {
                         XmlRpc::XmlRpcException exception("Invalid colour '" + b.top + "' (top)");
                         throw exception;
                     }
                     if (!bottomValid) {
                      XmlRpc::XmlRpcException exception("Invalid colour '" + b.bottom + "' (bottom)");
                      throw exception;
                     }

                     beacons.push_back(b);
                     ROS_INFO("%s Loaded beacon: %d: %s-%s", LOG_START, b.id, b.top.c_str(), b.bottom.c_str());
                 } else {
                     XmlRpc::XmlRpcException exception("Missing beacon parameter. Requires id, top, bottom");
                     throw exception;
                 }
            } else {
                XmlRpc::XmlRpcException exception("Incorrect format for beacon config");
                throw exception;
            }
        }
    } catch (XmlRpc::XmlRpcException& e) {
        ROS_ERROR("Unable to parse beacon parameter. (%s)", e.getMessage().c_str());
        throw e;
    }

    return beacons;
}

} // namespace comp3431

