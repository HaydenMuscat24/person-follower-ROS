/*
 * beacon.hpp
 *
 *  Created on: 13/07/2016
 *      Author: Timothy Wiley
 */

#ifndef COMP3431_STARTER_BEACON_HPP_
#define COMP3431_STARTER_BEACON_HPP_

#include <ros/node_handle.h>
#include <string>
#include <vector>

#define BEACON_BLUE         "blue"
#define BEACON_GREEN        "green"
#define BEACON_PINK         "pink"
#define BEACON_YELLOW       "yellow"

namespace comp3431 {

class Beacon {
public:
    int id;
    std::string top;
    std::string bottom;

    Beacon(int id = -1, std::string top = "", std::string bottom = "") :
        id(id), top(top), bottom(bottom)
    {};
    Beacon(const Beacon& other) :
        id(other.id), top(other.top), bottom(other.bottom)
    {};
    ~Beacon() {};


    static std::vector<Beacon> parseBeacons(ros::NodeHandle& paramNh);
};

}


#endif /* COMP3431_STARTER_BEACON_HPP_ */
