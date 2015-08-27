/*
 * beacons.h
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */

#ifndef COMP3431_BEACONS_H_
#define COMP3431_BEACONS_H_

#include <string>
#include <geometry_msgs/Point.h>

namespace comp3431 {

#define FLOAT_PARAM(P)		(((P).getType() == XmlRpc::XmlRpcValue::TypeInt)?(double)(int)(P):(double)(P))

struct Beacon {
public:
	std::string top, bottom;

	Beacon() {}
	Beacon(const Beacon& other) : top(other.top), bottom(other.bottom) {}
};

class Beacons {
public:
	std::vector< Beacon > beacons;

	Beacons();
};

} // namespace comp3431

#endif /* COMP3431_BEACONS_H_ */
