/*
 * beacons.cpp
 *
 *  Created on: 25/08/2013
 *      Author: rescue
 */


#include <sample/beacons.hpp>
#include <ros/ros.h>
#include <XmlRpcException.h>

namespace comp3431 {

Beacons::Beacons() {
	ros::NodeHandle nh;
	XmlRpc::XmlRpcValue beaconsCfg;
	nh.getParam("/beacons", beaconsCfg);
	try {
		int i = 0;
		do {
			char beaconName[256];
			sprintf(beaconName, "beacon%d", i);
			if (!beaconsCfg.hasMember(beaconName))
				break;

			XmlRpc::XmlRpcValue beaconCfg = beaconsCfg[std::string(beaconName)];
			Beacon b;
			if (!(beaconCfg.hasMember("top") && beaconCfg.hasMember("bottom")))
				continue;
			b.top = (std::string)beaconCfg[("top")];
			b.bottom = (std::string)beaconCfg["bottom"];

			beacons.push_back(b);
		} while ((++i) != 0);
	} catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR("Unable to parse beacon parameter. (%s)", e.getMessage().c_str());
	}
}

} // namespace comp3431
