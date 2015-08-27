/*
 * StatusDisplay.hpp
 *
 *  Created on: 30/08/2014
 *      Author: mmcgill
 */

#ifndef COMP3431_STATUSDISPLAY_HPP_
#define COMP3431_STATUSDISPLAY_HPP_

#include <ros/ros.h>
#include <rviz/panel.h>

#include <sample/Status.h>

#include <QObject>
#include <QWidget>
#include <QLabel>
#include <QPushButton>

namespace comp3431 {

class StatusDisplay : public rviz::Panel
{
Q_OBJECT
protected:
	ros::NodeHandle nh;
	ros::Subscriber status;
	ros::ServiceClient goClient;

	QLabel label;
	QPushButton goButton;

public:
	void callbackStatus(const sample::StatusConstPtr& status);
	StatusDisplay(QWidget* parent = 0);

public Q_SLOTS:
	void go();
};

} // namespace comp3431

#endif /* COMP3431_STATUSDISPLAY_HPP_ */
