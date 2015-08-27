/*
 * StatusDisplay.cpp
 *
 *  Created on: 30/08/2014
 *      Author: mmcgill
 */

#include "status_display.hpp"

#include <sample/Go.h>

#include <QVBoxLayout>
#include <QPalette>
#include <QColor>

namespace comp3431 {

StatusDisplay::StatusDisplay(QWidget* parent) :
	rviz::Panel( parent ), label("Hello World"), goButton("Go!")
{
	label.setAlignment(Qt::AlignCenter);

	QVBoxLayout* layout = new QVBoxLayout;
	layout->addWidget(&label);
	layout->addWidget(&goButton);
	setLayout(layout);


	connect( &goButton, SIGNAL( clicked()), this, SLOT( go()));

	status = nh.subscribe("sample/status", 1, &StatusDisplay::callbackStatus, this);
	goClient = nh.serviceClient< sample::Go >("sample/go", true);
}

void StatusDisplay::callbackStatus(const sample::StatusConstPtr& status) {
	QPalette sample_palette;
	QColor qcolor(status->colour.r, status->colour.g, status->colour.b, 255);
	float avg = (status->colour.r + status->colour.g + status->colour.b) / 3;

	sample_palette.setColor(QPalette::Background, qcolor);
	if (avg < 128) {
		sample_palette.setColor(QPalette::WindowText, Qt::white);
	} else {
		sample_palette.setColor(QPalette::WindowText, Qt::black);
	}

	label.setAutoFillBackground(true);
	label.setPalette(sample_palette);

	label.setText(QString(status->status.c_str()));

}

void StatusDisplay::go() {
	ROS_INFO("Go button pressed.");
	if (!goClient) {
		goClient = nh.serviceClient< sample::Go >("sample/go", true);
		if (!goClient) {
			ROS_WARN("Failed to reconnect to Go service.");
		}
	}

	if (goClient) {
		sample::Go g;
		if (!goClient.call(g)) {
			ROS_ERROR("Error calling Go service.");
			goClient.shutdown();
		}
	}

}

} // namespace comp3431


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(comp3431::StatusDisplay,rviz::Panel )
