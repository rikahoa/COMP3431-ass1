#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <iostream>


class Beacon {
    private:
        double x, y;
        // COLOURS
};

class BeaconFinder {
    public:
        
        BeaconFinder(ros::NodeHandle n) : n(n) {
            // location_pub = n.advertise<??>("/ass1/beacons", 1);
        }

    private:
        ros::NodeHandle n;
        ros::Publisher location_pub;
};


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {

        // Convert from ROS image msg to OpenCV matrix images
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat src = cv_ptr->image;

        // OpenCV filters
        Mat hsv, threshold;
        cv::cvtColor( src, hsv, CV_BGR2HSV);
        cv::inRange( hsv, Scalar(150,120,120), Scalar(170,255,255), threshold);

        // gui display
       // imshow("view", threshold);
       // imshow("orig", src);

        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}



int main(int argc, char *argv[]) {
    ros::init(argc, argv, "beacon_finder");
    ros::NodeHandle n;

    BeaconFinder beacon_finder(n);

    image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("image_raw", 1, imageCallback);

    ros::spin();

    return 0;
}
