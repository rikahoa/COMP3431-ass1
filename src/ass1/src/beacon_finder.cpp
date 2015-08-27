#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcException.h>

#include <cmath>
#include <iostream>
#include <string>

using namespace std;

class Beacon {
public:
    Beacon(string top, string bottom) : 
        x(0), y(0), known_location(false), top(top), bottom(bottom) {
        ROS_INFO_STREAM("Looking for beacon top=" << top << ",bottom=" << bottom);
    }
private:
    double x, y;
    bool known_location;
    string top, bottom;
    // COLOURS
};

class BeaconFinder {
public:
    BeaconFinder(ros::NodeHandle n, vector<Beacon> beacons) : n(n), beacons(beacons) {
        // location_pub = n.advertise<??>("/ass1/beacons", 1);
    }

private:
    ros::NodeHandle n;
    ros::Publisher location_pub;
    vector<Beacon> beacons;
};

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Convert from ROS image msg to OpenCV matrix images
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat src = cv_ptr->image;
        imshow("original", src);

        // OpenCV filters to find colours
        cv::Mat hsv, pink_threshold, yellow_threshold;
        cv::cvtColor(src, hsv, CV_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(140,90,90), cv::Scalar(180,255,255), pink_threshold);
        cv::inRange(hsv, cv::Scalar(20,90,90), cv::Scalar(30,255,255), yellow_threshold);


        // blob detection
        cv::SimpleBlobDetector::Params params; 
        params.filterByCircularity = true;
        params.minCircularity = 0.1;
        params.filterByArea = true;
        params.minArea = 100;
        
        std::vector<cv::KeyPoint> keypoints;
        cv::SimpleBlobDetector detector(params);
        detector.detect(pink_threshold, keypoints);
        cv::Mat blobs;
        cv::drawKeypoints( pink_threshold, keypoints, blobs, 
                cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        // gui display
        imshow("Blobs", blobs);
        imshow("Pink", pink_threshold);
        imshow("Yellow", yellow_threshold);

        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "beacon_finder");
    ros::NodeHandle n;

	XmlRpc::XmlRpcValue beacons_cfg;
	n.getParam("/beacons", beacons_cfg);
    std::vector<Beacon> beacons;

    try {
		int i = 0;
		do {
			char beacon_name[256];
			sprintf(beacon_name, "beacon%d", i);
			if (!beacons_cfg.hasMember(beacon_name)) {
				break;
            }

			XmlRpc::XmlRpcValue beacon_cfg = beacons_cfg[std::string(beacon_name)];
			if (!(beacon_cfg.hasMember("top") && beacon_cfg.hasMember("bottom"))) {
				continue;
            }

			beacons.push_back(Beacon((string) beacon_cfg["top"], (string) beacon_cfg["bottom"]));
		} while ((++i) != 0);
	} catch (XmlRpc::XmlRpcException& e) {
		ROS_ERROR("Unable to parse beacon parameter. (%s)", e.getMessage().c_str());
	}
    
    cv::namedWindow("Pink");
    cv::namedWindow("Yellow");
    cv::namedWindow("original");
    cv::namedWindow("blobs");
    cv::startWindowThread();

    BeaconFinder beacon_finder(n, beacons);

    image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("Pink");
    cv::destroyWindow("Yellow");
    cv::destroyWindow("original");
    cv::destroyWindow("blobs");

    return 0;
}
