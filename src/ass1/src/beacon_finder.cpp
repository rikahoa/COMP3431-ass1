#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcException.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <iostream>
#include <string>

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> SyncPolicy;

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
    BeaconFinder(ros::NodeHandle n, vector<Beacon> beacons) : n(n),
        beacons(beacons),
        img_msg(n, "/camera/rgb/image_color", 1),
        lsr_msg(n, "/scan", 1),
        sync(SyncPolicy(10), lsr_msg, img_msg) {

        // Use ApproximateTime message_filter to read both kinect image and laser.
        sync.registerCallback(boost::bind(&BeaconFinder::image_callback, this, _1, _2) );
    }

private:
    ros::NodeHandle n;
    vector<Beacon> beacons;
    message_filters::Subscriber<sensor_msgs::Image> img_msg;
    message_filters::Subscriber<sensor_msgs::LaserScan> lsr_msg;
    message_filters::Synchronizer<SyncPolicy> sync;

    void image_callback(const sensor_msgs::LaserScan::ConstPtr& laser, const sensor_msgs::Image::ConstPtr& image) {
        try {
            // Convert from ROS image msg to OpenCV matrix images
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            cv::Mat src = cv_ptr->image;

            // OpenCV filters to find colours
            cv::Mat hsv, pink_threshold, yellow_threshold, blue_threshold, green_threshold;
            cv::cvtColor(src, hsv, CV_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(130,90,90), cv::Scalar(170,255,255), pink_threshold);
            cv::inRange(hsv, cv::Scalar(15,50,50), cv::Scalar(30,255,255), yellow_threshold);
            cv::inRange(hsv, cv::Scalar(91,90,90), cv::Scalar(120,255,255), blue_threshold);
            cv::inRange(hsv, cv::Scalar(70,50,50), cv::Scalar(90,255,255), green_threshold);
            
            blue_threshold = cv::Scalar::all(255) - blue_threshold;
            pink_threshold = cv::Scalar::all(255) - pink_threshold;
            yellow_threshold = cv::Scalar::all(255) - yellow_threshold;
            green_threshold = cv::Scalar::all(255) - green_threshold;

            // blob detection
            cv::SimpleBlobDetector::Params params; 
            
            params.filterByArea = 1;
            params.minArea = 200;
            params.maxArea = 100000;
            params.filterByConvexity = 1;
            params.minConvexity = 0.5;
            params.filterByInertia = true;
            params.minInertiaRatio = 0.5;
            
            std::vector<cv::KeyPoint> pink_keypoints, yellow_keypoints, blue_keypoints, green_keypoints;

            cv::SimpleBlobDetector detector(params);
            detector.detect(pink_threshold, pink_keypoints);
            detector.detect(yellow_threshold, yellow_keypoints);
            detector.detect(blue_threshold, blue_keypoints);
            detector.detect(green_threshold, green_keypoints);
            
            cv::Mat blobs;
            cv::drawKeypoints( src, pink_keypoints, blobs, 
                    cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            cv::drawKeypoints( blobs, blue_keypoints, blobs, 
                    cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            cv::drawKeypoints( blobs, green_keypoints, blobs, 
                    cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

            cv::drawKeypoints( blobs, yellow_keypoints, blobs, 
                    cv::Scalar(125,125,125), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
           

            // acceptable horizontal distance between the 2 colours on a pillar
            double pillar_threshold = 20;

            for( std::vector<cv::KeyPoint>::iterator pink_pt = pink_keypoints.begin(); pink_pt != pink_keypoints.end(); pink_pt++ ) {

                double xCo = pink_pt->pt.x;
                xCo = xCo - 320;
                double theta = atan(xCo*tan(27.5)/320.0);
                double lTheta = theta - laser->angle_min;
                int distance_index = lTheta / laser->angle_increment;
                double distance = laser->ranges[distance_index];


                for(std::vector<cv::KeyPoint>::iterator blue_pt = blue_keypoints.begin(); blue_pt != blue_keypoints.end(); blue_pt++ ) {
                    if( std::abs(pink_pt->pt.x - blue_pt->pt.x) < pillar_threshold ) {
                        if( pink_pt->pt.y < blue_pt->pt.y ) {
                            ROS_INFO("T: pink, B: blue, dist(%f), angle(%f)", distance, lTheta );
                        } else {
                            ROS_INFO("T: blue, B: pink, dist(%f), angle(%f)", distance, lTheta );
                        }
                    }
                }
                for(std::vector<cv::KeyPoint>::iterator yellow_pt = yellow_keypoints.begin(); yellow_pt!= yellow_keypoints.end(); yellow_pt++ ) {
                    if( std::abs(pink_pt->pt.x - yellow_pt->pt.x) < pillar_threshold ) {
                        if( pink_pt->pt.y < yellow_pt->pt.y ) {
                            ROS_INFO("T: pink, B: yellow, dist(%f), angle(%f)", distance, lTheta );
                        } else {
                            ROS_INFO("T: yellow, B: pink, dist(%f), angle(%f)", distance, lTheta );
                        }
                    }
                }
                for(std::vector<cv::KeyPoint>::iterator green_pt = green_keypoints.begin(); green_pt!= green_keypoints.end(); green_pt++ ) {
                    if( std::abs(pink_pt->pt.x - green_pt->pt.x) < pillar_threshold ) {
                        if( pink_pt->pt.y < green_pt->pt.y ) {
                            ROS_INFO("T: pink, B: green, dist(%f), angle(%f)", distance, lTheta );
                        } else {
                            ROS_INFO("T: green, B: pink, dist(%f), angle(%f)", distance, lTheta );
                        }
                    }
                }


            }

            // gui display
            imshow("blobs", blobs);

            cv::waitKey(30);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
        }
    }
};

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

    cv::namedWindow("blobs");
    cv::startWindowThread();

    BeaconFinder beacon_finder(n, beacons);
    
    ros::spin();
    cv::destroyWindow("blobs");

    return 0;
}
