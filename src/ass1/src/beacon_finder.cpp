#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Point.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcException.h>
#include "ass1lib/bot.h"
#include "nav_msgs/Odometry.h"
#include "ass1/FoundBeacons.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace std;

typedef message_filters::sync_policies::
    ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> SyncPolicy;

class Beacon {
public:
    Beacon(string top, string bottom) : 
        x(0), y(0), known_location(false), top(top), bottom(bottom) {
        ROS_INFO_STREAM("Looking for beacon top=" << top << ",bottom=" << bottom);
    }
    bool found() { return known_location; }

    double x, y;
    bool known_location;
    string top, bottom;
    // COLOURS
};


class BeaconFinder {
public:
    BeaconFinder(ros::NodeHandle n, vector<Beacon> beacons) : n(n),
        beacons(beacons),
        img_sub(n, "/camera/rgb/image_color", 1),
        laser_sub(n, "/scan", 1),
        sync(SyncPolicy(10), laser_sub, img_sub) {

        odom_sub = n.subscribe("ass1/odom", 1, &BeaconFinder::odom_callback, this);
        beacons_pub = n.advertise<ass1::FoundBeacons>("/ass1/beacons", 1);

        // Use ApproximateTime message_filter to read both kinect image and laser.
        sync.registerCallback(boost::bind(&BeaconFinder::image_callback, this, _1, _2));
   }

private:
    ros::NodeHandle n;
    vector<Beacon> beacons;
    Bot bot;

    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

    ros::Publisher beacons_pub;
    ros::Subscriber odom_sub;

    message_filters::Synchronizer<SyncPolicy> sync;

    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
        this->bot.update(msg);
    }

    void image_callback(const sensor_msgs::LaserScan::ConstPtr& laser, 
                        const sensor_msgs::Image::ConstPtr& image) {
        try {
            // Convert from ROS image msg to OpenCV matrix images
            cv_bridge::CvImagePtr cv_ptr;
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
            cv::Mat src = cv_ptr->image;

            // OpenCV filters to find colours
            cv::Mat hsv, pink_threshold, yellow_threshold, blue_threshold, green_threshold;
            cv::cvtColor(src, hsv, CV_BGR2HSV);
            cv::inRange(hsv, cv::Scalar(130,90,90), cv::Scalar(170,255,255), pink_threshold);
            cv::inRange(hsv, cv::Scalar(20,120,50), cv::Scalar(40,255,255), yellow_threshold);
            cv::inRange(hsv, cv::Scalar(91,50,50), cv::Scalar(120,255,255), blue_threshold);
            cv::inRange(hsv, cv::Scalar(70,20,20), cv::Scalar(90,255,255), green_threshold);
            
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
            params.minInertiaRatio = 0.65;
            
            // find keypoints
            std::vector<cv::KeyPoint> pink_keypoints, yellow_keypoints, blue_keypoints, green_keypoints;

            cv::SimpleBlobDetector detector(params);
            detector.detect(pink_threshold, pink_keypoints);
            detector.detect(yellow_threshold, yellow_keypoints);
            detector.detect(blue_threshold, blue_keypoints);
            detector.detect(green_threshold, green_keypoints);
            
            cv::Mat blobs;
            cv::drawKeypoints (src, pink_keypoints, blobs, 
                    cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::drawKeypoints (blobs, blue_keypoints, blobs, 
                    cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::drawKeypoints (blobs, green_keypoints, blobs, 
                    cv::Scalar(0,255,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::drawKeypoints (blobs, yellow_keypoints, blobs, 
                    cv::Scalar(125,125,125), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            // acceptable horizontal distance between the 2 colours on a pillar
            for (auto pink_pt = pink_keypoints.begin(); pink_pt != pink_keypoints.end(); pink_pt++) {
                double xCo = pink_pt->pt.x;
                xCo = xCo - 320;
                // TODO: you might want atan2
                double theta = atan2(xCo*tan(0.48), 320.0);
		ROS_INFO_STREAM("theta: " << theta);
                double lTheta = theta - laser->angle_min;
                int distance_index = lTheta / laser->angle_increment;
                double distance = laser->ranges[distance_index];

                search_for_match(blue_keypoints.begin(), blue_keypoints.end(), pink_pt, "blue", make_pair(distance, -theta));
                search_for_match(yellow_keypoints.begin(), yellow_keypoints.end(), pink_pt, "yellow", make_pair(distance, -theta));
                search_for_match(green_keypoints.begin(), green_keypoints.end(), pink_pt, "green", make_pair(distance, -theta));
            }

            // gui display
            imshow("blobs", blobs);

            bool found_all = true; 
            int count = 0;
            for (auto it = beacons.begin(); it != beacons.end(); ++it) {
                if (!it->found()) {
                    found_all = false;
                } else {
                    count++;
                }
            }

            ROS_INFO_STREAM("Found  " << count << " of " << beacons.size() << " beacons.");
            if (found_all && this->bot.valid()) {
                ROS_INFO("Found all beacons");
                
                // Send beacon message
                ass1::FoundBeacons msg;
                msg.n = beacons.size();
                for (int i = 0; i < beacons.size(); ++i) {
                    geometry_msgs::Point point;
                    point.x = beacons[i].x;
                    point.y = beacons[i].y;
                    point.z = 0;
                    msg.positions.push_back(point);
                }
                beacons_pub.publish(msg);

                //shutdown subscriptions
                img_sub.unsubscribe();
                laser_sub.unsubscribe();
                odom_sub.shutdown();
            }

            cv::waitKey(30);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
        }
    }

    void found_beacon(string top, string bottom, pair<double, double> position) {
        double d = position.first;
        double theta = position.second;
        pair<double, double> bot_pos = bot.get_position();
        double beacon_yaw = bot.get_yaw() + theta;
        double b_x = bot_pos.first + d*cos(beacon_yaw);
        double b_y = bot_pos.second + d*sin(beacon_yaw);
        ROS_INFO_STREAM("BOT POS: " << b_x << "," << b_y << "beacon_yaw: " << beacon_yaw << " d: " << d << "robot_yaw " << bot.get_yaw() << "theta: " << theta);
        for (auto it = beacons.begin(); it != beacons.end(); ++it) {
            if (it->top == top && it->bottom == bottom) {
                it->known_location = true;
                it->x = b_x;
                it->y = b_y;
            }
        }
    }

    #define PILLAR_THRESHOLD 20
    void search_for_match(
            std::vector<cv::KeyPoint>::iterator begin, 
            std::vector<cv::KeyPoint>::iterator end, 
            std::vector<cv::KeyPoint>::iterator pink, 
            string colour,
            pair<double, double> position) 
    {
        for (auto it = begin; it != end; ++it) {
            if (std::abs(pink->pt.x - it->pt.x) < PILLAR_THRESHOLD) {
                if (pink->pt.y < it->pt.y) {
                    found_beacon("pink", colour, position); 
                } else {
                    found_beacon(colour, "pink", position);
                }
            }
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
