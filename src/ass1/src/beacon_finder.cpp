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
        x(0), y(0), known_location(false), 
        top(top), bottom(bottom) {
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
    BeaconFinder(ros::NodeHandle n, vector<Beacon> beacons) : n(n), pnh("~"),
        beacons(beacons),
        img_sub(n, "/camera/rgb/image_color", 1),
        laser_sub(n, "/scan", 1),
        sync(SyncPolicy(10), laser_sub, img_sub) {

        odom_sub = n.subscribe("ass1/odom", 1, &BeaconFinder::odom_callback, this);
        beacons_pub = n.advertise<ass1::FoundBeacons>("/ass1/beacons", 1);

        // Use ApproximateTime message_filter to read both kinect image and laser.
        sync.registerCallback(boost::bind(&BeaconFinder::image_callback, this, _1, _2));

        // Get colours from params

        XmlRpc::XmlRpcValue colour_thresholds;
        n.getParam("/beacon_finder/colour_ranges/pink", colour_thresholds);
        pink_ranges[0] = (int)colour_thresholds["h_lo"];
        pink_ranges[1] = (int)colour_thresholds["h_hi"];
        pink_ranges[2] = (int)colour_thresholds["s"];
        pink_ranges[3] = (int)colour_thresholds["v"];
        n.getParam("/beacon_finder/colour_ranges/blue", colour_thresholds);
        blue_ranges[0] = (int)colour_thresholds["h_lo"];
        blue_ranges[1] = (int)colour_thresholds["h_hi"];
        blue_ranges[2] = (int)colour_thresholds["s"];
        blue_ranges[3] = (int)colour_thresholds["v"];
        n.getParam("/beacon_finder/colour_ranges/green", colour_thresholds);
        green_ranges[0] = (int)colour_thresholds["h_lo"];
        green_ranges[1] = (int)colour_thresholds["h_hi"];
        green_ranges[2] = (int)colour_thresholds["s"];
        green_ranges[3] = (int)colour_thresholds["v"];
        n.getParam("/beacon_finder/colour_ranges/yellow", colour_thresholds);
        yellow_ranges[0] = (int)colour_thresholds["h_lo"];
        yellow_ranges[1] = (int)colour_thresholds["h_hi"];
        yellow_ranges[2] = (int)colour_thresholds["s"];
        yellow_ranges[3] = (int)colour_thresholds["v"];
        for( int i=0; i < 4; i++ ) {
            ROS_INFO("PINK %i = %d", i, pink_ranges[i] );
        }
        for( int i=0; i < 4; i++ ) {
            ROS_INFO("BLUE %i = %d", i, blue_ranges[i] );
        }
        for( int i=0; i < 4; i++ ) {
            ROS_INFO("GREEN %i = %d", i, green_ranges[i] );
        }
        for( int i=0; i < 4; i++ ) {
            ROS_INFO("YELLOW %i = %d", i, yellow_ranges[i] );
        }


   }

private:
    ros::NodeHandle n;
    ros::NodeHandle pnh;
    vector<Beacon> beacons;
    Bot bot;
    message_filters::Subscriber<sensor_msgs::Image> img_sub;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;

    ros::Publisher beacons_pub;
    ros::Subscriber odom_sub;

    message_filters::Synchronizer<SyncPolicy> sync;

    int pink_ranges[4];
    int blue_ranges[4];
    int green_ranges[4];
    int yellow_ranges[4];

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

            // Take hsv ranges from launch

            // OpenCV filters to find colours
            cv::Mat hsv, pink_threshold, yellow_threshold, blue_threshold, green_threshold;
            cv::cvtColor(src, hsv, CV_BGR2HSV);

            cv::inRange(hsv, cv::Scalar(pink_ranges[0],pink_ranges[2],pink_ranges[3]), cv::Scalar(pink_ranges[1],255,255), pink_threshold);
            cv::inRange(hsv, cv::Scalar(yellow_ranges[0],yellow_ranges[2],yellow_ranges[3]), cv::Scalar(yellow_ranges[1],255,255), yellow_threshold);
            cv::inRange(hsv, cv::Scalar(blue_ranges[0],blue_ranges[2],blue_ranges[3]), cv::Scalar(blue_ranges[1],255,255), blue_threshold);
            cv::inRange(hsv, cv::Scalar(green_ranges[0],green_ranges[2],green_ranges[3]), cv::Scalar(green_ranges[1],255,255), green_threshold);
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
                xCo = 320 - xCo;
                // TODO: you might want atan2
                double theta = atan2(xCo*tan(29 * M_PI / 180), 320.0);
		//ROS_INFO_STREAM("theta: " << (theta * 180 / M_PI));
                double lTheta = theta - laser->angle_min;
                int distance_index = lTheta / laser->angle_increment;
                double distance = laser->ranges[distance_index];
                if( std::isfinite(distance) && std::isfinite(theta) ) { 
                    distance = distance - 0.1;
            //ROS_INFO_STREAM("theta " << (theta * 180 / M_PI) << " distance " << distance);
                    search_for_match(blue_keypoints.begin(), blue_keypoints.end(), pink_pt, "blue", make_pair(distance, theta));
                    search_for_match(yellow_keypoints.begin(), yellow_keypoints.end(), pink_pt, "yellow", make_pair(distance, theta));
                    search_for_match(green_keypoints.begin(), green_keypoints.end(), pink_pt, "green", make_pair(distance, theta));
                }
            }

            // gui display
            imshow("blobs", blobs);

            bool found_all = true; 
            int count = 0;
            for (auto it = beacons.begin(); it != beacons.end(); ++it) {
                if (!it->found()) {
                    found_all = false;
                } else {
                    ROS_INFO_STREAM("Found T: " << it->top << " B: " << it->bottom << " at " << it->x << " " << it->y);
                    count++;
                }
            }

            ROS_INFO_STREAM("Found  " << count << " of " << beacons.size() << " beacons.");
            if (found_all && this->bot.valid()) {
                ROS_INFO("Found all beacons");
                
                // Send beacon message
                ass1::FoundBeacons msg;
                msg.n = beacons.size();
                for (size_t i = 0; i < beacons.size(); ++i) {
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
        ROS_DEBUG_STREAM("BOT POS: " << b_x << "," << b_y << "beacon_yaw: " << 
                beacon_yaw << " d: " << d << "robot_yaw " << bot.get_yaw() << "theta: " << theta);
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
    ROS_INFO_STREAM("SPINNING");



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
