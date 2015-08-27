#include <utility>
#include <string>
#include <iostream>
#include <cmath>

class MapUtils {
  public:
    static ExplorationMap toExplorationMap(Occupancy Grid, Odom, Transform listener) { 
      //TODO 
      // First convert odom data to x y coordinate using getPositinIndex ie start = getPositionIndex
      // Copy data

      return NULL; 
    }

    static std::pair<int,int> getPositionIndex(Odom,Occupancy Grid, Transform listener) {
      // first convert odom to map frame using transofrm listener
      // use occupancy grid meta data to convert and get x and y pair for the robot
      return NULL;
    }

    static TwistMessage calculateMovement(std::pair<int,int> robot, std::pair<int,int> destination, ODOM) {
      //Do some geometry shit also need to get current angle of  robot

      return NULL;
    }

    static double angleBetweenPoints(std::pair<int,int> p1, std::pair<int,int> p2) {
	int dX = p2.first - p1.first;
	int dY = p2.second - p2.second;
	
	double angleInRad = atan2(dY,dX);
	return angleInRad;
    }

   static double angleFromPose(Quaternion) {
	//calculate some angle in the x axis from the quaternion
   }


}
