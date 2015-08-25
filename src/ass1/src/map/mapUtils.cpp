#include <utility>
#include <string>
#include <iostream>

class MapUtils {
  public:
    static ExplorationMap toExplorationMap(Occupancy Grid, Odom, Transform listener) { 
      //TODO 
      // First convert odom data to x y coordinate using getPositinIndex
      // Copy data

      return NULL; 
    }

    static std::pair<int,int> getPositionIndex(Odom,Occupancy Grid, Transform listener) {
      // first convert odom to map frame using transofrm listener
      // use occupancy grid meta data to convert and get x and y pair
      return NULL;
    }

    static TwistMessage calculateMovement(std::pair<int,int> robot, std::pair<int,int> destination, ODOM) {
      //Do some geometry shit also need to get current angle of  robot

      return NULL;
    }


}
