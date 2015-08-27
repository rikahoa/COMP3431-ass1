#include <utility>
#include <string>
#include <iostream>

class ExplorationMap {
	private:
	  int width;
	  int height;
	  std::vector<std::vector<int> map;

	  std::pair<int,int> start;

  public: 
	  ExplorationMap(int awidth, int aheight,   std::vector<std::vector<int> amap, int robotx, int roboty) {
	    map = amap;
	    width = awidth;
      height =  aheight;
	    start = std::make_pair(robotx,roboty);
	  }

	  std::pair<int,int> firstPointOnPathToNearestUnknown() {
		  //TODO Call Path To Nearest Unknown and get first one
		  std::pair<int,int> data  = std::make_pair(1,1)

		  return data;
	  }

	  std::vector<std::pair<int,int>> pathToNearestUnknown() {
	    std::vector<std::pair<int,int>> path;
		  //TODO actually do an astar

		  return path
	  }
}
