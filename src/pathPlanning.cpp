#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <fstream>


#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "nabo/nabo.h"
#include "eigen_conversions/eigen_msg.h"
#include "pointmatcher_ros/get_params_from_server.h"

#include "tf/transform_broadcaster.h"

#define checkValue -999


/*
 *   update -> gridMapper -> # TODO travelSearch (another node)-> # TODO pathPlanning (another node)
 *
*/


using namespace grid_map;
using namespace std;
using namespace PointMatcherSupport;

class pathPlanning
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

    // not used ?
    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

public:
    pathPlanning(ros::NodeHandle &n);
    ~pathPlanning();
    ros::NodeHandle& n;

    void pathPlanner();

private:


};

pathPlanning::~pathPlanning()
{}

pathPlanning::pathPlanning(ros::NodeHandle& n):
    n(n)
{

}

void pathPlanning::pathPlanner()
{


}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathPlanning");

  ros::NodeHandle n;

  pathPlanning pathplanning(n);

  return 0;
}
