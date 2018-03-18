#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

#include "pointmatcher/PointMatcher.h"
#include "pointmatcher/Timer.h"
#include "pointmatcher_ros/point_cloud.h"
#include "pointmatcher_ros/transform.h"
#include "nabo/nabo.h"
#include "eigen_conversions/eigen_msg.h"
#include "pointmatcher_ros/get_params_from_server.h"

using namespace grid_map;
using namespace std;
using namespace PointMatcherSupport;

class Traversability
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

    // not used ?
    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

public:
    Traversability(ros::NodeHandle &n);
    ~Traversability();
    ros::NodeHandle& n;


    void process();

private:


};

Traversability::~Traversability()
{}

Traversability::Traversability(ros::NodeHandle& n):
    n(n)
{

    this->process();
}

void Traversability::process()
{

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability");

  ros::NodeHandle n;

  Traversability traversability(n);

  return 0;
}
