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

    ros::Subscriber gridMapSub;
    ros::Publisher gridPublisher;

    double slopeCritical;

    void check(const grid_map_msgs::GridMap& gridMapIn);

private:


};

Traversability::~Traversability()
{}

Traversability::Traversability(ros::NodeHandle& n):
    n(n),
    slopeCritical(getParam<double>("slopeCritical", 0))
{
    gridMapSub = n.subscribe("grid_map", 10, &Traversability::check, this);
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map_travelChecked", 1, true);

}

void Traversability::check(const grid_map_msgs::GridMap& gridMapIn)
{
    cout<<"----------------------------"<<endl;

    // recieve the message
    grid_map::GridMap localGridMap;
    GridMapRosConverter::fromMessage(gridMapIn, localGridMap);

    // slope check
    localGridMap.add("traversability_slope", Matrix::Zero(localGridMap.getSize()(0), localGridMap.getSize()(1)));
    double slope, slopeMax = 0.0;
    for (GridMapIterator it(localGridMap);!it.isPastEnd(); ++it)
    {
        // Check if there is a surface normal (empty cell).
        if (!localGridMap.isValid(*it, "normal_z"))
            continue;
        // Compute slope from surface normal z
        slope = acos(localGridMap.at("normal_z", *it));

        if (slope < slopeCritical) {
          localGridMap.at("traversability_slope", *it) = 1.0 - slope / slopeCritical;
        }
        else {
          localGridMap.at("traversability_slope", *it) = 0.0;
        }

        if (slope > slopeMax) slopeMax = slope; // slopeMax can be used for cout
    }

    // Publish grid map & cloud
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    gridPublisher.publish(message);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability");

  ros::NodeHandle n;

  Traversability traversability(n);

  ros::spin();

  return 0;
}
