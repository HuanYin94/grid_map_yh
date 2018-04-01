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
    ros::Publisher occuMapPublisher;

    nav_msgs::OccupancyGrid occuMap;

    double slopeCritical;
    double stepCritical;
    double stepRadiusFirst;
    double stepRadiusSecond;
    int cellsCritical;

    void check(const grid_map_msgs::GridMap& gridMapIn);

private:


};

Traversability::~Traversability()
{}

Traversability::Traversability(ros::NodeHandle& n):
    n(n),
    slopeCritical(getParam<double>("slopeCritical", 0)),
    stepCritical(getParam<double>("stepCritical", 0)),
    stepRadiusFirst(getParam<double>("stepRadiusFirst", 0)),
    stepRadiusSecond(getParam<double>("stepRadiusSecond", 0)),
    cellsCritical(getParam<int>("cellsCritical", 0))
{
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map_travelChecked", 1, true);
    occuMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("occuMap", 1, true);
    gridMapSub = n.subscribe("grid_map", 10, &Traversability::check, this);
}

void Traversability::check(const grid_map_msgs::GridMap& gridMapIn)
{
    cout<<"----------------------------"<<endl;

    // recieve the message
    grid_map::GridMap localGridMap;
    GridMapRosConverter::fromMessage(gridMapIn, localGridMap);

    /// step check
    /// from ethz_asl step_filter of traversability estimation
    localGridMap.add("traversability_step");
    localGridMap.add("step_height");
    double height, step;

    // First iteration through the elevation map.
    for (GridMapIterator iterator(localGridMap); !iterator.isPastEnd(); ++iterator)
    {
      if (!localGridMap.isValid(*iterator, "elevation"))
        continue;
      height = localGridMap.at("elevation", *iterator);
      double heightMax, heightMin;

      // Requested position (center) of circle in map.
      Eigen::Vector2d center;
      localGridMap.getPosition(*iterator, center);

      // Get the highest step in the circular window.
      bool init = false;
      for (CircleIterator submapIterator(localGridMap, center, stepRadiusFirst);
          !submapIterator.isPastEnd(); ++submapIterator)
      {
        if (!localGridMap.isValid(*submapIterator, "elevation"))
          continue;
        height = localGridMap.at("elevation", *submapIterator);
        // Init heightMax and heightMin
        if (!init)
        {
          heightMax = height;
          heightMin = height;
          init = true;
          continue;
        }
        if (height > heightMax)
          heightMax = height;
        if (height < heightMin)
          heightMin = height;
      }

      if (init)
        localGridMap.at("step_height", *iterator) = heightMax - heightMin;
    }

    // Second iteration through the elevation map.
    for (GridMapIterator iterator(localGridMap); !iterator.isPastEnd(); ++iterator)
    {
      int nCells = 0;
      double stepMax = 0.0;
      bool isValid = false;

      // Requested position (center) of circle in map.
      Eigen::Vector2d center;
      localGridMap.getPosition(*iterator, center);

      // Compute the step height.
      for (CircleIterator submapIterator(localGridMap, center, stepRadiusSecond);
          !submapIterator.isPastEnd(); ++submapIterator)
      {
        if (!localGridMap.isValid(*submapIterator, "step_height"))
          continue;
        isValid = true;
        if (localGridMap.at("step_height", *submapIterator) > stepMax)
        {
          stepMax = localGridMap.at("step_height", *submapIterator);
        }
        if (localGridMap.at("step_height", *submapIterator) > stepCritical)
          nCells++;
      }

      if (isValid)
      {
        step = std::min(stepMax,
                        (double) nCells / (double) cellsCritical * stepMax);
        if (step < stepCritical)
        {
          // fully = 1
          localGridMap.at("traversability_step", *iterator) = 1.0 - step / stepCritical;
        }
        else
        {
          // cannot traverse
          localGridMap.at("traversability_step", *iterator) = 0.0;
        }
      }
    }



    // Publish grid map & cloud
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    gridPublisher.publish(message);

    // convert to 2D-costmap and publish
    GridMapRosConverter::toOccupancyGrid(localGridMap, "traversability_step", 0.0, 1.0, occuMap);
    occuMapPublisher.publish(occuMap);

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversability");

  ros::NodeHandle n;

  Traversability traversability(n);

  ros::spin();

  return 0;
}
