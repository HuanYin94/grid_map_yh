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

#define checkValue -999

using namespace grid_map;
using namespace std;
using namespace PointMatcherSupport;

class gridMapping
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;
    typedef PM::Matches Matches;

    // not used ?
    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

public:
    gridMapping(ros::NodeHandle &n);
    ~gridMapping();
    ros::NodeHandle& n;

    DP mapCloud;

    ros::Publisher gridPublisher;
    ros::Publisher cloudPublisher;
    double size0;
    double size1;
    double resolution;
    double robotX;
    double robotY;
    double robotHeight;
    string loadMapName;

    void process();

private:


};

gridMapping::~gridMapping()
{}

gridMapping::gridMapping(ros::NodeHandle& n):
    n(n),
    size0(getParam<double>("size0", 0)),
    size1(getParam<double>("size1", 0)),
    resolution(getParam<double>("resolution", 0)),
    robotX(getParam<double>("robotX", 0)),
    robotY(getParam<double>("robotY", 0)),
    robotHeight(getParam<double>("robotHeight", 0)),
    loadMapName(getParam<string>("loadMapName", "."))
{
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    cloudPublisher = n.advertise<sensor_msgs::PointCloud2>("map_cloud", 2, true);

    this->process();
}

void gridMapping::process()
{
    mapCloud = DP::load(loadMapName);

    // create the grid map
    grid_map::GridMap localGridMap({"elevation"});
    localGridMap.setFrameId("map");
    localGridMap.setGeometry(Length(size0, size1), resolution, Position(robotX, robotY));

    localGridMap.add("traversability", Matrix::Zero(localGridMap.getSize()(0), localGridMap.getSize()(1)));

    ros::Rate rate(3.0);
    while (ros::ok())
    {

        // find the points in the grid
        for(int p=0; p<mapCloud.features.cols(); p++)
        {
            Index index;
            Position position(mapCloud.features(0, p), mapCloud.features(1, p));
            if (!localGridMap.getIndex(position, index) || mapCloud.features(2, p)>robotHeight)
                continue;   // Skip this point if it does not lie within the elevation map.

            if (!localGridMap.isValid(index))
            {
                if(localGridMap.at("elevation", index) > checkValue)
                {
                    // upper bound
                    if(mapCloud.features(2, p) > localGridMap.at("elevation", index))
                        localGridMap.at("elevation", index) = mapCloud.features(2, p);                }
                else
                    localGridMap.at("elevation", index) = mapCloud.features(2, p);
            }
        }

        // check the traversability area
//        for()
//        {

//        }

        // Publish grid map & cloud
        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(localGridMap, message);
        gridPublisher.publish(message);
        cloudPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(mapCloud, "map", ros::Time(0)));

        rate.sleep();
    }

}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");

  ros::NodeHandle n;

  gridMapping gridmapping(n);

  return 0;
}
