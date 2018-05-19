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
#include "tf/transform_listener.h"

#define checkValue -999


/*
 *   cumulation -> gridMapper -> # TODO travelSearch (another node)-> # TODO pathPlanning (another node)
 *
*/


using namespace grid_map;
using namespace std;
using namespace PointMatcherSupport;

class gridMapping
{
    typedef PointMatcher<float> PM;
    typedef PM::DataPoints DP;

public:
    gridMapping(ros::NodeHandle &n);
    ~gridMapping();
    ros::NodeHandle& n;

    ros::Publisher gridPublisher;
    ros::Subscriber velodyneSubscriber;

    double size0;
    double size1;
    double resolution;
    double robotHeight;
    string cloudFilterName;
    double velodyneHeight;
    string robotFrame;
    string globalFrame;

    double fillRadius;
    double rangeRadius;
    double groundTolarance;

    unique_ptr<PM::Transformation> transformation;

    void gridMapper(const sensor_msgs::PointCloud2& cloudMsgIn);

    bool isInRange(Eigen::Vector2d center);

private:


};

gridMapping::~gridMapping()
{}

gridMapping::gridMapping(ros::NodeHandle& n):
    n(n),
    size0(getParam<double>("size0", 0)),
    size1(getParam<double>("size1", 0)),
    resolution(getParam<double>("resolution", 0)),
    robotHeight(getParam<double>("robotHeight", 0)),
    cloudFilterName(getParam<string>("cloudFilterName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    velodyneHeight(getParam<double>("velodyneHeight", 0)),
    robotFrame(getParam<string>("robotFrame", ".")),
    fillRadius(getParam<double>("fillRadius", 0)),
    rangeRadius(getParam<double>("rangeRadius", 0)),
    groundTolarance(getParam<double>("groundTolarance", 0))
{

    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    velodyneSubscriber = n.subscribe("velodyne_points", 10, &gridMapping::gridMapper, this);
}

void gridMapping::gridMapper(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    DP velodyneCloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn);

    // create the grid map
    // robot centric
    grid_map::GridMap localGridMap({"elevation", "normal_x", "normal_y", "normal_z"});
    localGridMap.setFrameId(robotFrame);
    localGridMap.setGeometry(Length(size0, size1), resolution, Position(0, 0));
    localGridMap.add("elevation", -velodyneHeight);
    localGridMap.add("update", 0);

    // transfer the 3D point cloud to the grid map of elevation / normal_z
    for(int p=0; p<velodyneCloud.features.cols(); p++)
    {
        Index index;
        Position position(velodyneCloud.features(0, p), velodyneCloud.features(1, p));
        if (!localGridMap.getIndex(position, index) || velodyneCloud.features(2, p)>robotHeight)
            continue;   // Skip this point if it does not lie within the elevation map and above the robot Height

        if(velodyneCloud.features(2, p) < -velodyneHeight)
            velodyneCloud.features(2, p) = -velodyneHeight;

        if (!localGridMap.isValid(index))
        {
            // groove or hill

            // need update?
            if(localGridMap.at("update", index) == 0) // need
            {
                localGridMap.at("elevation", index) = velodyneCloud.features(2, p); //update
                localGridMap.at("update", index) = 1;
            }
            else
            {
                if(velodyneCloud.features(2, p) > localGridMap.at("elevation", index)) // a higher value
                    localGridMap.at("elevation", index) = velodyneCloud.features(2, p); //update
            }

        }
    }

    // Those that without update,average filter them
    // travel the map
    for(GridMapIterator iterator(localGridMap); !iterator.isPastEnd(); ++iterator)
    {

        // measurement, continue
        if(localGridMap.at("update", *iterator) == 1)
            continue;

        Eigen::Vector2d center;
        localGridMap.getPosition(*iterator, center);

        // do not operate the nearest ones, for the person following the robot!
        if(this->isInRange(center))
            continue;

        // initial
        vector<double> measureEle;

        for (CircleIterator submapIterator(localGridMap, center, fillRadius);
            !submapIterator.isPastEnd(); ++submapIterator)
        {
            if (localGridMap.at("update", *submapIterator) == 1)
                measureEle.push_back(localGridMap.at("elevation", *submapIterator));
        }

        // calculate the mean value fot those not updated!
        if(measureEle.size() > 0) //has measures?
        {
            double sum = std::accumulate(std::begin(measureEle), std::end(measureEle), 0.0);
            double mean =  sum / measureEle.size();
            // judge
            if(abs(mean-localGridMap.at("elevation", *iterator)) > this->groundTolarance)
                localGridMap.at("elevation", *iterator) = mean;
        }

    }


    // Publish grid map & cloud
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    gridPublisher.publish(message);

}

bool gridMapping::isInRange(Eigen::Vector2d center)
{
    double dis = pow(center(0)-0, 2) + pow(center(1)-0, 2);

    if(dis > this->rangeRadius)
        return false;
    else
        return true;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmapping");

  ros::NodeHandle n;

  gridMapping gridmapping(n);

  ros::spin();

  return 0;
}
