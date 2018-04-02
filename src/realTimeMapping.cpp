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
    typedef PM::Matches Matches;

    // not used ?
    typedef typename Nabo::NearestNeighbourSearch<float> NNS;
    typedef typename NNS::SearchType NNSearchType;

public:
    gridMapping(ros::NodeHandle &n);
    ~gridMapping();
    ros::NodeHandle& n;

    ros::Publisher gridPublisher;
    ros::Publisher mapCloudPublisher;

    ros::Subscriber velodyneSubscriber;

    double size0;
    double size1;
    double resolution;
    double robotHeight;
    string cloudFilterName;
    double velodyneHeight;
    string robotFrame;
    string globalFrame;

    PM::DataPointsFilters cloudFilters;

    // first time?
    bool firstTime;

    double matchThreshold;

    tf::TransformListener tfListener;

    PM::TransformationParameters TrobotToGlobal;
    PM::TransformationParameters TrobotLastToGlobal;
    PM::TransformationParameters TrobotLastToRobot;
    PM::TransformationParameters TrobotToRobotLast;


    DP localMapCloud;
    shared_ptr<NNS> localMapNNS;

    unique_ptr<PM::Transformation> transformation;

    void cumulation(const sensor_msgs::PointCloud2& cloudMsgIn);

    void gridMapper(DP cloudIn);

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
    matchThreshold(getParam<double>("matchThreshold", 0)),
    cloudFilterName(getParam<string>("cloudFilterName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    velodyneHeight(getParam<double>("velodyneHeight", 0)),
    robotFrame(getParam<string>("robotFrame", ".")),
    globalFrame(getParam<string>("globalFrame", "."))
{
    firstTime = true;

    // initilization of filters
    ifstream filterStr(cloudFilterName);
    cloudFilters = PM::DataPointsFilters(filterStr);

    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    mapCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("map_cloud", 2, true);

    velodyneSubscriber = n.subscribe("velodyne_points", 10, &gridMapping::cumulation, this);
}

void gridMapping::cumulation(const sensor_msgs::PointCloud2& cloudMsgIn)
{
    DP velodyneCloud = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloudMsgIn);

    // listen to the real time trandformation form robot to world
    try
    {
        TrobotToGlobal = PointMatcher_ros::eigenMatrixToDim<float>(
                PointMatcher_ros::transformListenerToEigenMatrix<float>(
                tfListener,
                globalFrame,
                robotFrame,
                cloudMsgIn.header.stamp
            ), 4);
    }
    catch (exception& e)
    {
        cout << e.what() << endl;
        return;
    }

    // publish
    double t0 = ros::Time::now().toSec();

    // if needs initiliazation
    if(firstTime)
    {
        firstTime = false;
        // copy the velodyne to localMapCloud
        localMapCloud = velodyneCloud;
        // transformation initilization
        TrobotLastToGlobal = TrobotToGlobal;   // simple, one word
        return;
    }
    // Sth. wrong? for transformation
    // right now
    TrobotLastToRobot = TrobotToGlobal.inverse() * TrobotLastToGlobal;
    transformation->correctParameters(TrobotLastToRobot);
    if(transformation->checkParameters(TrobotLastToRobot))
    {
        localMapCloud = transformation->compute(localMapCloud, TrobotLastToRobot);
    }

    // accumulation of points, using kd match
    PM::Matches matches_velodyne(
        Matches::Dists(1, velodyneCloud.features.cols()),
        Matches::Ids(1, velodyneCloud.features.cols())
    );

    // update the kd-tree of local map cloud
    localMapNNS.reset(NNS::create(localMapCloud.features, localMapCloud.features.rows() - 1, NNS::KDTREE_LINEAR_HEAP, NNS::TOUCH_STATISTICS));
    // search
    localMapNNS->knn(velodyneCloud.features, matches_velodyne.ids, matches_velodyne.dists, 1, 0);

    // search for the update
    double dist;
    vector<int> addIndexVector;
    for(int p=0; p<velodyneCloud.features.cols(); p++)
    {
        dist = sqrt(matches_velodyne.dists(0, p));
        if(dist>matchThreshold)
            addIndexVector.push_back(p);
    }

    // rip the cloud according to the matching distance of velodyne cloud
    // faster than concatenate the velodyne directly
    DP concatenateCloud = velodyneCloud.createSimilarEmpty();
    int count = 0;
    for(int v=0; v<addIndexVector.size(); v++)
    {
        concatenateCloud.setColFrom(count, velodyneCloud, addIndexVector.at(v));
        count++;
    }
    concatenateCloud.conservativeResize(count);

    // update the localMapCloud
    localMapCloud.concatenate(concatenateCloud);
    // filter and add des! vital!
    cloudFilters.apply(localMapCloud);

    mapCloudPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(localMapCloud, robotFrame, cloudMsgIn.header.stamp));

    // one by one pose
    TrobotLastToGlobal = TrobotToGlobal;

    double t1 = ros::Time::now().toSec();
    cout<<"Cumulation Time:     "<<t1-t0<<endl;

    // next for elevation mapping
    this->gridMapper(localMapCloud);

    double t2 = ros::Time::now().toSec();
    cout<<"Gridding Time:       "<<t2-t1<<endl;

}

void gridMapping::gridMapper(DP cloudIn)
{

    // create the grid map
    // robot centric
    grid_map::GridMap localGridMap({"elevation", "normal_x", "normal_y", "normal_z"});
    localGridMap.setFrameId(robotFrame);
    localGridMap.setGeometry(Length(size0, size1), resolution, Position(0, 0));
    localGridMap.add("elevation", -velodyneHeight);
    localGridMap.add("update", 0);

    // transfer the 3D point cloud to the grid map of elevation / normal_z
    for(int p=0; p<cloudIn.features.cols(); p++)
    {
        Index index;
        Position position(cloudIn.features(0, p), cloudIn.features(1, p));
        if (!localGridMap.getIndex(position, index) || cloudIn.features(2, p)>robotHeight)
            continue;   // Skip this point if it does not lie within the elevation map and above the robot Height

        if(cloudIn.features(2, p) < -velodyneHeight)
            cloudIn.features(2, p) = -velodyneHeight;

        if (!localGridMap.isValid(index))
        {
            // groove or hill

            // need update?
            if(localGridMap.at("update", index) == 0) // need
            {
                localGridMap.at("elevation", index) = cloudIn.features(2, p); //update
                localGridMap.at("update", index) = 1;
            }
            else
            {
                if(cloudIn.features(2, p) > localGridMap.at("elevation", index)) // a higher value
                    localGridMap.at("elevation", index) = cloudIn.features(2, p); //update
            }

        }

    }


    // Publish grid map & cloud
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    gridPublisher.publish(message);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmapping");

  ros::NodeHandle n;

  gridMapping gridmapping(n);

  ros::spin();

  return 0;
}
