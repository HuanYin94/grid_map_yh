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
    ros::Publisher velodynePublisher;
    ros::Publisher mapCloudPublisher;

    double size0;
    double size1;
    double resolution;
    double robotHeight;
    string loadVelodyneDirName;
    string loadPoseName;
    string cloudFilterName;

    PM::DataPointsFilters cloudFilters;

    double matchThreshold;

    vector<vector<double>> robotPoses;
    tf::TransformBroadcaster tfBroadcaster;

    PM::TransformationParameters TrobotToGlobal;
    PM::TransformationParameters TrobotLastToGlobal;
    PM::TransformationParameters TrobotLastToRobot;
    PM::TransformationParameters TrobotToRobotLast;


    DP localMapCloud;
    shared_ptr<NNS> localMapNNS;

    unique_ptr<PM::Transformation> transformation;

    void cumulation(int index);

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
    loadVelodyneDirName(getParam<string>("loadVelodyneDirName", ".")),
    loadPoseName(getParam<string>("loadPoseName", ".")),
    cloudFilterName(getParam<string>("cloudFilterName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation"))
{
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    velodynePublisher = n.advertise<sensor_msgs::PointCloud2>("velodyne_cloud", 2, true);
    mapCloudPublisher = n.advertise<sensor_msgs::PointCloud2>("map_cloud", 2, true);
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    // initilization of filters
    ifstream filterStr(cloudFilterName);
    cloudFilters = PM::DataPointsFilters(filterStr);

    // read initial transformation
    int x, y;
    double temp;
    vector<double> test;
    ifstream in(loadPoseName);
    if (!in) {
        cout << "Cannot open file.\n";
    }
    for (y = 0; y < 9999999; y++) {
        test.clear();
    for (x = 0; x < 16; x++) {
      in >> temp;
      test.push_back(temp);
    }
      robotPoses.push_back(test);
    }
    in.close();

    for(int index=0; index<3000; index++)
    {
        this->cumulation(index);
    }

}

void gridMapping::cumulation(int index)
{

    // loading velodyne
    stringstream ss;
    ss<<index;
    string str;
    ss>>str;
    string veloName = loadVelodyneDirName + str + ".vtk";
    {
        cout<<"-----------------------------------------------------------------------"<<endl;
        cout<<veloName<<endl;
    }
    DP velodyneCloud = DP::load(veloName);


    // load poses
    // Transformation's transformation
    TrobotToGlobal = PM::TransformationParameters::Identity(4, 4);

    TrobotToGlobal(0,0)=robotPoses[index][0];TrobotToGlobal(0,1)=robotPoses[index][1];TrobotToGlobal(0,2)=robotPoses[index][2];TrobotToGlobal(0,3)=robotPoses[index][3];
    TrobotToGlobal(1,0)=robotPoses[index][4];TrobotToGlobal(1,1)=robotPoses[index][5];TrobotToGlobal(1,2)=robotPoses[index][6];TrobotToGlobal(1,3)=robotPoses[index][7];
    TrobotToGlobal(2,0)=robotPoses[index][8];TrobotToGlobal(2,1)=robotPoses[index][9];TrobotToGlobal(2,2)=robotPoses[index][10];TrobotToGlobal(2,3)=robotPoses[index][11];
    TrobotToGlobal(3,0)=robotPoses[index][12];TrobotToGlobal(3,1)=robotPoses[index][13];TrobotToGlobal(3,2)=robotPoses[index][14];TrobotToGlobal(3,3)=robotPoses[index][15];

    // publish
    tfBroadcaster.sendTransform(PointMatcher_ros::eigenMatrixToStampedTransform<float>(TrobotToGlobal, "global", "robot", ros::Time::now()));
    velodynePublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(velodyneCloud, "robot", ros::Time::now()));

    double t0 = ros::Time::now().toSec();

    // if needs initiliazation
    if(index==0)
    {
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

    mapCloudPublisher.publish(PointMatcher_ros::pointMatcherCloudToRosMsg<float>(localMapCloud, "robot", ros::Time::now()));

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
    // in libpointmatcher
    int rowNormalX = cloudIn.getDescriptorStartingRow("normal_x");
    int rowNormalY = cloudIn.getDescriptorStartingRow("normal_y");
    int rowNormalZ = cloudIn.getDescriptorStartingRow("normal_z");

    // create the grid map
    // robot centric
    grid_map::GridMap localGridMap({"elevation", "normal_x", "normal_y", "normal_z"});
    localGridMap.setFrameId("robot");
    localGridMap.setGeometry(Length(size0, size1), resolution, Position(0, 0));

//    localGridMap.add("traversability", Matrix::Zero(localGridMap.getSize()(0), localGridMap.getSize()(1)));

    // transfer the 3D point cloud to the grid map of elevation / normal_z
    for(int p=0; p<cloudIn.features.cols(); p++)
    {
        Index index;
        Position position(cloudIn.features(0, p), cloudIn.features(1, p));
        if (!localGridMap.getIndex(position, index) || cloudIn.features(2, p)>robotHeight)
            continue;   // Skip this point if it does not lie within the elevation map.

        if (!localGridMap.isValid(index))
        {
            if(localGridMap.at("elevation", index) > checkValue)  // check if has points already
            {
                // upper bound
                if(cloudIn.features(2, p) > localGridMap.at("elevation", index))
                {
                    localGridMap.at("elevation", index) = cloudIn.features(2, p); //elevation
                    localGridMap.at("normal_x", index) = cloudIn.descriptors(rowNormalX, p);
                    localGridMap.at("normal_y", index) = cloudIn.descriptors(rowNormalY, p);
                    localGridMap.at("normal_z", index) = cloudIn.descriptors(rowNormalZ, p);
                }
            }
            else            // init this voxel
            {
                localGridMap.at("elevation", index) = cloudIn.features(2, p);
                localGridMap.at("normal_x", index) = cloudIn.descriptors(rowNormalX, p);
                localGridMap.at("normal_y", index) = cloudIn.descriptors(rowNormalY, p);
                localGridMap.at("normal_z", index) = cloudIn.descriptors(rowNormalZ, p);
            }
        }
    }

/* Tailor Work
 *  Haha, trick work
*/
    /*
    for (GridMapIterator it(localGridMap); !it.isPastEnd(); ++it)
    {
        if(localGridMap.at("elevation", *it) > checkValue)
            continue;

        // find its neighbors
        Position position, position;
        localGridMap.getPosition(*it, position);


    }
    */

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

  return 0;
}
