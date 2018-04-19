#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <fstream>
#include <numeric>

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
 *   update the laser map frame by frame
 *
*/


using namespace grid_map;
using namespace std;

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

    double size0;
    double size1;
    double resolution;
    double robotHeight;
    double velodyneHeight;
    string loadVelodyneDirName;
    string loadPoseName;
    double fillRadius;
    double rangeRadius;

    vector<vector<double>> robotPoses;
    tf::TransformBroadcaster tfBroadcaster;

    int startIndex;
    int readNum;

    PM::TransformationParameters TrobotToGlobal;

    DP velodyneCloud;

    unique_ptr<PM::Transformation> transformation;

    void update(int index);

    void gridMapper(DP cloudIn);

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
    velodyneHeight(getParam<double>("velodyneHeight", 0)),
    loadVelodyneDirName(getParam<string>("loadVelodyneDirName", ".")),
    loadPoseName(getParam<string>("loadPoseName", ".")),
    transformation(PM::get().REG(Transformation).create("RigidTransformation")),
    readNum(getParam<int>("readNum", 0)),
    startIndex(getParam<int>("startIndex", 0)),
    fillRadius(getParam<double>("fillRadius", 0)),
    rangeRadius(getParam<double>("rangeRadius", 0))
{
    gridPublisher = n.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    velodynePublisher = n.advertise<sensor_msgs::PointCloud2>("velodyne_cloud", 2, true);

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

    // one by one frame
    for(int index=startIndex; index<startIndex+readNum; index++)
    {
        this->update(index);
    }

}

void gridMapping::update(int index)
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
    velodyneCloud = DP::load(veloName);

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

    this->gridMapper(velodyneCloud);

}

void gridMapping::gridMapper(DP cloudIn)
{

    // create the grid map
    // robot centric
    grid_map::GridMap localGridMap({"elevation", "normal_x", "normal_y", "normal_z"});
    localGridMap.setFrameId("robot");
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

    // Those that without update,average filter them
    // travel the map
    for(GridMapIterator iterator(localGridMap); !iterator.isPastEnd(); ++iterator)
    {

        // measurement, continue
        if(localGridMap.at("update", *iterator) == 1)
            continue;

        Eigen::Vector2d center;
        localGridMap.getPosition(*iterator, center);

        // do not operate he nearest ones, for the person
        if()

        // initial
        vector<double> measureEle;

        for (CircleIterator submapIterator(localGridMap, center, fillRadius);
            !submapIterator.isPastEnd(); ++submapIterator)
        {
            if (localGridMap.at("update", *submapIterator) == 1)
                measureEle.push_back(localGridMap.at("elevation", *submapIterator));
        }

        if(measureEle.size() > 0) //has measures?
        {
            double sum = accumulate(measureEle.begin(),measureEle.end(),0);
            localGridMap.at("elevation", *iterator) = sum / measureEle.size();
            cout<<localGridMap.at("elevation", *iterator)<<endl;
        }

    }


    // Publish grid map & cloud
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(localGridMap, message);
    gridPublisher.publish(message);

}


bool gridMapping::isInRange(Eigen::Vector2d center)
{

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gridmapping");

  ros::NodeHandle n;

  gridMapping gridmapping(n);

  return 0;
}
