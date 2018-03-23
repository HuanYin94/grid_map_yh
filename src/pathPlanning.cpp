#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <limits>
#include <fstream>

#include "tf/transform_broadcaster.h"

#define checkValue -999


/*
 *   update -> gridMapper -> travelSearch (another node)-> # TODO AStar (another node)
 *
*/

using namespace grid_map;
using namespace std;

struct Vec2i
{
    int x, y;
    bool operator == (const Vec2i& coordinates_);
};

struct Node
{
    uint G, H;
    Vec2i coordinates;
    Node *parent;

    Node(Vec2i coord_, Node *parent_ = nullptr);
    uint getScore();
};

class AStar
{

public:
    AStar(ros::NodeHandle &n);
    ~AStar();
    ros::NodeHandle& n;

    ros::Subscriber occuMapSub;
    nav_msgs::OccupancyGrid occuMap;

    std::vector<Vec2i> occus;
    std::vector<Vec2i> frees;

    void pathPlanner(const nav_msgs::OccupancyGrid& occuMapIn);
    void heuristic(Vec2i source, Vec2i target);

private:


};

AStar::~AStar()
{}

AStar::AStar(ros::NodeHandle& n):
    n(n)
{
    occuMapSub = n.subscribe("occuMap", 10, &AStar::pathPlanner, this);

}

void AStar::pathPlanner(const nav_msgs::OccupancyGrid& occuMapIn)
{
    cout<<occuMapIn.data.size()<<endl;
    for(int i=0; i<occuMapIn.data.size(); i++)
    {


    }

    int count = 0;
    for(int m=0; m<38; m++)
    {
        for(int n=0; n<38; n++)
        {
            cout<<occuMapIn.data.at(count);count++;
        }
        cout<<""<<endl;
    }

}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "Astar");

  ros::NodeHandle n;

  AStar Astar(n);

  ros::spin();

  return 0;
}
