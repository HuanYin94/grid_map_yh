#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <fstream>
#include <set>
#include <functional>

#include "tf/transform_broadcaster.h"
#include "pointmatcher_ros/get_params_from_server.h"

#define checkValue -999

/*
 *   update -> gridMapper -> travelSearch (another node)-> # TODO AStar (another node)
 *
*/
namespace pP
{

    using namespace grid_map;
    using namespace std;
    using uint = unsigned int;

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

        Vec2i source;
        Vec2i target;
        vector<Vec2i> direction = {
            { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
            { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
        };
        uint directionCount;
        bool enableCross;

        std::set<Node*> pathNodes;
        std::set<Node*> nextNodes;

        double size0;
        double resolution;
        int gridSize;

        void pathPlanner(const nav_msgs::OccupancyGrid& occuMapIn);
        uint euclidean(Vec2i source, Vec2i target);
        bool isCollision(Vec2i coord);
        Node* findNodeOnList(std::set<Node*>& nodes, Vec2i coord);
        Vec2i getDelta(Vec2i source, Vec2i target);
        void releaseNodes(std::set<Node*>& nodes);

    private:


    };

}
