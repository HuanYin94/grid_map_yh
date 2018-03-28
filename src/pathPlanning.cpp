#include "pathPlanning.hpp"

bool pP::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

pP::Vec2i operator + (const pP::Vec2i& left_, const pP::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

pP::uint pP::Node::getScore()
{
    return G + H;
}

pP::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

pP::AStar::~AStar()
{}

pP::AStar::AStar(ros::NodeHandle& n):
    n(n),
    size0(getParam<double>("size0", 0)),
    resolution(getParam<double>("resolution", 0)),
    enableCross(getParam<bool>("enableCross", 0))
{
    mapSize = size0/resolution;
    directionCount = enableCross ? 8 : 4;
    source = {mapSize/2, mapSize/2};
    target = {mapSize, mapSize/2};

    occuMapSub = n.subscribe("occuMap", 10, &AStar::pathPlanner, this);
    pathPointsPub = n.advertise<nav_msgs::Path>("pathPoints", 2, true);

}

void pP::AStar::pathPlanner(const nav_msgs::OccupancyGrid& occuMapIn)
{
    cout<<"-----------------------------------"<<endl;

    //haha, don't forget
    occus.clear();

    // build the map: occus & frees
    int count = 0;
    for(int m=0; m<mapSize; m++)
    {
        for(int n=0; n<mapSize; n++)
        {
            int a = occuMapIn.data.at(count);
            count++;
//            cout<<a<<"  ";

            //turns to 2D coordinate in a plane
            Vec2i coord = {mapSize-m, n};

            if(a != 100) // full traversability check
                occus.push_back(coord);

        }
//        cout<<"------------------"<<endl;
    }

//    cout<<"occuNum: "<<occus.size()<<endl;

    //start to find the path
    // robot start at the centric
    Node *currentNode  = nullptr;
    nextNodes.insert(new Node(source));

    while(!nextNodes.empty())
    {
        currentNode = *nextNodes.begin();
        for (auto node : nextNodes) {
            if (node->getScore() <= currentNode->getScore()) {
                currentNode = node;
            }
        }

        if (currentNode->coordinates == target) {
            break;
        }

        pathNodes.insert(currentNode);
        nextNodes.erase(std::find(nextNodes.begin(), nextNodes.end(), currentNode));

        for (uint i = 0; i < directionCount; ++i)
        {
            Vec2i newCoord(currentNode->coordinates + direction[i]);
            if (isCollision(newCoord) ||
                findNodeOnList(pathNodes, newCoord))
            {
                continue;
            }

            uint totalCost = currentNode->G + ((i < 4) ? 2 : 4);

            Node *successor = findNodeOnList(nextNodes, newCoord);
            if (successor == nullptr)
            {
                successor = new Node(newCoord, currentNode);
                successor->G = totalCost;
                successor->H = euclidean(successor->coordinates, target);
                nextNodes.insert(successor);
            }
            else if (totalCost < successor->G)
            {
                successor->parent = currentNode;
                successor->G = totalCost;
            }
        }

    }

    std::vector<Vec2i> path;
    while (currentNode != nullptr)
    {
        path.push_back(currentNode->coordinates);
        currentNode = currentNode->parent;
    }

    this->drawPath(path);

    this->releaseNodes(nextNodes);
    this->releaseNodes(pathNodes);
//    nextNodes.clear();
//    pathNodes.clear();

}

bool pP::AStar::isCollision(Vec2i coord)
{
    if( coord.x >= mapSize || coord.x < 0 ||
        coord.y >= mapSize || coord.y < 0 ||
        std::find(occus.begin(), occus.end(), coord) != occus.end())
        return true;
    else
        return false;
}

pP::Node* pP::AStar::findNodeOnList(std::set<Node*>& nodes, Vec2i coord)
{
    for (auto node : nodes) {
        if (node->coordinates == coord) {
            return node;
        }
    }
    return nullptr;
}

pP::Vec2i pP::AStar::getDelta(Vec2i source, Vec2i target)
{
    return{ abs(source.x - target.x),  abs(source.y - target.y) };
}

pP::uint pP::AStar::euclidean(Vec2i source, Vec2i target)
{
    auto delta = std::move(getDelta(source, target));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

void pP::AStar::releaseNodes(std::set<Node*>& nodes)
{
    for (auto it = nodes.begin(); it != nodes.end();)
    {
        delete *it;
        it = nodes.erase(it);
    }
}

void pP::AStar::drawPath(std::vector<Vec2i> path)
{
    nav_msgs::Path pathShow;
    pathShow.header.frame_id = "robot";
    pathShow.header.stamp = ros::Time::now();

    // Coord-transform, resolution added
    // robot coordinate & my coordinate
    for(int i=0; i<path.size(); i++)
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = pathShow.header.frame_id;
        pose.header.stamp = pathShow.header.stamp;
        pose.pose.position.x = (path.at(i).y - mapSize/2)*resolution;
        pose.pose.position.y = (mapSize/2 - path.at(i).x)*resolution;
        pose.pose.position.z = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);  // always 0;
        pathShow.poses.push_back(pose);

//        cout<<pose.pose.position.x<<"   "<<pose.pose.position.y<<endl;
    }
//    cout<<"pointsNum:   "<<pathShow.poses.size()<<endl;
    pathPointsPub.publish(pathShow);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Astar");

  ros::NodeHandle n;

  pP::AStar Astar(n);

  ros::spin();

  return 0;
}
