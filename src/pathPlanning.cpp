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
    gridSize = size0/resolution;
    directionCount = enableCross ? 8 : 4;
    source = {gridSize/2, gridSize/2};
    target = {gridSize/2, gridSize};

    cout<<"directionCount:  "<<directionCount<<endl;

    occuMapSub = n.subscribe("occuMap", 10, &AStar::pathPlanner, this);

}

void pP::AStar::pathPlanner(const nav_msgs::OccupancyGrid& occuMapIn)
{
    // build the map: occus & frees
    int count = 0;
    for(int m=0; m<gridSize; m++)
    {
        for(int n=0; n<gridSize; n++)
        {
            int a = occuMapIn.data.at(count);
            count++;

            //turns to 2D coordinate in a plane
            Vec2i coord = {gridSize-m, gridSize-n};

            if(a > 0)
                occus.push_back(coord);

        }
    }

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

            uint totalCost = currentNode->G + ((i < 4) ? 10 : 14);

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

    std::vector<Vec2i> pathNodes;
    while (currentNode != nullptr)
    {
        pathNodes.push_back(currentNode->coordinates);
        currentNode = currentNode->parent;
    }

    //cout
    for(int i=0; i<pathNodes.size(); i++)
    {
        cout<<i<<"  "<<pathNodes.at(i).x<<"    "<<pathNodes.at(i).y<<endl;
    }

}

bool pP::AStar::isCollision(Vec2i coord)
{
    if( abs(coord.x) >= gridSize ||
        abs(coord.y) >= gridSize ||
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


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Astar");

  ros::NodeHandle n;

  pP::AStar Astar(n);

  ros::spin();

  return 0;
}
