#include "nav2_rrtree_planner/rrt_planner.hpp"
#include <random>
#include <cmath>
#include <cstdlib>

namespace rrt_planner
{
    rrt_planner::rrt_planner()
    {
        srand(time(NULL));
    }

    void rrt_planner::setStartAndGoal(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal
    )
    {
        sizeX = costmap_->getSizeInCellsX();
        sizeY = costmap_->getSizeInCellsY();
        unsigned int s[2], d[2];
        costmap_->worldToMap(start.pose.position.x, start.pose.position.y, s[0], s[1]);
        costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, d[0], d[1]);
        treeRRT = new rrt_nodes(s[0], s[1]);
        goalRRT = new rrt_nodes(d[0], d[1]);
        nearestRRT = new rrt_nodes();

        nearestDistance = 50000;

        std::cout << "size: " << sizeX << " " << sizeY << std::endl;
        std::cout << "start: " << treeRRT->_xPos << " " << treeRRT->_yPos << std::endl;
        std::cout << "end: " << goalRRT->_xPos << " " << goalRRT->_yPos << std::endl;
    }

    coordsW rrt_planner::samplePoint()
    {
        coordsW temp;
        int coin = rand() % 100;
        if (coin < 10)
        {
            temp.x = goalRRT->_xPos;
            temp.y = goalRRT->_yPos;
        }
        else
        {
            temp.x = rand() % sizeX;
            temp.y = rand() % sizeY;
        }
        return temp;
    }

    float rrt_planner::distance(rrt_nodes* node, coordsW point)
    {
        coordsW temp;
        temp.x = node->_xPos - point.x;
        temp.y = node->_yPos - point.y;
        return sqrt(temp.x * temp.x + temp.y * temp.y);
    }

    coordsW rrt_planner::getUnitVector(rrt_nodes* start, coordsW end)
    {
        coordsW vector;
        vector.x = end.x - (double) start->_xPos;
        vector.y = end.y - (double) start->_yPos;
        
        double norm = sqrt(vector.x * vector.x + vector.y * vector.y);
        coordsW unitVector;
        unitVector.x = vector.x / norm;
        unitVector.y = vector.y / norm;

        return unitVector;
    }

    void rrt_planner::addChild(coordsW point)
    {
        if (point.x == goalRRT->_xPos && point.y == goalRRT->_yPos)
        {
            if (nearestRRT->_child.size() != 0)
            {
                std::cout << "Encoutered child\n";
                goalRRT->_parent = nearestRRT->_child.back();
                nearestRRT->_child.back()->_child.push_back(goalRRT);
            }
            else
            {
                std::cout << "Encoutered zero child\n";
                goalRRT->_parent = nearestRRT;
                nearestRRT->_child.push_back(goalRRT);
            }
            std::cout << nearestRRT->_child.size() << std::endl;
            
        }
        else
        {
            rrt_nodes* temp = new rrt_nodes(point.x, point.y);
            nearestRRT->_child.push_back(temp);
            temp->_parent = nearestRRT;
        }    
    }

    coordsW rrt_planner::extentToPoint(rrt_nodes* start, coordsW end)
    {
        coordsW offset;
        coordsW u_vector = getUnitVector(start, end);
        offset.x = branchLen * u_vector.x;
        offset.y = branchLen * u_vector.y;
        //std::cout << "offset: " << offset.x << " " << offset.y << std::endl;
        coordsW point;
        point.x = start->_xPos + round(offset.x);
        point.y = start->_yPos + round(offset.y);

        if (point.x >= sizeX)
        {
            point.x = sizeX - 1;
        }
        if (point.y >= sizeY)
        {
            point.y = sizeY - 1;
        }
        if (point.x < 0)
        {
            point.x = 1;
        }
        if (point.y < 0)
        {
            point.y = 1;
        }
        return point;
    }

    bool rrt_planner::checkObstacle(rrt_nodes* start, coordsW end)
    {
        coordsW unitVector = getUnitVector(start, end);
        coordsW tempPoint;
        for (int i = 0; i < (int) distance(start, end); i++)
        {
            tempPoint.x = round(start->_xPos + i * unitVector.x);
            tempPoint.y = round(start->_yPos + i * unitVector.y);

            int cost = costmap_->getCost((int) (sizeX * tempPoint.y + tempPoint.x));
            if (cost >= LETHAL_COST && cost <= OBS_COST)
            {
                return true;
            }
        }
        return false;
    }

    

    bool rrt_planner::checkGoal(coordsW point)
    {
        return (distance(goalRRT, point) <= branchLen);
    }


    void rrt_planner::findNearestNode(rrt_nodes *root, coordsW point)
    {
        if(!root)
        {
            return;
        }
        float dist = distance(root, point);
        if (dist <= nearestDistance)
        {
            nearestDistance = dist;
            nearestRRT = root;
        }
        
        for (int i = 0; i < root->_child.size(); i++)
        {
            findNearestNode(root->getChild(i), point);
        }

        if (nearestRRT->_xPos < 0)
        {
            std::cout << "OverflowNearest: " << root->_xPos << " " << root->_yPos << std::endl;
        }
    }

    void rrt_planner::resetNearestValues()
    {
        nearestDistance = 50000;
        nearestRRT = nullptr;
    }

    void rrt_planner::backtrack(rrt_nodes *goal)
    {
        if (goal->_xPos == treeRRT->_xPos && goal->_yPos == treeRRT->_yPos)
        {
            return;
        }
        coordsW current;
        current.x = goal->_xPos;
        current.y = goal->_yPos;
        waypoints.push_back(current);
        
        backtrack(goal->_parent);
    }

    void rrt_planner::generateRawPath()
    {
        std::cout << "Generating raw path" << std::endl;
        coordsW temp;
        temp.x = goalRRT->_xPos;
        temp.y = goalRRT->_yPos;
        if (distance(treeRRT, temp) <= branchLen)
        {
            nearestRRT = treeRRT;
            addChild(temp);
            return;
        }
        for (int i = 0; i < iterations; i++)
        {
            resetNearestValues();

            coordsW point = samplePoint();
            
            findNearestNode(treeRRT, point);

            coordsW newPoint = extentToPoint(nearestRRT, point);

            bool isObstructed = checkObstacle(nearestRRT, newPoint);
            if (!isObstructed)
            {
                if (newPoint.x == goalRRT->_xPos && newPoint.y == goalRRT->_yPos)
                {
                    addChild(newPoint);
                    break;
                }
                else
                {
                    addChild(newPoint);
                    if (checkGoal(newPoint))
                    {
                        //std::cout << checkObstacle(goalRRT, newPoint) << std::endl;
                        std::cout << "Found goal!" << std::endl;
                        coordsW temp;
                        temp.x = goalRRT->_xPos;
                        temp.y = goalRRT->_yPos;
                        addChild(temp);
                        break;
                    }
                }          
            }   
        }
    }

    void rrt_planner::clearTree(rrt_nodes *tree)
    {
        if (!tree)
        {
            return;
        }
        for (int i = 0; i < tree->_child.size(); i++)
        {
            clearTree(tree->getChild(i));
        }
        std::cout << "Deleting node " << tree->_xPos << " " << tree->_yPos << " " << tree->_child.size() << std::endl;
        
        if (tree->_xPos == goalRRT->_xPos && tree->_yPos == goalRRT->_yPos)
        {
            std::cout << "This is the goal" << std::endl;
            std::cout << "Deleting goal node " << tree->_xPos << " " << tree->_yPos << " " << tree->_child.size() << std::endl;
            if (tree->_parent)
            {
                std::cout << "Parent node: " << tree->_parent->_xPos << " " << tree->_parent->_yPos << " " << tree->_parent->_child.size() << std::endl;
                for (int i = 0; i < tree->_parent->_child.size(); i++)
                {
                    std::cout << tree->_parent->getChild(i)->_xPos << " " << tree->_parent->getChild(i)->_yPos << std::endl;
                }
            }
        }
        
        if (tree)
        {
            delete tree;
        }
    }

    void rrt_planner::generatePathROS(std::vector<coordsW> & raw_path)
    {
        generateRawPath();
        

        std::cout << "backtracking" << std::endl;

        
        if (goalRRT->_parent)
        {

            std::cout << std::endl;
            backtrack(goalRRT);
            std::cout << "Array size: " << waypoints.size() << std::endl;
            for (int i = 0; i < waypoints.size(); i++)
            {
                std::cout << waypoints[i].x << " " << waypoints[i].y << std::endl;
            }

            raw_path.reserve(waypoints.size());

            coordsW temp;
            costmap_->mapToWorld(treeRRT->_xPos, treeRRT->_yPos, temp.x, temp.y);
            raw_path.push_back(temp);

            for (int i = waypoints.size() - 1; i >= 0; i--)
            {
                costmap_->mapToWorld(waypoints[i].x, waypoints[i].y, temp.x, temp.y);
                raw_path.push_back(temp);
            }

            waypoints.clear();
        }
        else
        {
            std::cout << "IterLimit reached / No suitable path" << std::endl;

        }

        
        std::cout << "Clearing tree\n";
        clearTree(treeRRT);

    }
}