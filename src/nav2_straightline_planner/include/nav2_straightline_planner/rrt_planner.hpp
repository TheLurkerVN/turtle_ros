#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_straightline_planner/rrt_nodes.hpp"
#include "nav_msgs/msg/path.hpp"
#include <memory>
#include <vector>

const int UNKNOWN_COST = 255;
const int OBS_COST = 254;
const int LETHAL_COST = 200;



namespace rrt_planner
{
    
    class rrt_planner
    {
        public:
            nav2_costmap_2d::Costmap2D * costmap_{};
            int sizeX, sizeY;
            int occupiedMinX;
            int occupiedMinY;
            int occupiedMaxX;
            int occupiedMaxY;
            rrt_nodes* treeRRT;
            rrt_nodes* goalRRT;
            rrt_nodes* nearestRRT;
            //rrt_nodes nearestRRT;
            float nearestDistance = 50000;
            int branchLen = 5;
            int iterations = 50000;
            std::vector<coordsW> waypoints;


            rrt_planner();
            void setStartAndGoal(
                const geometry_msgs::msg::PoseStamped & start,
                const geometry_msgs::msg::PoseStamped & goal);

            coordsW samplePoint();

            float distance(rrt_nodes* node, coordsW point);

            coordsW getUnitVector(rrt_nodes* start, coordsW end);

            void addChild(coordsW point);
            
            coordsW extentToPoint(rrt_nodes* start, coordsW end); 

            bool checkObstacle(rrt_nodes* start, coordsW end);

            bool checkGoal(coordsW point);

            void findNearestNode(rrt_nodes *root, coordsW point);

            void resetNearestValues();

            void backtrack(rrt_nodes *goal);

            void test();

            void generateRawPath();

            void clearTree(rrt_nodes *tree);

            void generatePathROS(std::vector<coordsW> & raw_path);

            
    };
}