#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <custom_msgs/pointData.h>
#include <custom_msgs/pointDataArray.h>

#include <mutex>

static int seq_num = 1;

double resolution;

std::vector<octomap::point3d> centerArray;
octomap::point3d goal;
std::string odometry_frame_id;
std::mutex visMutex;

void arrayCallback(const custom_msgs::pointDataArray::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(visMutex);

    centerArray.clear();

    for (int i=0; i< msg->centerPointsArray.size(); i++)
    {
        octomap::point3d position = octomap::point3d(msg->centerPointsArray[i].x, msg->centerPointsArray[i].y, msg->centerPointsArray[i].z);
        centerArray.push_back(position);
    }
}

void goalCallback(const custom_msgs::pointData::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(visMutex);
	goal = octomap::point3d(msg->x, msg->y, msg->z);
}

int main(int argc, char **argv){

	ros::init (argc, argv, "Goal_Visualizer");
	ros::NodeHandle node;

    ROS_INFO("Initialized the rviz_node");

    node.param("cluster/resolution",                resolution,         0.05);
    node.param<std::string>("map/frame",            odometry_frame_id,  "odom");

    octomap::ColorOcTree* outTree = new octomap::ColorOcTree(resolution);

    ROS_INFO("rviz_node : loaded parameters");

    ros::Subscriber array_sub = node.subscribe("goalCenterArray", 1, arrayCallback);
	ros::Subscriber goal_sub = node.subscribe("goalCenters", 1, goalCallback);

    ROS_INFO("rviz_node : subscribers created");

	ros::Publisher map_pub = node.advertise<octomap_msgs::Octomap>("octomap_centers", 1, true);

    ROS_INFO("rviz_node : publishers advertised");

    ros::AsyncSpinner spinner (3);
    spinner.start();

	while(ros::ok()){
        outTree->clear();

        std::unique_lock<std::mutex> lock(visMutex);

        for(int i=0; i<centerArray.size(); i++){
            octomap::ColorOcTreeNode* n = outTree->updateNode(centerArray[i], true);
            n->setColor(255, 0, 0); // set color to red
        }

        octomap::ColorOcTreeNode* n = outTree->updateNode(goal, true);
        n->setColor(0, 255, 0); // set color to green

        lock.unlock();
        
        octomap_msgs::Octomap map_msg;
        octomap_msgs::fullMapToMsg(*outTree, map_msg); // (.ot)

        map_msg.header.frame_id = "map";
        map_msg.header.seq = seq_num;  seq_num+=1;

        map_pub.publish(map_msg);

        ros::Duration(0.01).sleep();
    }
	return 0;
}

  
