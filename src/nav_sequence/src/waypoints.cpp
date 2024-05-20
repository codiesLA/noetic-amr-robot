// include ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// move_base pkg
#include <move_base_msgs/MoveBaseAction.h>
// simple move action
#include <actionlib/client/simple_action_client.h>
// stamped point message
#include <geometry_msgs/PointStamped.h>
// tf2_matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// yaml-file handling
#include <yaml-cpp/yaml.h>
// stream library for both read and write from/to files
#include <fstream>

// typedef for convinience
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> ( const YAML::Node& node, T& i )
{
  i = node.as<T>();
}
    
/*
   Build waypoints from yaml file
*/

bool buildWaypointsFromFile(std::vector<geometry_msgs::PointStamped> &waypoints) // pass array of PointStamped messages by reference
{
    // clear waypoints vector
    waypoints.clear();

    // setup waypoints filename
    // declare string for input file name including the path inside the package
    std::string waypoints_filename = "config/waypoints.yaml";
    std::string waypoints_path_filename;

    // get the package path
    std::string pkg_path = ros::package::getPath("nav_sequence");
    waypoints_path_filename = pkg_path + "/" + waypoints_filename;

    #if __DEBUG__
        ROS_INFO( "waypoints filename: %s", waypoints_path_filename.c_str());
    #endif

    try 
    {
        // determine if file opens
        std::ifstream ifs( waypoints_path_filename.c_str(), std::ifstream::in);
        if(!ifs.good()) 
        {
            ROS_FATAL("buildWaypointsFromFile could not open file");
            return false;
        }
        // yaml node
        YAML::Node yaml_node;
        // load file
        yaml_node = YAML::Load(ifs);
        // read data
        const YAML::Node &wp_node_tmp = yaml_node["waypoints"];
        const YAML::Node *wp_node = wp_node_tmp ? &wp_node_tmp : NULL;

        if (wp_node != NULL) 
        {
            // loop over all the waypoints
            for (int i = 0; i < wp_node->size(); i++) 
            {
                // get each waypoint
                // declare 'point' which is used to define each waypoint's coordinates (double)
                geometry_msgs::PointStamped point;

                (*wp_node)[i]["point"]["x"] >> point.point.x;
                (*wp_node)[i]["point"]["y"] >> point.point.y;
                (*wp_node)[i]["point"]["th"] >> point.point.z; // 'th' from here on is 'z'
                waypoints.push_back(point);
            }
        }
        else 
        {
            return false;
        }
    }
    catch (YAML::ParserException &e) 
    {
        return false;
    }
    catch (YAML::RepresentationException &e) 
    {
        return false;
    } 

    return true;
}

/*
   Build Navigation Goal Message and send to MoveBase
*/

void run(std::string ref_frame, move_base_msgs::MoveBaseGoal &goal, const std::vector<geometry_msgs::PointStamped> &waypoints, int index)
{
    // tell the action client that we want to spin a thread by default
    MoveBaseClient action_client("move_base", true);

    // wait for the action server to come up 
    while (!action_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    // fill in the navigation goal message
    goal.target_pose.header.frame_id = ref_frame;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoints[index].point.x;
    goal.target_pose.pose.position.y = waypoints[index].point.y;
    goal.target_pose.pose.position.z = 0.0;

    // convert the degrees to quaternion
    double yaw = (waypoints[index].point.z) *M_PI / 180;
    tf2::Quaternion q;
    q.setRPY(0., 0., yaw);
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    // send navigation goal to MoveBase qq
    ROS_INFO("Sending goal: (%.2f, %.2f, %.2f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z);
    action_client.sendGoal(goal);
    action_client.waitForResult(ros::Duration(180.0)); // waits 120 seconds to receive a result
    if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
        ROS_INFO("The base reached its waypoints");
    else 
        ROS_INFO("The base fail to reach its waypoints");
    ros::Duration(0.5).sleep();
}

/*
   main function
*/

int main(int argc, char **argv) {
    // Create Node
    ros::init(argc, argv, "waypoint_nav_node");
    // Start roscpp node by creating a ros node handle
    ros::NodeHandle ros_nh;
    // reference frame
    std::string ref_frame = "map"; // or use "base_link" for targets in the frame relative to the robots position
    // declare an array of PointStamped messages to keep track of the waypoints
    std::vector<geometry_msgs::PointStamped> waypoints;
    move_base_msgs::MoveBaseGoal goal;
    uint8_t task = 0;
    // parse waypoints from YAML file
    bool built = buildWaypointsFromFile(waypoints);
    int i =0;
    if (!built) 
    {
        ROS_FATAL("building waypoints from header file");
        return 0;
    }

    ros::Rate rate(1); // in Hz, make a best effort at maintaining a particular rate for a loop
    while(ros::ok()) 
    {
        switch (task)
        {
        case 0:
            i++;
            if(i==20) {
                task = 8;
                break;
            }
            ROS_INFO_ONCE("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 0);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ROS_INFO("%d time", i);
            ros::Duration(5).sleep();
            task = 8;
        case 1:
            ROS_INFO("Moving to drop off place");
            // function call to run waypoint navigation to waypoint 1
            run(ref_frame, goal, waypoints, 1);
            task = 2;
        case 2:
            ROS_INFO("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 2);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ros::Duration(5).sleep();
            task = 3;
            break;
        case 3: 
            // ROS_INFO("Done!");
            ROS_INFO("Moving to drop off place");
            // function call to run waypoint navigation to waypoint 1
            run(ref_frame, goal, waypoints, 3);
            task = 4;
            break;
        case 4:
            ROS_INFO_ONCE("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 4);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ros::Duration(5).sleep();
            task = 5;
            break;
        case 5:
            ROS_INFO_ONCE("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 5);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ros::Duration(5).sleep();
            task = 6;
            break;
        case 6:
            ROS_INFO_ONCE("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 6);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ros::Duration(5).sleep();
            task = 7;
            break;

        case 7:
            ROS_INFO_ONCE("Moving to pick up place");
            // function call to run waypoint navigation to waypoint 0
            run(ref_frame, goal, waypoints, 7);
            ROS_INFO("Loading of goods ... (waiting for 5 seconds)");
            ros::Duration(5).sleep();
            task = 0;
            break;
        case 8:
            ROS_INFO_ONCE("Done!");
            break;
        }
    }
    return 0;
}