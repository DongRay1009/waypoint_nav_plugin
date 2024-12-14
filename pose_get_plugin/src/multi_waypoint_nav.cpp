#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

struct Waypoint
{
  int id;
  double x, y, z;
  double qx, qy, qz, qw;
};

std::vector<Waypoint> loadWaypoints(const std::string& filename)
{
  std::vector<Waypoint> waypoints;
  std::ifstream file(filename);
  if (!file.is_open())
  {
    ROS_ERROR_STREAM("Failed to open file: " << filename);
    return waypoints;
  }

  std::string line;
  int line_number = 0;
  while (std::getline(file, line))
  {
    line_number++;
    std::istringstream iss(line);
    Waypoint waypoint;
    if (!(iss >> waypoint.id >> waypoint.x >> waypoint.y >> waypoint.z >> waypoint.qx >> waypoint.qy >> waypoint.qz >> waypoint.qw))
    {
      ROS_ERROR_STREAM("Failed to parse line " << line_number << ": " << line);
      continue;
    }
    ROS_INFO_STREAM("Loaded waypoint from line " << line_number << ": " << line);
    waypoints.push_back(waypoint);
  }

  file.close();
  return waypoints;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multi_waypoint_nav");
  ros::NodeHandle nh;

  std::string filename = "/home/ray/waypoints.txt"; // Modify to your waypoints.txt path
  std::vector<Waypoint> waypoints = loadWaypoints(filename);

  if (waypoints.empty())
  {
    ROS_ERROR("No waypoints loaded.");
    return 1;
  }

  // Create a SimpleActionClient to communicate with move_base
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  // Wait for the move_base action server to start
  ROS_INFO("Waiting for move_base action server to start...");
  ac.waitForServer();
  ROS_INFO("move_base action server started.");

  for (const auto& waypoint : waypoints)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = waypoint.x;
    goal.target_pose.pose.position.y = waypoint.y;
    goal.target_pose.pose.position.z = waypoint.z;
    goal.target_pose.pose.orientation.x = waypoint.qx;
    goal.target_pose.pose.orientation.y = waypoint.qy;
    goal.target_pose.pose.orientation.z = waypoint.qz;
    goal.target_pose.pose.orientation.w = waypoint.qw;

    ROS_INFO_STREAM("Sending goal ID " << waypoint.id << ": " << waypoint.x << ", " << waypoint.y << ", " << waypoint.z);
    ac.sendGoal(goal);

    // Wait for the robot to reach the target point
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO_STREAM("Completed waypoint ID " << waypoint.id << " of " << waypoints.size());
    }
    else
    {
      ROS_WARN_STREAM("Failed to reach waypoint ID " << waypoint.id);
    }
  }

  return 0;
}