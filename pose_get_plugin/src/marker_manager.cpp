#include "pose_get_plugin/marker_manager.h"

namespace rviz_plugin
{
void MarkerManager::initialize(ros::NodeHandle& nh)
{
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  marker_.header.frame_id = "map";
  marker_.ns = "goal_tool";
  marker_.id = 0;
  marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_.mesh_resource = "package://pose_get_plugin/meshes/waypoint.dae";
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.scale.x = 1.0;
  marker_.scale.y = 1.0;
  marker_.scale.z = 1.0;
  marker_.color.r = 1.0;
  marker_.color.g = 1.0;
  marker_.color.b = 0.0;
  marker_.color.a = 1.0;
}

void MarkerManager::setPosition(const Ogre::Vector3& position)
{
  marker_.pose.position.x = position.x;
  marker_.pose.position.y = position.y;
  marker_.pose.position.z = position.z;
  marker_pub_.publish(marker_);
}

void MarkerManager::setOrientation(const Ogre::Quaternion& orientation)
{
  marker_.pose.orientation.x = orientation.x;
  marker_.pose.orientation.y = orientation.y;
  marker_.pose.orientation.z = orientation.z;
  marker_.pose.orientation.w = orientation.w;
  marker_pub_.publish(marker_);
}

} // namespace rviz_plugin