#include "pose_get_plugin/goal_tool.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace rviz_plugin
{
GoalTool::GoalTool() : dragging_(false), waypoint_saver_("/home/ray/waypoints.txt"), waypoint_id_(1) {}

GoalTool::~GoalTool() {}

void GoalTool::onInitialize()
{
  marker_manager_.initialize(nh_);
}

void GoalTool::activate() {}

void GoalTool::deactivate() {}

int GoalTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  if (event.leftDown())
  {
    Ogre::Vector3 position;
    if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
    {
      position.z = 0; // Fix z coordinate
      start_position_ = position;
      marker_manager_.setPosition(position);
      dragging_ = true;
    }
  }
  else if (event.leftUp())
  {
    if (dragging_)
    {
      dragging_ = false;
      Ogre::Vector3 end_position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, end_position))
      {
        end_position.z = 0; // Fix z coordinate
        Ogre::Vector3 direction = end_position - start_position_;
        direction.z = 0; // Ensure direction is on the horizontal plane
        direction.normalise();
        Ogre::Quaternion orientation = Ogre::Vector3::UNIT_X.getRotationTo(direction);
        marker_manager_.setOrientation(orientation);

        // Calculate and output quaternion and Euler angles
        tf2::Quaternion tf_quat(orientation.x, orientation.y, orientation.z, orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        ROS_INFO_STREAM("Goal position: " << start_position_.x << ", " << start_position_.y << ", " << start_position_.z);
        ROS_INFO_STREAM("Goal orientation (quaternion): [" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << "]");

        // Save waypoint
        geometry_msgs::Pose pose;
        pose.position.x = start_position_.x;
        pose.position.y = start_position_.y;
        pose.position.z = start_position_.z;
        pose.orientation.x = orientation.x;
        pose.orientation.y = orientation.y;
        pose.orientation.z = orientation.z;
        pose.orientation.w = orientation.w;
        waypoint_saver_.addWaypoint(pose, waypoint_id_);
        waypoint_saver_.saveWaypoints();

        // Output waypoint information
        ROS_INFO_STREAM("Waypoint ID: " << waypoint_id_ << ", Position: (" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "), Orientation: (" << pose.orientation.x << ", " << pose.orientation.y << ", " << pose.orientation.z << ", " << pose.orientation.w << ")");
        ROS_INFO_STREAM("Waypoints saved to file.");

        // Increment waypoint ID
        waypoint_id_++;
      }
    }
  }
  else if (event.rightDown())
  {
    waypoint_saver_.removeLastWaypoint();
    if (waypoint_id_ > 1) // Ensure ID is not less than 1
    {
      waypoint_id_--;
    }
  }
  else if (event.type == QEvent::MouseMove && dragging_)
  {
    Ogre::Vector3 current_position;
    if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, current_position))
    {
      current_position.z = 0; // Fix z coordinate
      Ogre::Vector3 direction = current_position - start_position_;
      direction.z = 0; // Ensure direction is on the horizontal plane
      direction.normalise();
      Ogre::Quaternion orientation = Ogre::Vector3::UNIT_X.getRotationTo(direction);
      marker_manager_.setOrientation(orientation);
    }
  }
  return Render;
}

} // namespace rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::GoalTool, rviz::Tool)