#include <rviz/tool.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>
#include <rviz/selection/selection_manager.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "waypoint_saver.cpp" // 包含waypoint_saver

namespace rviz_plugin
{
class GoalTool : public rviz::Tool
{
public:
  GoalTool() : dragging_(false), waypoint_saver_("/home/ray/waypoints.txt") {}
  ~GoalTool() override {}

  void onInitialize() override
  {
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
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

  void activate() override {}
  void deactivate() override {}

  int processMouseEvent(rviz::ViewportMouseEvent& event) override
  {
    if (event.leftDown())
    {
      Ogre::Vector3 position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, position))
      {
        position.z = 0; // 固定z坐标
        start_position_ = position;
        marker_.pose.position.x = position.x;
        marker_.pose.position.y = position.y;
        marker_.pose.position.z = position.z;
        marker_pub_.publish(marker_);
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
          end_position.z = 0; // 固定z坐标
          Ogre::Vector3 direction = end_position - start_position_;
          direction.z = 0; // 确保方向在水平面上
          direction.normalise();
          Ogre::Quaternion orientation = Ogre::Vector3::UNIT_X.getRotationTo(direction);
          marker_.pose.orientation.x = orientation.x;
          marker_.pose.orientation.y = orientation.y;
          marker_.pose.orientation.z = orientation.z;
          marker_.pose.orientation.w = orientation.w;
          marker_pub_.publish(marker_);

          // 计算并输出四元数和欧拉角
          tf::Quaternion tf_quat(orientation.x, orientation.y, orientation.z, orientation.w);
          double roll, pitch, yaw;
          tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

          ROS_INFO_STREAM("Goal position: " << start_position_.x << ", " << start_position_.y << ", " << start_position_.z);
          ROS_INFO_STREAM("Goal orientation (quaternion): [" << orientation.x << ", " << orientation.y << ", " << orientation.z << ", " << orientation.w << "]");
          ROS_INFO_STREAM("Goal orientation (euler angles): [roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << "]");

          // 保存waypoint
          geometry_msgs::Pose pose;
          pose.position.x = start_position_.x;
          pose.position.y = start_position_.y;
          pose.position.z = start_position_.z;
          pose.orientation.x = orientation.x;
          pose.orientation.y = orientation.y;
          pose.orientation.z = orientation.z;
          pose.orientation.w = orientation.w;
          waypoint_saver_.addWaypoint(pose);
          waypoint_saver_.saveWaypoints();
        }
      }
    }
    else if (event.type == QEvent::MouseMove && dragging_)
    {
      Ogre::Vector3 current_position;
      if (context_->getSelectionManager()->get3DPoint(event.viewport, event.x, event.y, current_position))
      {
        current_position.z = 0; // 固定z坐标
        Ogre::Vector3 direction = current_position - start_position_;
        direction.z = 0; // 确保方向在水平面上
        direction.normalise();
        Ogre::Quaternion orientation = Ogre::Vector3::UNIT_X.getRotationTo(direction);
        marker_.pose.orientation.x = orientation.x;
        marker_.pose.orientation.y = orientation.y;
        marker_.pose.orientation.z = orientation.z;
        marker_.pose.orientation.w = orientation.w;
        marker_pub_.publish(marker_);
      }
    }
    return Render;
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  Ogre::Vector3 start_position_;
  bool dragging_;
  waypoint_saver::WaypointSaver waypoint_saver_;
};

} // namespace rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugin::GoalTool, rviz::Tool)