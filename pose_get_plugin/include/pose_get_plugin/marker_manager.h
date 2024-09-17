#ifndef MARKER_MANAGER_H
#define MARKER_MANAGER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <OgreVector3.h>
#include <OgreQuaternion.h>

namespace rviz_plugin
{
class MarkerManager
{
public:
  void initialize(ros::NodeHandle& nh);
  void setPosition(const Ogre::Vector3& position);
  void setOrientation(const Ogre::Quaternion& orientation);

private:
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
};

} // namespace rviz_plugin

#endif // MARKER_MANAGER_H