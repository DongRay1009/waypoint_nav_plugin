#ifndef GOAL_TOOL_H
#define GOAL_TOOL_H

#include <rviz/tool.h>
#include <rviz/display_context.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>
#include <rviz/selection/selection_manager.h>
#include <ros/ros.h>
#include "waypoint_saver.h"
#include "marker_manager.h"

namespace rviz_plugin
{
class GoalTool : public rviz::Tool
{
public:
  GoalTool();
  ~GoalTool() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz::ViewportMouseEvent& event) override;

private:
  ros::NodeHandle nh_;
  MarkerManager marker_manager_;
  Ogre::Vector3 start_position_;
  bool dragging_;
  waypoint_saver::WaypointSaver waypoint_saver_;
  int waypoint_id_;
};

} // namespace rviz_plugin

#endif // GOAL_TOOL_H