#ifndef WAYPOINT_SAVER_H
#define WAYPOINT_SAVER_H

#include <geometry_msgs/Pose.h>
#include <vector>
#include <string>

namespace waypoint_saver
{
class WaypointSaver
{
public:
  WaypointSaver(const std::string& filename);
  void addWaypoint(const geometry_msgs::Pose& pose, int id);
  void removeLastWaypoint();
  void saveWaypoints();

private:
  std::string filename_;
  std::vector<std::pair<int, geometry_msgs::Pose>> waypoints_;
};

} // namespace waypoint_saver

#endif // WAYPOINT_SAVER_H