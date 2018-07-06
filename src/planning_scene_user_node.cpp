#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>

/**
 * @class PlannigSceneUserNode
 *
 * A node with a planning scene monitor that prints out infos about the stored
 * planning scene when a service is
 * triggered.
 */
class PlannigSceneUserNode {
public:
  bool init();

protected:
  boost::shared_ptr<tf::TransformListener> _transformListenerPtr;
  planning_scene_monitor::PlanningSceneMonitorPtr _planningSceneMonitorPtr;

  ros::ServiceServer _srvPrintInfos;

  /**
   * service callback for printing transform information from within the
   * planning scene
   */
  bool _cbPrintPlanningSceneTransforms(std_srvs::EmptyRequest& req,
                                       std_srvs::EmptyResponse& resp);
};

bool PlannigSceneUserNode::init() {

  ros::NodeHandle nh("~");

  _transformListenerPtr = boost::make_shared<tf::TransformListener>(ros::Duration(10.0));
  _planningSceneMonitorPtr = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      "robot_description", _transformListenerPtr);

  // guard against empty planning scene
  if (_planningSceneMonitorPtr->getRobotDescription().empty()) {
    return false;
  }

  // initialize monitor
  _planningSceneMonitorPtr->startSceneMonitor();
  _planningSceneMonitorPtr->startWorldGeometryMonitor();
  _planningSceneMonitorPtr->startStateMonitor();

  // establish printing service
  _srvPrintInfos =
      nh.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
          "print_infos", boost::bind(&PlannigSceneUserNode::_cbPrintPlanningSceneTransforms,
                                     this, _1, _2));

  return true;
}

bool PlannigSceneUserNode::_cbPrintPlanningSceneTransforms(std_srvs::EmptyRequest& req,
                                                           std_srvs::EmptyResponse& resp) {

  _planningSceneMonitorPtr->updateFrameTransforms();
  planning_scene_monitor::LockedPlanningSceneRO ls(_planningSceneMonitorPtr);
  const moveit::core::RobotState robotState = ls->getCurrentState();

  ROS_INFO("--- INFO START ---");
  robotState.printTransforms();
  ROS_INFO("--- INFO END ---");

  return true;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "palnning scene user");
  PlannigSceneUserNode node;
  if (!node.init()) {
    return EXIT_FAILURE;
  }

  ros::spin();
}
