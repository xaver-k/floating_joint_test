This package demonstrates a bug in the `moveit_ros_planning`-package version 0.9.12 (see https://github.com/ros-planning/moveit), which leads to wrong poses in the PlanningScene if floating joints with non-identity origins are used.


# Installation

(It is assumed you have a ROS Kinetic environment with moviet 0.9.12 already installed.)

* Clone the repository into your workspace.
* Install dependencies: `rosdep install floating_joint_test`
* Build: `catkin build`


# Running

* Launch the demo environment: ` roslaunch floating_joint_test test_environment.launch `
* Make the demo node print the transformations currently stored in the internal PlanningScene: `rosservice call /planning_scene_user/print_infos`


# Explanation

The URDF defines a "world"-link and two child links, `link_A` and `link_B`, both of which are connected to `world` by floating joints.
While the `world_to_link_B`-joint does not define a joint origin (defaults to identity transformation), the `world_to_link_A` does define
one which is not equal to the identity transformation (`xyz="1 0 0" rpy="0 0 0"`).

For both floating joints, we publish a joint transformation of `xyz=[2, 0, 0]` and `rpy=[0, 0, 0]` via TF.
The `planning_scene_user`-node hold an internal PlanningScene, which is updated by a PlanningSceneMonitor.
It is expected that the same values that we publish to TF are also available in the node's internal PlanningScene.

However, if we look at the printout from the planning scene, we get:

```
[ INFO] [1530873081.401119299]: --- INFO START ---
Joint transforms:
  ASSUMED_FIXED_ROOT_JOINT: T.xyz = [0, 0, 0], Q.xyzw = [0, 0, 0, 1]
  world_to_link_A: T.xyz = [2, 0, 0], Q.xyzw = [0, 0, 0, 1]
  world_to_link_B: T.xyz = [2, 0, 0], Q.xyzw = [0, 0, 0, 1]
Link poses:
  world: T.xyz = [0, 0, 0], Q.xyzw = [0, 0, 0, 1]
  link_A: T.xyz = [3, 0, 0], Q.xyzw = [0, 0, 0, 1]
  link_B: T.xyz = [2, 0, 0], Q.xyzw = [0, 0, 0, 1]
[ INFO] [1530873081.401330196]: --- INFO END ---
```

Note how the pose of `link_A` is not what we published, but instead the concatenation of the origin-transform from the URDF and the transform we published via TF.
