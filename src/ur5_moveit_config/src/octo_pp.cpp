#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>

const std::string PLANNING_GROUP_UR5 = "ur5";
const std::string PLANNING_GROUP_GRIPPER = "gripper";
const double JUMP_THRESHOLD = 0.0;
const double EEF_STEP = 0.01;

int main(int argc, char **argv) {

  ros::init(argc, argv, "octo_pp");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group_ur5(PLANNING_GROUP_UR5);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(PLANNING_GROUP_GRIPPER);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  // Boxes are added to the scene as CollisionObjects

  // Create two collision objects for the robot's planning scene
  moveit_msgs::CollisionObject table_object, box_object;

  // Set the header frame ID to the robot's planning frame
  const std::string planning_frame = move_group_ur5.getPlanningFrame();
  table_object.header.frame_id = planning_frame;
  box_object.header.frame_id = planning_frame;

  // Define the table's properties
  table_object.id = "base_table";
  shape_msgs::SolidPrimitive table_shape;
  table_shape.type = table_shape.BOX;
  table_shape.dimensions.resize(3);
  table_shape.dimensions[0] = 0.5;
  table_shape.dimensions[1] = 0.5;
  table_shape.dimensions[2] = 0.02;
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.z = -0.01;
  table_object.primitives.push_back(table_shape);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;

  // Define the box's properties
  box_object.id = "box1";
  shape_msgs::SolidPrimitive box_shape;
  box_shape.type = box_shape.BOX;
  box_shape.dimensions.resize(3);
  box_shape.dimensions[0] = 0.05;
  box_shape.dimensions[1] = 0.05;
  box_shape.dimensions[2] = 0.05;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.4;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.2;
  box_object.primitives.push_back(box_shape);
  box_object.primitive_poses.push_back(box_pose);
  box_object.operation = box_object.ADD;

  // Add the collision objects to a vector
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(table_object);
  collision_objects.push_back(box_object);

  planning_scene_interface.applyCollisionObjects(collision_objects);

  ROS_INFO("Added a box into the planning scene");

  ros::Duration(0.1).sleep();

  // Allow collision betn box and gripper

  planning_scene_monitor::LockedPlanningSceneRW ls(planning_scene_monitor);
  collision_detection::AllowedCollisionMatrix &acm = ls->getAllowedCollisionMatrixNonConst();
  acm.setEntry("box1", "robotiq_85_left_finger_tip_link", true);
  acm.setEntry("box1", "robotiq_85_right_finger_tip_link", true);
  std::cout << "\nAllowedCollisionMatrix:\n";
  acm.print(std::cout);

  moveit_msgs::PlanningScene diff_scene;
  ls->getPlanningSceneDiffMsg(diff_scene);

  planning_scene_interface.applyPlanningScene(diff_scene);

  ROS_INFO("Allowed the collision between box and gripper");
  ros::Duration(0.1).sleep();

  // Move the robot to the home position
  move_group_ur5.setJointValueTarget(move_group_ur5.getNamedTargetValues("home"));
  move_group_ur5.move();
  ROS_INFO("Moved to home position");

  // Move the robot above the box
  move_group_ur5.setStartStateToCurrentState();
  geometry_msgs::PoseStamped current_pose = move_group_ur5.getCurrentPose("ee_link");
  geometry_msgs::Pose target_pose_1;
  target_pose_1.orientation = current_pose.pose.orientation;
  target_pose_1.position.x = 0.4;
  target_pose_1.position.y = -0.2;
  target_pose_1.position.z = 0.775 - 0.594 + 0.3;
  move_group_ur5.setPoseTarget(target_pose_1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_ur5.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success)
  {
    move_group_ur5.move();
    ROS_INFO("Moved above box");
    }
    else
    {
      ROS_ERROR("Failed to move above box");
      return 1;
    }

    // Opening the gripper
    move_group_gripper.setJointValueTarget(move_group_gripper.getNamedTargetValues("gripper_open"));
    move_group_gripper.move();

    // cartesian is better here
    geometry_msgs::Pose initState = move_group_ur5.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose = initState;
    
    for (int i = 0; i < 4; i++)
    {
      target_pose.position.z -= 0.03;
      waypoints.push_back(target_pose);
    }
    
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_ur5.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    my_plan.trajectory_ = trajectory;
    move_group_ur5.execute(my_plan);

    ROS_INFO("Cartesian motion to box done %s", success ? "" : "FAILED");


// Closing the gripper

    move_group_gripper.setJointValueTarget(move_group_gripper.getNamedTargetValues("gripper_closed"));

    move_group_gripper.move();

// attach the box to the gripper

    moveit_msgs::AttachedCollisionObject aco;
    aco.object.id = "box1";
    aco.link_name = "robotiq_85_right_finger_tip_link";
    aco.touch_links.push_back("robotiq_85_left_finger_tip_link");
    aco.object.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyAttachedCollisionObject(aco);

    // Cartesian motion above the box works better here

    geometry_msgs::Pose currState = move_group_ur5.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_1;

    geometry_msgs::Pose target_1 = currState;
    target_1.position.z += 0.1;
    waypoints_1.push_back(target_1);

    moveit_msgs::RobotTrajectory trajectory_1;
    double fraction_1 = move_group_ur5.computeCartesianPath(waypoints_1, EEF_STEP, JUMP_THRESHOLD, trajectory_1);

    my_plan.trajectory_ = trajectory_1;
    move_group_ur5.execute(my_plan);
    ROS_INFO("Cartesian above %s", success ? "" : "FAILED");

    // Move box to the other end (Will have to move to a waypoint before moving this)
    geometry_msgs::Pose target_pose_2;
    target_pose_2.position.x = 0.4;
    target_pose_2.position.y = 0.3;
    target_pose_2.position.z = 0.775-0.594+0.1;

    move_group_ur5.setPoseTarget(target_pose_2);

    success = (move_group_ur5.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group_ur5.move();

    currState = move_group_ur5.getCurrentPose().pose;
    std::vector<geometry_msgs::Pose> waypoints_2;

    geometry_msgs::Pose target_2 = currState;
    target_2.position.z -= 0.17;
    waypoints_2.push_back(target_2);
    
    moveit_msgs::RobotTrajectory trajectory_2;
    double fraction_2 = move_group_ur5.computeCartesianPath(waypoints_2, EEF_STEP, JUMP_THRESHOLD, trajectory_2);

    my_plan.trajectory_ = trajectory_2;
    move_group_ur5.execute(my_plan);

    // Open the gripper
    move_group_gripper.setJointValueTarget(move_group_gripper.getNamedTargetValues("gripper_open"));

    move_group_gripper.move();

    std::vector<std::string> object_ids;
    object_ids.push_back(box_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    move_group_ur5.setJointValueTarget(move_group_ur5.getNamedTargetValues("home"));
    move_group_ur5.move();

    ros::shutdown();
    return 0;

}

