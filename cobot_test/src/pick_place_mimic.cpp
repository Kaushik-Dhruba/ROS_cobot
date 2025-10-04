#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  
    posture.joint_names.resize(1);
    posture.joint_names[0] = "gripper_right_joint";
    //posture.joint_names[1] = "gripper_left_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0;
    //posture.points[0].positions[1] = 0;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
    posture.joint_names.resize(1);
    posture.joint_names[0] = "gripper_right_joint";
    //posture.joint_names[1] = "gripper_left_joint";

    posture.points.resize(1);
    posture.points[0].positions.resize(1);
    posture.points[0].positions[0] = 0.051;
    //posture.points[0].positions[1] = -0.051;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    orientation.setRPY(tau/4, -tau/4, 0);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 1;
    grasps[0].grasp_pose.pose.position.y = 0.085;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    // Direction is set as positive x axis
    grasps[0].pre_grasp_approach.direction.vector.y = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    //Direction is set as positive z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    openGripper(grasps[0].pre_grasp_posture);

    closedGripper(grasps[0].grasp_posture);

    move_group.setSupportSurfaceName("table1");

    move_group.pick("object", grasps);
    
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "base_link";
    tf2::Quaternion orientation;

    orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 1;
    place_location[0].place_pose.pose.position.z = 0.5;

    place_location[0].pre_place_approach.direction.header.frame_id = "base_link";
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    place_location[0].post_place_retreat.direction.header.frame_id = "base_link";
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    openGripper(place_location[0].post_place_posture);

    group.setSupportSurfaceName("table2");
    
    group.place("object", place_location);

}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;
  
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 1;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    collision_objects[0].operation = collision_objects[0].ADD;


    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "base_link";

    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.2;
    collision_objects[1].primitives[0].dimensions[1] = 0.4;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 1;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].id = "object";
    collision_objects[2].header.frame_id = "base_link";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 1;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("arm");
    group.setPlanningTime(45.0);

    addCollisionObject(planning_scene_interface);
    ros::WallDuration(1.0).sleep();
    pick(group);
    ros::WallDuration(1.0).sleep();
    place(group);
    ros::waitForShutdown();
    return 0;

}
