/** BSD 3-Clause License
 * Copyright (c) 2022, National University of Singapore
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Sipu Ruan */

#include "planning_app/collision_common.h"
#include "util/interactive_robot.h"
#include "util/pose_string.h"

#include <ros/ros.h>

// MoveIt
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

planning_scene::PlanningScene *g_planning_scene = 0;
shapes::ShapePtr g_world_cube_shape;
ros::Publisher *g_marker_array_publisher = 0;
ros::Publisher *g_marker_min_dist_publisher = 0;
visualization_msgs::MarkerArray g_collision_points;
visualization_msgs::Marker g_min_distance;

void publishMarkers(visualization_msgs::MarkerArray &markers) {
  // delete old markers
  if (g_collision_points.markers.size()) {
    for (int i = 0; i < g_collision_points.markers.size(); i++)
      g_collision_points.markers[i].action = visualization_msgs::Marker::DELETE;

    g_marker_array_publisher->publish(g_collision_points);
  }

  // move new markers into g_collision_points
  std::swap(g_collision_points.markers, markers.markers);

  // draw new markers (if there are any)
  if (g_collision_points.markers.size())
    g_marker_array_publisher->publish(g_collision_points);
}

void publishMinDistLine(const Eigen::Vector3d &x1, const Eigen::Vector3d &x2) {
  g_min_distance.action = visualization_msgs::Marker::DELETE;
  g_marker_min_dist_publisher->publish(g_min_distance);

  visualization_msgs::Marker new_marker;
  // Create new line marker connecting two witness points
  new_marker.header.frame_id = "panda_link0";
  new_marker.ns = "distance";
  new_marker.action = visualization_msgs::Marker::ADD;
  new_marker.pose.orientation.w = 1.0;
  new_marker.id = 0;
  new_marker.type = visualization_msgs::Marker::LINE_LIST;
  new_marker.scale.x = 0.005;
  new_marker.color.a = 1.0;
  new_marker.color.r = 1.0;
  new_marker.color.g = 1.0;
  new_marker.color.b = 0.0;

  // Assign points
  new_marker.points.resize(2);
  new_marker.points.at(0).x = x1(0);
  new_marker.points.at(0).y = x1(1);
  new_marker.points.at(0).z = x1(2);
  new_marker.points.at(1).x = x2(0);
  new_marker.points.at(1).y = x2(1);
  new_marker.points.at(1).z = x2(2);

  new_marker.header.stamp = ros::Time::now();
  new_marker.lifetime = ros::Duration();

  std::swap(g_min_distance, new_marker);

  g_marker_min_dist_publisher->publish(g_min_distance);
}

// Construct Superquadric model for robot links
std::vector<RobotSQGeometryPtr> constructRobotSQModel(InteractiveRobot &robot) {
  std::vector<RobotSQGeometryPtr> links;
  std::vector<shapes::ShapeConstPtr> link_collision_obj;
  RobotSQGeometry link_sq_geom;

  for (auto link_model : robot.robotState()
                             ->getRobotModel()
                             ->getLinkModelsWithCollisionGeometry()) {
    link_collision_obj = link_model->getShapes();

    for (size_t i = 0; i < link_collision_obj.size(); ++i) {
      link_sq_geom = constructRobotSQGeometry(link_model->getName(), i,
                                              link_collision_obj.at(i));

      links.push_back(std::make_shared<RobotSQGeometry>(link_sq_geom));
    }
  }

  return links;
}

// Construct Superquadric model for obstacles
std::vector<cfc::SuperQuadrics>
constructObstacleSQModel(InteractiveRobot &robot) {
  std::vector<cfc::SuperQuadrics> obstacle;

  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_planning_scene->getWorldNonConst()->moveShapeInObject(
      "cubes", g_world_cube_shape, world_cube_pose);

  Eigen::Quaterniond quat(world_cube_pose.rotation());
  cfc::Shape3D obstacle_shape{
      {world_cube_size / 2.0, world_cube_size / 2.0, world_cube_size / 2.0},
      {quat.w(), quat.x(), quat.y(), quat.z()},
      {0.1, 0.1},
      {world_cube_pose(0, 3), world_cube_pose(1, 3), world_cube_pose(2, 3)}};

  obstacle.push_back(cfc::SuperQuadrics(obstacle_shape));

  return obstacle;
}

// Callback function for collision detection
void computeCollisionContactPoints(InteractiveRobot &robot) {
  std::vector<RobotSQGeometryPtr> links = constructRobotSQModel(robot);
  std::vector<cfc::SuperQuadrics> obstacle = constructObstacleSQModel(robot);

  // Checking for Collisions
  collision_detection::CollisionRequest c_req;
  c_req.group_name = robot.getGroupName();
  c_req.contacts = true;
  c_req.max_contacts = 10;
  c_req.max_contacts_per_pair = 1;
  c_req.verbose = false;

  collision_detection::CollisionResult c_res;

  checkCollision(links, obstacle, robot.robotState(), &c_req, &c_res,
                 std::make_shared<collision_detection::AllowedCollisionMatrix>(
                     g_planning_scene->getAllowedCollisionMatrix()));

  // Display results
  if (c_res.collision) {
    ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
    if (c_res.contact_count > 0) {
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.8;
      visualization_msgs::MarkerArray markers;

      /* Get the contact ponts and display them as markers */
      collision_detection::getCollisionMarkersFromContacts(
          markers, "panda_link0", c_res.contacts, color,
          ros::Duration(), // remain until deleted
          0.01);           // radius
      publishMarkers(markers);
    }
  } else {
    ROS_INFO("Not colliding");

    // delete the old collision point markers
    visualization_msgs::MarkerArray empty_marker_array;
    publishMarkers(empty_marker_array);
  }
}

// Callback function for distance query
void queryMinimumDistance(InteractiveRobot &robot) {
  std::vector<RobotSQGeometryPtr> links = constructRobotSQModel(robot);
  std::vector<cfc::SuperQuadrics> obstacle = constructObstacleSQModel(robot);

  // Distance requests
  collision_detection::DistanceRequest d_req;
  d_req.group_name = robot.getGroupName();
  d_req.enable_nearest_points = true;
  d_req.verbose = false;
  d_req.enable_signed_distance = true;

  collision_detection::DistanceResult d_res;

  queryRobotWorldDistance(links, obstacle, robot.robotState(), &d_req, &d_res);

  // Display results
  ROS_INFO_STREAM("Minimum distance: " << d_res.minimum_distance.distance);

  publishMinDistLine(d_res.minimum_distance.nearest_points[0],
                     d_res.minimum_distance.nearest_points[1]);
}

// Callback function for visualizing collision
void demoVisualizeCollision(InteractiveRobot &robot) {
  computeCollisionContactPoints(robot);
  queryMinimumDistance(robot);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_visualize_collision");
  ros::NodeHandle nh;

  InteractiveRobot robot;
  g_planning_scene = new planning_scene::PlanningScene(robot.robotModel());

  // Adding geometry to the PlanningScene
  Eigen::Isometry3d world_cube_pose;
  double world_cube_size;
  robot.getWorldGeometry(world_cube_pose, world_cube_size);
  g_world_cube_shape.reset(
      new shapes::Box(world_cube_size, world_cube_size, world_cube_size));
  g_planning_scene->getWorldNonConst()->addToObject("cubes", g_world_cube_shape,
                                                    world_cube_pose);

  // Create a marker array publisher for publishing contact points
  g_marker_array_publisher =
      new ros::Publisher(nh.advertise<visualization_msgs::MarkerArray>(
          "interactive_robot_marray", 100));
  g_marker_min_dist_publisher = new ros::Publisher(
      nh.advertise<visualization_msgs::Marker>("min_distance_line", 100));

  // Collision detection and distance query
  robot.setUserCallback(demoVisualizeCollision);

  ros::spin();

  delete g_planning_scene;
  delete g_marker_array_publisher;

  ros::shutdown();
  return 0;
}
