#pragma once

#include "geometry/SuperQuadrics.h"
#include "util/MeshGenerator.h"
#include "util/MultiBodyTree3D.h"

#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

void plotPoints(const std::vector<std::vector<geometry_msgs::Point>> &points,
                const ros::Publisher &publisher, const double r, const double g,
                const double b);

void plotMesh(const std::vector<EMesh> &mesh_list,
              const ros::Publisher &publisher, const double r, const double g,
              const double b, const double a);

std::vector<geometry_msgs::Point> getShapeBoundary(
    const cfc::SuperQuadrics &shape);

void plotShapePoint(const std::vector<cfc::SuperQuadrics> &shapes,
                    const ros::Publisher &publisher, const double r,
                    const double g, const double b);

void plotShapeSurf(const std::vector<cfc::SuperQuadrics> &shapes,
                   const ros::Publisher &publisher, const double r,
                   const double g, const double b, const double a);

void plotPlan(MultiBodyTree3D *robot,
              const trajectory_msgs::MultiDOFJointTrajectoryPoint &path,
              const ros::Publisher &path_pub);

void plotRobot(MultiBodyTree3D *robot,
               const geometry_msgs::Transform *transform,
               const ros::Publisher &pub);

void transformRobot(MultiBodyTree3D *robot,
                    const geometry_msgs::Transform *transform);

void plotPointCloud(const pcl::PointCloud<pcl::PointXYZ> *cloud,
                    const ros::Publisher &pcl_pub);

geometry_msgs::Transform poseToGeomMsgs(const std::vector<double> &pose);
