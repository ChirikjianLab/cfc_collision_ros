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

void plotPoints(const std::vector<std::vector<geometry_msgs::Point>> &points,
                const ros::Publisher &publisher, const double r, const double g,
                const double b);

void plotMesh(const std::vector<EMesh> &mesh_list,
              const ros::Publisher &publisher, const double r, const double g,
              const double b, const double a);

std::vector<geometry_msgs::Point>
getShapeBoundary(const cfc::SuperQuadrics &shape);

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

geometry_msgs::Transform poseToGeomMsgs(const std::vector<double> &pose);
