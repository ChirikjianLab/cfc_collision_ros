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
#include "util/MultiBodyTree3D.h"

#include <moveit/collision_detection/collision_tools.h>
#include <moveit/macros/class_forward.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <memory>
#include <set>

MOVEIT_STRUCT_FORWARD(RobotSQGeometry);

/** \brief Structure including link info and shape */
struct RobotSQGeometry {
  /** \brief Name of the link */
  std::string link_name;

  /** \brief Index of the collision geometry within the link */
  std::size_t collision_index;

  /** \brief Superquadric model */
  cfc::SuperQuadrics *sq;
};

/** \brief Construct superquadric geometry model for the robot */
RobotSQGeometry constructRobotSQGeometry(const std::string link_name,
                                         const std::size_t index,
                                         const shapes::ShapeConstPtr &shape);

/** \brief Construct superquadric geometric model */
void constructSQGeometry(const shapes::ShapeConstPtr &shape, cfc::Shape3D *sq);

/** \brief Check collision using MoveIt collision request and result structure
 */
void checkCollision(
    const std::vector<RobotSQGeometryPtr> &robot,
    const std::vector<cfc::SuperQuadrics> &obstacle,
    const moveit::core::RobotStatePtr &state,
    const collision_detection::CollisionRequest *request,
    collision_detection::CollisionResult *result,
    const collision_detection::AllowedCollisionMatrixConstPtr &acm);

/** \brief Check collision with world object using MoveIt collision request and
 * result structure */
void checkRobotWorldCollision(
    const std::vector<RobotSQGeometryPtr> &robot,
    const std::vector<cfc::SuperQuadrics> &obstacle,
    const moveit::core::RobotStatePtr &state,
    const collision_detection::CollisionRequest *request,
    collision_detection::CollisionResult *result,
    const collision_detection::AllowedCollisionMatrixConstPtr &acm);

/** \brief Check collision with other links using MoveIt collision request and
 * result structure */
void checkRobotSelfCollision(
    const std::vector<RobotSQGeometryPtr> &robot,
    const moveit::core::RobotStatePtr &state,
    const collision_detection::CollisionRequest *request,
    collision_detection::CollisionResult *result,
    const collision_detection::AllowedCollisionMatrixConstPtr &acm);

/** \brief Check collision using MultiBodyTree3D representation */
void checkCollision(const MultiBodyTree3D &robot,
                    const std::vector<cfc::SuperQuadrics> &obstacle,
                    bool *isCollide);

/** \brief Check collision using a list of cfc::SuperQuadrics class */
void checkCollision(const std::vector<cfc::SuperQuadrics> &robot,
                    const std::vector<cfc::SuperQuadrics> &obstacle,
                    bool *isCollide);

/** \brief Check collision with world objects using a list of cfc::SuperQuadrics
 * class */
void checkRobotWorldCollision(const std::vector<cfc::SuperQuadrics> &robot,
                              const std::vector<cfc::SuperQuadrics> &obstacle,
                              bool *isCollide);

/** \brief Check collision with other links using a list of cfc::SuperQuadrics
 * class */
void checkRobotSelfCollision(const std::vector<cfc::SuperQuadrics> &robot,
                             bool *isCollide);

/** \brief Query distance using MoveIt distance request and result structures */
void queryRobotWorldDistance(
    const std::vector<RobotSQGeometryPtr> &robot,
    const std::vector<cfc::SuperQuadrics> &obstacle,
    const moveit::core::RobotStatePtr &state,
    const collision_detection::DistanceRequest *request,
    collision_detection::DistanceResult *result);
