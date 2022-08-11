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

struct RobotSQGeometry {
    std::string link_name;
    std::size_t collision_index;
    cfc::SuperQuadrics* sq;
};

RobotSQGeometry constructRobotSQGeometry(const std::string link_name,
                                         const std::size_t index,
                                         const shapes::ShapeConstPtr& shape);

void constructSQGeometry(const shapes::ShapeConstPtr& shape, cfc::Shape3D* sq);

void checkCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm);

void checkRobotWorldCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm);

void checkRobotSelfCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm);

void checkCollision(const MultiBodyTree3D& robot,
                    const std::vector<cfc::SuperQuadrics>& obstacle,
                    bool* isCollide);

void checkCollision(const std::vector<cfc::SuperQuadrics>& robot,
                    const std::vector<cfc::SuperQuadrics>& obstacle,
                    bool* isCollide);

void checkRobotWorldCollision(const std::vector<cfc::SuperQuadrics>& robot,
                              const std::vector<cfc::SuperQuadrics>& obstacle,
                              bool* isCollide);

void checkRobotSelfCollision(const std::vector<cfc::SuperQuadrics>& robot,
                             bool* isCollide);

void queryRobotWorldDistance(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::DistanceRequest* request,
    collision_detection::DistanceResult* result);
