#include "planning_app/collision_common.h"
#include "planning_app/collision_helper.h"

#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/collision_matrix.h>

#include <boost/thread/mutex.hpp>
#include <memory>

RobotSQGeometry constructRobotSQGeometry(const std::string link_name,
                                         const std::size_t index,
                                         const shapes::ShapeConstPtr& shape) {
    RobotSQGeometry geom;
    geom.link_name = link_name;
    geom.collision_index = index;

    cfc::Shape3D sq_shape;
    constructSQGeometry(shape, &sq_shape);
    geom.sq = new cfc::SuperQuadrics(sq_shape);

    return geom;
}

void constructSQGeometry(const shapes::ShapeConstPtr& shape, cfc::Shape3D* sq) {
    if (shape->type == shapes::CYLINDER) {
        std::shared_ptr<const shapes::Cylinder> link_shape =
            std::dynamic_pointer_cast<const shapes::Cylinder>(shape);

        sq->a[0] = link_shape->radius;
        sq->a[1] = link_shape->radius;
        sq->a[2] = link_shape->length / 2.0;
        sq->eps[0] = 0.1;
        sq->eps[1] = 1.0;
    } else if (shape->type == shapes::BOX) {
        std::shared_ptr<const shapes::Box> link_shape =
            std::dynamic_pointer_cast<const shapes::Box>(shape);

        sq->a[0] = link_shape->size[0] / 2.0;
        sq->a[1] = link_shape->size[1] / 2.0;
        sq->a[2] = link_shape->size[2] / 2.0;
        sq->eps[0] = 0.1;
        sq->eps[1] = 0.1;
    } else if (shape->type == shapes::SPHERE) {
        std::shared_ptr<const shapes::Sphere> link_shape =
            std::dynamic_pointer_cast<const shapes::Sphere>(shape);

        sq->a[0] = link_shape->radius;
        sq->a[1] = link_shape->radius;
        sq->a[2] = link_shape->radius;
        sq->eps[0] = 1.0;
        sq->eps[1] = 1.0;
    }
}

void checkCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm) {
    result->collision = false;
    result->contact_count = 0;
    result->distance = Inf;
    result->contacts.clear();

    checkRobotWorldCollision(robot, obstacle, state, request, result, acm);
    checkRobotSelfCollision(robot, state, request, result, acm);
}

void checkRobotWorldCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm) {
    for (auto link : robot) {
        // Tranform each link with respect to world frame
        Eigen::Isometry3d tf = state->getCollisionBodyTransform(
            link->link_name, link->collision_index);
        Eigen::Quaterniond quat(tf.rotation());

        link->sq->setOrientation(quat);
        link->sq->setPosition(tf(0, 3), tf(1, 3), tf(2, 3));

        for (auto obs : obstacle) {
            if (result->contact_count >= request->max_contacts) {
                return;
            }

            // Check collision
            cfc::DistanceResult distanceInfo =
                collision_helper::collide(link->sq, &obs);

            // Record minimum distance
            if (distanceInfo.distance < result->distance) {
                result->distance = distanceInfo.distance;
            }

            // Record contact information
            if (distanceInfo.is_collision) {
                result->collision = true;
                result->contact_count++;

                collision_detection::Contact new_contact;
                new_contact.pos = distanceInfo.closest_point_s1;
                new_contact.depth = distanceInfo.distance;
                new_contact.body_name_1 = link->link_name;
                new_contact.body_type_1 =
                    collision_detection::BodyType::ROBOT_LINK;
                new_contact.body_name_2 = "obstacle";
                new_contact.body_type_2 =
                    collision_detection::BodyType::WORLD_OBJECT;
                new_contact.nearest_points[0] = distanceInfo.closest_point_s1;
                new_contact.nearest_points[1] = distanceInfo.closest_point_s2;

                std::vector<collision_detection::Contact> contacts;
                contacts.push_back(new_contact);

                result->contacts.insert(std::make_pair(
                    std::make_pair(link->link_name, "obstacle"), contacts));
            }
        }
    }
}

void checkRobotSelfCollision(
    const std::vector<RobotSQGeometryPtr>& robot,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::CollisionRequest* request,
    collision_detection::CollisionResult* result,
    const collision_detection::AllowedCollisionMatrixConstPtr& acm) {
    for (auto link_1 : robot) {
        // Tranform each link with respect to world frame
        Eigen::Isometry3d tf = state->getCollisionBodyTransform(
            link_1->link_name, link_1->collision_index);
        Eigen::Quaterniond quat(tf.rotation());

        link_1->sq->setOrientation(quat);
        link_1->sq->setPosition(tf(0, 3), tf(1, 3), tf(2, 3));

        for (auto link_2 : robot) {
            if (result->contact_count >= request->max_contacts) {
                return;
            }

            if (link_1->link_name == link_2->link_name) {
                continue;
            }

            // Use allowed collision matrix to rule out unnecessary collision
            // checks
            collision_detection::AllowedCollision::Type ac_type;
            bool hasEntry =
                acm->getEntry(link_1->link_name, link_2->link_name, ac_type);

            if (ac_type == collision_detection::AllowedCollision::ALWAYS) {
                return;
            }

            // Tranform each link with respect to world frame
            Eigen::Isometry3d tf = state->getCollisionBodyTransform(
                link_2->link_name, link_1->collision_index);
            Eigen::Quaterniond quat(tf.rotation());

            link_2->sq->setOrientation(quat);
            link_2->sq->setPosition(tf(0, 3), tf(1, 3), tf(2, 3));

            // Check collision
            cfc::DistanceResult distanceInfo =
                collision_helper::collide(link_1->sq, link_2->sq);

            // Record minimum distance
            if (distanceInfo.distance < result->distance) {
                result->distance = distanceInfo.distance;
            }

            // Record contact information
            if (distanceInfo.is_collision) {
                result->collision = true;
                result->contact_count++;

                collision_detection::Contact new_contact;
                new_contact.pos = distanceInfo.closest_point_s1;
                new_contact.depth = distanceInfo.distance;
                new_contact.body_name_1 = link_1->link_name;
                new_contact.body_type_1 =
                    collision_detection::BodyType::ROBOT_LINK;
                new_contact.body_name_2 = link_2->link_name;
                new_contact.body_type_2 =
                    collision_detection::BodyType::ROBOT_LINK;
                new_contact.nearest_points[0] = distanceInfo.closest_point_s1;
                new_contact.nearest_points[1] = distanceInfo.closest_point_s2;

                std::vector<collision_detection::Contact> contacts;
                contacts.push_back(new_contact);

                result->contacts.insert(std::make_pair(
                    std::make_pair(link_1->link_name, link_2->link_name),
                    contacts));
            }
        }
    }
}

void checkCollision(const MultiBodyTree3D& robot,
                    const std::vector<cfc::SuperQuadrics>& obstacle,
                    bool* isCollide) {
    std::vector<cfc::SuperQuadrics> robotBody = robot.getBodyShapes();

    checkCollision(robotBody, obstacle, isCollide);
}

void checkCollision(const std::vector<cfc::SuperQuadrics>& robot,
                    const std::vector<cfc::SuperQuadrics>& obstacle,
                    bool* isCollide) {
    bool isRobotWorldCollide = false;
    bool isRobotSelfCollide = false;
    *isCollide = false;

    checkRobotWorldCollision(robot, obstacle, &isRobotWorldCollide);
    checkRobotSelfCollision(robot, &isRobotSelfCollide);

    if (isRobotWorldCollide || isRobotSelfCollide) {
        *isCollide = true;
    }
}

void checkRobotWorldCollision(const std::vector<cfc::SuperQuadrics>& robot,
                              const std::vector<cfc::SuperQuadrics>& obstacle,
                              bool* isCollide) {
    if (*isCollide) {
        return;
    }

    for (auto obs : obstacle) {
        // Collision query
        for (auto body : robot) {
            auto result = collision_helper::collide(&body, &obs, "cfc_cn_fp");

            if (result.is_collision) {
                *isCollide = true;
                return;
            }
        }
    }
}

void checkRobotSelfCollision(const std::vector<cfc::SuperQuadrics>& robot,
                             bool* isCollide) {
    if (*isCollide || robot.size() < 3) {
        return;
    }

    cfc::SuperQuadrics body_1 = robot.at(0);
    cfc::SuperQuadrics body_2 = robot.at(0);

    for (size_t i = 0; i < robot.size() - 2; ++i) {
        body_1 = robot.at(i);

        // Collision query for non-adjacent bodies
        for (size_t j = i + 2; j < robot.size(); ++j) {
            body_2 = robot.at(j);

            auto result =
                collision_helper::collide(&body_1, &body_2, "cfc_cn_fp");

            if (result.is_collision) {
                *isCollide = true;
                return;
            }
        }
    }
}

void queryRobotWorldDistance(
    const std::vector<RobotSQGeometryPtr>& robot,
    const std::vector<cfc::SuperQuadrics>& obstacle,
    const moveit::core::RobotStatePtr& state,
    const collision_detection::DistanceRequest* request,
    collision_detection::DistanceResult* result) {
    result->minimum_distance.distance = Inf;
    result->collision = false;

    for (auto link : robot) {
        // Tranform each link with respect to world frame
        Eigen::Isometry3d tf = state->getCollisionBodyTransform(
            link->link_name, link->collision_index);
        Eigen::Quaterniond quat(tf.rotation());

        link->sq->setOrientation(quat);
        link->sq->setPosition(tf(0, 3), tf(1, 3), tf(2, 3));

        for (auto obs : obstacle) {
            // Check collision
            cfc::DistanceResult distanceInfo =
                collision_helper::collide(link->sq, &obs);

            // Record minimum distance
            if (distanceInfo.distance < result->minimum_distance.distance) {
                // Minimum distance info
                result->minimum_distance.body_types[0] =
                    collision_detection::BodyType::ROBOT_LINK;
                result->minimum_distance.body_types[1] =
                    collision_detection::BodyType::WORLD_OBJECT;

                result->minimum_distance.link_names[0] = link->link_name;
                result->minimum_distance.link_names[1] = "obstacle";

                // Enable signed distance
                if (request->enable_signed_distance) {
                    result->minimum_distance.distance = distanceInfo.distance;
                } else {
                    result->minimum_distance.distance = 0.0;
                }

                // Enable nearest points
                if (request->enable_nearest_points) {
                    result->minimum_distance.nearest_points[0] =
                        distanceInfo.closest_point_s1;
                    result->minimum_distance.nearest_points[1] =
                        distanceInfo.closest_point_s2;
                }
            }

            // Record contact information
            if (distanceInfo.is_collision) {
                result->collision = true;
            }
        }
    }
}
