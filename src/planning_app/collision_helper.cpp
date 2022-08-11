#include "planning_app/collision_helper.h"

#include "ros/ros.h"

namespace collision_helper {

void filter(const cfc::SuperQuadrics* sq1, const cfc::SuperQuadrics* sq2,
            cfc::DistanceResult& result) {
    // Culling using Spherical bounding volume
    const double center_dist =
        std::sqrt(std::pow(sq1->pos()[0] - sq2->pos()[0], 2.0) +
                  std::pow(sq1->pos()[1] - sq2->pos()[1], 2.0) +
                  std::pow(sq1->pos()[2] - sq2->pos()[2], 2.0));
    const double sphere_dist =
        std::sqrt(3) *
        (std::max(std::max(sq1->a()[0], sq1->a()[1]), sq1->a()[2]) +
         std::max(std::max(sq2->a()[0], sq2->a()[1]), sq2->a()[2]));

    if (center_dist > sphere_dist) {
        result.is_collision = false;
    }
}

cfc::DistanceResult collide(const cfc::SuperQuadrics* sq1,
                            const cfc::SuperQuadrics* sq2) {
    cfc::DistanceResult result;
    filter(sq1, sq2, result);
    if (!result.is_collision) {
        return result;
    }

    // Collision query
    cfc::collision_cfc::DistanceLeastSquares<cfc::SuperQuadrics,
                                             cfc::SuperQuadrics>
        cfc_distance(sq1, sq2);
    cfc_distance.query();

    result = cfc_distance.getDistanceInfo();

    return result;
}

cfc::DistanceResult collide(const cfc::SuperQuadrics* sq1,
                            const cfc::SuperQuadrics* sq2,
                            const string& opt_algorithm) {
    cfc::DistanceResult result;
    filter(sq1, sq2, result);
    if (!result.is_collision) {
        return result;
    }

    // Collision query, with different choices
    if (opt_algorithm == "cfc_dist_ls") {
        cfc::collision_cfc::DistanceLeastSquares<cfc::SuperQuadrics,
                                                 cfc::SuperQuadrics>
            cfc_distance(sq1, sq2);
        cfc_distance.query();
        result = cfc_distance.getDistanceInfo();
    } else if (opt_algorithm == "cfc_cn_ls") {
        cfc::collision_cfc::DistanceLeastSquaresCommonNormal<cfc::SuperQuadrics,
                                                             cfc::SuperQuadrics>
            cfc_distance(sq1, sq2);
        cfc_distance.query();
        result = cfc_distance.getDistanceInfo();
    } else if (opt_algorithm == "cfc_cn_fp") {
        cfc::collision_cfc::DistanceFixedPoint<cfc::SuperQuadrics,
                                               cfc::SuperQuadrics>
            cfc_distance(sq1, sq2);
        cfc_distance.query();
        result = cfc_distance.getDistanceInfo();
    } else {
        ROS_ERROR_STREAM("Algorithm not supported!");
    }

    return result;
}

}  // namespace collision_helper
