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

#include "planning_app/collision_helper.h"

#include "ros/ros.h"

namespace collision_helper {

void filter(const cfc::SuperQuadrics *sq1, const cfc::SuperQuadrics *sq2,
            cfc::DistanceResult &result) {
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

cfc::DistanceResult collide(const cfc::SuperQuadrics *sq1,
                            const cfc::SuperQuadrics *sq2) {
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

cfc::DistanceResult collide(const cfc::SuperQuadrics *sq1,
                            const cfc::SuperQuadrics *sq2,
                            const string &opt_algorithm) {
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

} // namespace collision_helper
