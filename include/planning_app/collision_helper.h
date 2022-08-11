#pragma once

#include "cfc/DistanceFixedPoint.h"
#include "cfc/DistanceLeastSquares.h"
#include "cfc/DistanceLeastSquaresCommonNormal.h"
#include "geometry/SuperQuadrics.h"

namespace collision_helper {

/** \brief Early filter out objects that are far from each other
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object
 * \param result Reference to the collision result object */
void filter(const cfc::SuperQuadrics* sq1, const cfc::SuperQuadrics* sq2,
            cfc::DistanceResult& result);

/** \brief Narrow phase collision query for two objects using CFC-Dist-LS
 * algorithm
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object */
cfc::DistanceResult collide(const cfc::SuperQuadrics* sq1,
                            const cfc::SuperQuadrics* sq2);

/** \brief Narrow phase collision query for two objects using CFC-Dist-LS
 * algorithm
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object
 * \param opt_algorithm Option for collision algorithm */
cfc::DistanceResult collide(const cfc::SuperQuadrics* sq1,
                            const cfc::SuperQuadrics* sq2,
                            const string& opt_algorithm);

}  // namespace collision_helper
