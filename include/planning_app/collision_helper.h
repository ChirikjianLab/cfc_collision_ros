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

#include "cfc/DistanceFixedPoint.h"
#include "cfc/DistanceLeastSquares.h"
#include "cfc/DistanceLeastSquaresCommonNormal.h"
#include "geometry/SuperQuadrics.h"

namespace collision_helper {

/** \brief Early filter out objects that are far from each other
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object
 * \param result Reference to the collision result object */
void filter(const cfc::SuperQuadrics *sq1, const cfc::SuperQuadrics *sq2,
            cfc::DistanceResult &result);

/** \brief Narrow phase collision query for two objects using CFC-Dist-LS
 * algorithm
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object */
cfc::DistanceResult collide(const cfc::SuperQuadrics *sq1,
                            const cfc::SuperQuadrics *sq2);

/** \brief Narrow phase collision query for two objects using CFC-Dist-LS
 * algorithm
 * \param sq1 First superquadric collision object
 * \param sq2 Second superquadric collision object
 * \param opt_algorithm Option for collision algorithm */
cfc::DistanceResult collide(const cfc::SuperQuadrics *sq1,
                            const cfc::SuperQuadrics *sq2,
                            const string &opt_algorithm);

} // namespace collision_helper
