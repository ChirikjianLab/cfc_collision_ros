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
#include "util/ParseURDF.h"

#include <eigen3/Eigen/Geometry>

class MultiBodyTree3D {
public:
  MultiBodyTree3D(cfc::SuperQuadrics base);

public:
  cfc::SuperQuadrics getBase() const { return base_; }
  std::vector<cfc::SuperQuadrics> getLinks() const { return link_; }
  std::vector<cfc::SuperQuadrics> getBodyShapes() const {
    std::vector<cfc::SuperQuadrics> body;

    body.push_back(base_);
    for (auto link : link_) {
      body.push_back(link);
    }

    return body;
  }

  double getNumLinks() const { return numLinks_; }
  std::vector<Eigen::Matrix4d> getTF() const { return tf_; }

  void addBody(cfc::SuperQuadrics link);
  void addBody(cfc::SuperQuadrics link, const urdf::Model &urdfModel);

  void robotTF(Eigen::Matrix4d tf);
  void robotTF(const std::string urdfFile, const Eigen::Matrix4d *gBase,
               const Eigen::VectorXd *jointConfig);
  void robotTF(ParseURDF kdl, const Eigen::Matrix4d *gBase,
               const Eigen::VectorXd *jointConfig);

public:
  cfc::SuperQuadrics base_;
  double numLinks_ = 0;
  std::vector<cfc::SuperQuadrics> link_;
  std::vector<Eigen::Matrix4d> tf_;
};
