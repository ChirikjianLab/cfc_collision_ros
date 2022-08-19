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

#include "util/MultiBodyTree3D.h"

MultiBodyTree3D::MultiBodyTree3D(cfc::SuperQuadrics base) : base_(base) {}

void MultiBodyTree3D::addBody(cfc::SuperQuadrics link) {
  // Add link
  link_.push_back(link);
  numLinks_++;

  // Add tranformation related to parent body
  Eigen::Matrix4d g;
  g.setIdentity();
  g.block<3, 3>(0, 0) = link.quaternion().toRotationMatrix();
  g.block<3, 1>(0, 3) =
      Eigen::Vector3d({link.pos()[0], link.pos()[1], link.pos()[2]});
  tf_.push_back(g);
}

// Transform rigid body
void MultiBodyTree3D::robotTF(Eigen::Matrix4d g) {
  // Set transform of base
  base_.setPosition(g(0, 3), g(1, 3), g(2, 3));

  Eigen::Matrix3d rotMat = g.topLeftCorner(3, 3);
  Eigen::Quaterniond quat(rotMat);
  base_.setOrientation(quat);

  // Set transform for each link
  Eigen::Matrix4d gLink;
  for (size_t i = 0; i < numLinks_; i++) {
    gLink = g * tf_.at(i);

    link_.at(i).setPosition(gLink(0, 3), gLink(1, 3), gLink(2, 3));

    rotMat = gLink.topLeftCorner(3, 3);
    quat = Eigen::Quaterniond(rotMat);
    link_.at(i).setOrientation(quat);
  }
}

// Tranform articulated body
void MultiBodyTree3D::robotTF(const std::string urdfFile,
                              const Eigen::Matrix4d *gBase,
                              const Eigen::VectorXd *jointConfig) {
  ParseURDF kdl(urdfFile);
  robotTF(kdl, gBase, jointConfig);
}

// Tranform articulated body
void MultiBodyTree3D::robotTF(ParseURDF kdl, const Eigen::Matrix4d *gBase,
                              const Eigen::VectorXd *jointConfig) {
  // Set transform of base
  base_.setPosition(gBase->coeff(0, 3), gBase->coeff(1, 3), gBase->coeff(2, 3));

  Eigen::Matrix3d rotMat = gBase->topLeftCorner(3, 3);
  Eigen::Quaterniond quat(rotMat);
  base_.setOrientation(quat);

  // Set transform for each link
  Eigen::Matrix4d gLink;
  KDL::JntArray jointArray;
  jointArray.data = *jointConfig;

  // Get link info from URDF
  urdf::Model urdfModel = kdl.getURDFModel();
  std::vector<urdf::LinkSharedPtr> linksPtr;
  urdfModel.getLinks(linksPtr);

  for (size_t i = 0; i < numLinks_; ++i) {
    gLink = *gBase * kdl.getTransform(&jointArray, linksPtr.at(i + 1)->name) *
            tf_.at(i);

    link_.at(i).setPosition(gLink(0, 3), gLink(1, 3), gLink(2, 3));

    rotMat = gLink.topLeftCorner(3, 3);
    quat = Eigen::Quaterniond(rotMat);
    link_.at(i).setOrientation(quat);
  }
}
