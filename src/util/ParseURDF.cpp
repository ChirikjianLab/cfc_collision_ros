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

#include "util/ParseURDF.h"

#include "kdl_parser/kdl_parser.hpp"

ParseURDF::ParseURDF(const KDL::Tree &kdlTree) : kdlTree_(kdlTree) {}

ParseURDF::ParseURDF(const std::string urdfFile) {
  if (!kdl_parser::treeFromFile(urdfFile, kdlTree_)) {
    std::cout << "Failed to parse and construct KDL tree..." << std::endl;
  }

  if (!urdfModel_.initFile(urdfFile)) {
    std::cout << "Failed to parse urdf file" << std::endl;
  }
}

Eigen::Matrix4d ParseURDF::getTransform(const KDL::JntArray *jointConfig,
                                        const std::string bodyName) {
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  KDL::Frame frame;

  KDL::TreeFkSolverPos_recursive kinematics(kdlTree_);
  if (kinematics.JntToCart(*jointConfig, frame, bodyName) < 0) {
    std::cout << "Error in solving forward kinematics" << std::endl;
  }

  // Assign data to Eigen Matrix
  Eigen::Quaterniond quat;
  frame.M.GetQuaternion(quat.x(), quat.y(), quat.z(), quat.w());
  transform.topLeftCorner(3, 3) = quat.toRotationMatrix();
  transform.topRightCorner(3, 1) =
      Eigen::Vector3d(frame.p.data[0], frame.p.data[1], frame.p.data[2]);

  return transform;
}
