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

/** \author Xiaoli Wang */

#include "util/Parse2dCsvFile.h"
#include "util/ParseURDF.h"

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <fstream>

#include <cfc_collision_ros/SolveIK.h>

class SolveIK {
public:
  SolveIK(ros::NodeHandle &n)
      : nh(n), ROBOT_NAME("panda_arm"), robot_model_loader("robot_description"),
        kinematic_model(robot_model_loader.getModel()),
        kinematic_state(new moveit::core::RobotState(kinematic_model)),
        joint_model_group(kinematic_model->getJointModelGroup(ROBOT_NAME)),
        joint_names(joint_model_group->getActiveJointModelNames()) {
    nh.getParam("ee_pose_file", EE_POSE_FILE);
    nh.getParam("joint_config_file", JOINT_CONFIG_FILE);

    srv_solveIk = nh.advertiseService("solve_ik_service",
                                      &SolveIK::callBackSolveIK, this);

    ROS_INFO("Finished construct SolveIK object, ready to provide SolveIK "
             "service");
  }

public:
  bool callBackSolveIK(cfc_collision_ros::SolveIK::Request &req,
                       cfc_collision_ros::SolveIK::Response &res) {
    // Get end effector poses from the list
    ee_poses = parse2DCsvFile(EE_POSE_FILE);

    std::ofstream outfile;
    outfile.open(JOINT_CONFIG_FILE);

    for (size_t i = 0; i < ee_poses.size(); ++i) {
      ROS_INFO_STREAM("IK for pose # " << i);

      auto ee_pose_tf = getTFMatrix(ee_poses.at(i));

      // Solve IK for poses
      const double timeout = 0.1;

      bool found_ik = getIK(ee_pose_tf, timeout);

      res.success = 1;
      // Store joint-space states into files
      for (size_t j = 0; j < joint_names.size(); ++j) {
        if (found_ik) {
          outfile << joint_values.at(j) << ',';
        } else {
          res.success = 0;
          outfile.close();
          ROS_ERROR("Unable to find IK solution for pose %ld", j);
          return 0;
        }
      }
      outfile << std::endl;
      if (i == 0) {
        res.firstJointPositions = joint_values;
      } else {
        res.secondJointPositions = joint_values;
      }
    }

    outfile.close();

    ROS_INFO("Stored IK solution!");
    return 1;
  }

  /**
   * \brief Inverse Kinematics
   * \param end_effector_state end effector pose
   * \param timeout: 0.1 s
   */
  bool getIK(const Eigen::Isometry3d &ee_pose, const double timeout) {
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group, ee_pose, timeout);

    // Now, we can print out the IK solution (if found):
    if (found_ik) {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    } else {
      ROS_INFO("Cannot find IK solution!");
    }

    return found_ik;
  }

  Eigen::Isometry3d getTFMatrix(const std::vector<double> &config) {
    Eigen::Isometry3d g = Eigen::Isometry3d::Identity();

    Eigen::Vector3d axis(config[3], config[4], config[5]);
    axis.normalize();

    Eigen::Quaterniond quat(Eigen::AngleAxisd(config[6], axis));

    g.linear() = quat.toRotationMatrix();
    g.translation() = Eigen::Vector3d({config[0], config[1], config[2]});

    return g;
  }

private:
  ros::NodeHandle nh;
  ros::ServiceServer srv_solveIk;

  std::string ROBOT_NAME;
  std::string EE_POSE_FILE;
  std::string JOINT_CONFIG_FILE;

  robot_model_loader::RobotModelLoader robot_model_loader;
  const moveit::core::RobotModelPtr &kinematic_model;
  moveit::core::RobotStatePtr kinematic_state;
  moveit::core::JointModelGroup *joint_model_group;

  std::vector<std::vector<double>> ee_poses;
  const std::vector<std::string> &joint_names;
  std::vector<double> joint_values;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_ik_solver_service");
  ros::NodeHandle nh;

  SolveIK solve_ik(nh);

  ros::spin();

  return 0;
}
