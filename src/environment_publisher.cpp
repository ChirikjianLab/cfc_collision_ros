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

#include "util/ParsePlanningSettings.h"
#include "util/VisualizationUtils.h"

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

struct EnvironmentFiles {
  std::string obstacle_config;
  std::string obstacle_pcl;
} env_config;

class PlanningEnvironment {
public:
  /** \brief Constructor */
  PlanningEnvironment(ros::NodeHandle nh) : nh_(nh), name_("env_loader") {}

  // Read environmental config files
  void load() {
    std::string ENV_FILE_PREFIX;
    std::string ENV_TYPE;

    nh_.getParam("env_file_prefix", ENV_FILE_PREFIX);
    nh_.getParam("env_type", ENV_TYPE);

    // Read and setup environment config
    ROS_INFO("Loading obstacles...");
    obstacle_ =
        loadVectorGeometry(ENV_FILE_PREFIX + "obstacle_" + ENV_TYPE + ".csv");
  }

  // Setup publishers
  void setup() {
    tf_world_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf_world_.setRotation(tf::Quaternion(0, 0, 0, 1));
    world_frame_pub_.sendTransform(
        tf::StampedTransform(tf_world_, ros::Time::now(), "map", "world"));

    obstacle_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("obstacle", 1);
    obstacle_surf_pub_ =
        nh_.advertise<visualization_msgs::MarkerArray>("obstacle_surf", 1);
  }

  /** \brief Visualize 2D arena and obstacles */
  void visualize() {
    plotShapePoint(obstacle_, obstacle_pub_, 1.0, 0.0, 0.0);
    plotShapeSurf(obstacle_, obstacle_surf_pub_, 1.0, 0.0, 0.0, 1.0);
  }

private:
  ros::NodeHandle nh_;
  std::string name_;

  std::vector<cfc::SuperQuadrics> obstacle_;

  tf::TransformBroadcaster world_frame_pub_;
  tf::Transform tf_world_;

  ros::Publisher obstacle_pub_;
  ros::Publisher obstacle_surf_pub_;
  std::vector<ros::Publisher> obstacle_pcl_pub_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "environment_publisher");
  ros::NodeHandle nh;
  ROS_INFO("Loading Planning Environment...");

  PlanningEnvironment visual(nh);
  visual.load();
  visual.setup();

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()) {
    visual.visualize();

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
