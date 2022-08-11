#include "util/Parse2dCsvFile.h"
#include "util/ParseURDF.h"

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <fstream>

/**
 * \brief Inverse Kinematics
 * \param end_effector_state end effector pose
 * \param timeout: 0.1 s
 */
bool getIK(const moveit::core::JointModelGroup* joint_model_group,
           moveit::core::RobotStatePtr kinematic_state,
           const Eigen::Isometry3d& ee_pose, const double timeout,
           const std::vector<std::string>& joint_names,
           std::vector<double>& joint_values) {
    bool found_ik =
        kinematic_state->setFromIK(joint_model_group, ee_pose, timeout);

    // Now, we can print out the IK solution (if found):
    if (found_ik) {
        kinematic_state->copyJointGroupPositions(joint_model_group,
                                                 joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    } else {
        ROS_INFO("Cannot find IK solution!");
    }

    return found_ik;
}

Eigen::Isometry3d getTFMatrix(const std::vector<double>& config) {
    Eigen::Isometry3d g = Eigen::Isometry3d::Identity();

    Eigen::Vector3d axis(config[3], config[4], config[5]);
    axis.normalize();

    Eigen::Quaterniond quat(Eigen::AngleAxisd(config[6], axis));

    g.linear() = quat.toRotationMatrix();
    g.translation() = Eigen::Vector3d({config[0], config[1], config[2]});

    return g;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ik_solver");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    std::string ROBOT_NAME;
    std::string EE_POSE_FILE;
    std::string JOINT_CONFIG_FILE;

    nh.getParam("robot_name", ROBOT_NAME);
    nh.getParam("ee_pose_file", EE_POSE_FILE);
    nh.getParam("joint_config_file", JOINT_CONFIG_FILE);

    // Load robot model
    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    const moveit::core::RobotModelPtr& kinematic_model =
        robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    moveit::core::RobotStatePtr kinematic_state(
        new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group =
        kinematic_model->getJointModelGroup(ROBOT_NAME);

    const std::vector<std::string>& joint_names =
        joint_model_group->getActiveJointModelNames();

    // Get end effector poses from the list
    auto ee_poses = parse2DCsvFile(EE_POSE_FILE);

    std::ofstream outfile;
    outfile.open(JOINT_CONFIG_FILE);

    for (size_t i = 0; i < ee_poses.size(); ++i) {
        ROS_INFO_STREAM("IK for pose # " << i);

        auto ee_pose_tf = getTFMatrix(ee_poses.at(i));

        // Solve IK for poses
        std::vector<double> q;
        const double timeout = 0.1;

        bool found_ik = getIK(joint_model_group, kinematic_state, ee_pose_tf,
                              timeout, joint_names, q);

        // Store joint-space states into files
        for (size_t j = 0; j < joint_names.size(); ++j) {
            if (found_ik) {
                outfile << q.at(j) << ',';
            } else {
                outfile << 0.0 << ',';
            }
        }
        outfile << std::endl;
    }

    outfile.close();
    ROS_INFO("Stored IK solution!");

    ros::shutdown();
    return 0;
}
