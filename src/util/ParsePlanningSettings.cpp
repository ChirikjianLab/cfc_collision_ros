#include "util/ParsePlanningSettings.h"

std::vector<cfc::SuperQuadrics> loadVectorGeometry(
    const std::string config_file) {
    // Read config file
    std::vector<std::vector<double>> object_config =
        parse2DCsvFile(config_file);

    // Generate SQ object (orientation from Quaternion parameterization)
    std::vector<cfc::SuperQuadrics> object;
    for (auto config : object_config) {
        Eigen::Vector3d axis(config[8], config[9], config[10]);
        axis.normalize();

        Eigen::Quaterniond quat(Eigen::AngleAxisd(config[11], axis));

        object.emplace_back(
            cfc::SuperQuadrics({{config[0], config[1], config[2]},
                                {quat.w(), quat.x(), quat.y(), quat.z()},
                                {config[3], config[4]},
                                {config[5], config[6], config[7]}}));
    }

    return object;
}

MultiBodyTree3D loadRobotMultiBody3D(const std::string robot_config_file) {
    // Read and setup robot info
    std::vector<cfc::SuperQuadrics> robot_parts =
        loadVectorGeometry(robot_config_file);

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); ++i) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}

MultiBodyTree3D loadRobotMultiBody3D(const string robot_config_file,
                                     const string robot_urdf_file) {
    // Read and setup robot info
    std::vector<cfc::SuperQuadrics> robot_parts =
        loadVectorGeometry(robot_config_file);

    // Load URDF and apply offset for links
    urdf::Model urdf_model;
    urdf_model.initFile(robot_urdf_file);

    std::vector<urdf::LinkSharedPtr> links;
    urdf_model.getLinks(links);

    for (size_t i = 0; i < links.size(); ++i) {
        urdf::Vector3 pos = links.at(i)->collision->origin.position;
        urdf::Rotation rot = links.at(i)->collision->origin.rotation;

        robot_parts.at(i).setPosition(pos.x, pos.y, pos.z);
        robot_parts.at(i).setOrientation(rot.w, rot.x, rot.y, rot.z);
    }

    // Generate multibody tree for robot
    MultiBodyTree3D robot(robot_parts[0]);
    for (size_t i = 1; i < robot_parts.size(); ++i) {
        robot.addBody(robot_parts[i]);
    }

    return robot;
}
