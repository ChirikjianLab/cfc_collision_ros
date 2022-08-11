#pragma once

#include "geometry/SuperQuadrics.h"
#include "util/MultiBodyTree3D.h"
#include "util/Parse2dCsvFile.h"

/** \brief loadVectorGeometry Load vector of 3D superquadrics*/
std::vector<cfc::SuperQuadrics> loadVectorGeometry(
    const std::string config_file);

/** \brief loadRobotMultiBody3D Load multi-body tree in 3D */
MultiBodyTree3D loadRobotMultiBody3D(const std::string robot_config_file);

/** \brief loadRobotMultiBody3D Load multi-body tree with URDF in 3D */
MultiBodyTree3D loadRobotMultiBody3D(const std::string robot_config_file,
                                     const std::string robot_urdf_file);
