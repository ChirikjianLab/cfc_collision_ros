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

#include "planning_app/ompl_planner.h"

ompl_planner::ompl_planner(const MultiBodyTree3D &robot,
                           const std::string urdfFile,
                           const std::vector<cfc::SuperQuadrics> &obstacle,
                           const planning_request &req)
    : robot_(robot), urdfFile_(urdfFile), obstacle_(obstacle), req_(req) {
  // Parse URDF file and construct KDL tree
  kdl_ = new ParseURDF(urdfFile_);
  numJoint_ = kdl_->getKDLTree().getNrOfJoints();
}

ompl_planner::~ompl_planner() {}

void ompl_planner::setup() {
  // Setup state space
  setStateSpace();

  // Set collision checker
  ss_->setStateValidityChecker(
      [this](const ob::State *state) { return isStateValid(state); });
  ss_->getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  // Set planner and sampler
  setPlanner();
  setValidStateSampler();

  ss_->setup();

  OMPL_INFORM("Finished setting up planner!");
}

bool ompl_planner::plan() {
  if (!ss_) {
    return false;
  }

  // Set start and goal states for planning
  setStartAndGoalState();

  // Path planning
  OMPL_INFORM("Planning...");

  try {
    res_.isSolved = ss_->solve(req_.maxTimeInSec);

    // Get solving time
    res_.totalTime = ss_->getLastPlanComputationTime();

  } catch (ompl::Exception &ex) {
    std::stringstream es;
    es << ex.what() << std::endl;
    OMPL_WARN(es.str().c_str());
  }

  // Get planning results
  OMPL_INFORM("Retrieving solution info...");
  getSolution();

  return true;
}

void ompl_planner::getSolution() {
  if (res_.isSolved) {
    try {
      // Simplify and get solution path
      ss_->simplifySolution();
      auto path = ss_->getSolutionPath().getStates();

      // Number of nodes in solved path
      res_.lengthPath = ss_->getSolutionPath().getStates().size();

      // Save path
      for (auto state : path) {
        res_.path.push_back(setVectorFromState(state));
      }

      // Interpolate to get smoothed path
      OMPL_INFORM("Smoothing solved path...");
      ss_->getSolutionPath().interpolate(50);
      auto path_smooth = ss_->getSolutionPath().getStates();

      for (auto state : path_smooth) {
        res_.path_smooth.push_back(setVectorFromState(state));
      }
    } catch (ompl::Exception &ex) {
    }
  }

  // Retrieve planning data
  ob::PlannerData pd(ss_->getSpaceInformation());
  ss_->getPlannerData(pd);

  // Number of vertices, edges
  res_.numGraphVertex = pd.numVertices();
  res_.numGraphEdges = pd.numEdges();

  // Save vertices and edges
  const ob::State *state;
  res_.vertex.clear();
  for (unsigned int i = 0; i < res_.numValidStates; ++i) {
    state = pd.getVertex(i).getState()->as<ob::State>();
    res_.vertex.push_back(setVectorFromState(state));
  }

  std::vector<std::vector<unsigned int>> edgeInfo(res_.numValidStates);
  res_.edge.clear();
  for (unsigned int i = 0; i < res_.numValidStates; ++i) {
    pd.getEdges(i, edgeInfo.at(i));
    for (auto edgeI : edgeInfo.at(i))
      res_.edge.push_back(std::make_pair(i, edgeI));
  }

  // Total number of checked and valid motions
  res_.numCollisionChecks =
      pd.getSpaceInformation()->getMotionValidator()->getCheckedMotionCount();
  res_.numValidStates =
      pd.getSpaceInformation()->getMotionValidator()->getValidMotionCount();
}

void ompl_planner::setStateSpace() {
  std::vector<urdf::LinkSharedPtr> links;
  kdl_->getURDFModel().getLinks(links);
  std::vector<urdf::JointSharedPtr> joints;
  for (auto link : links) {
    if (link->child_links.size() != 0 &&
        link->child_joints.front()->type != urdf::Joint::FIXED) {
      joints.push_back(link->child_joints.front());
    }
  }

  auto space(std::make_shared<ob::RealVectorStateSpace>(numJoint_));

  OMPL_INFORM("Enforcing bounds");
  ob::RealVectorBounds bounds(numJoint_);
  for (size_t i = 0; i < numJoint_; ++i) {
    bounds.setLow(i, joints.at(i)->limits->lower);
    bounds.setHigh(i, joints.at(i)->limits->upper);
  }
  space->setBounds(bounds);

  OMPL_INFORM("Using simple setup");
  ss_ = std::make_shared<og::SimpleSetup>(space);
}

void ompl_planner::setPlanner() {
  // Set planner
  if (req_.plannerId == "PRM") {
    ss_->setPlanner(std::make_shared<og::PRM>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "LazyPRM") {
    ss_->setPlanner(std::make_shared<og::LazyPRM>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "RRT") {
    ss_->setPlanner(std::make_shared<og::RRT>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "RRTConnect") {
    ss_->setPlanner(
        std::make_shared<og::RRTConnect>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "EST") {
    ss_->setPlanner(std::make_shared<og::EST>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "SBL") {
    ss_->setPlanner(std::make_shared<og::SBL>(ss_->getSpaceInformation()));
  } else if (req_.plannerId == "KPIECE1") {
    ss_->setPlanner(std::make_shared<og::KPIECE1>(ss_->getSpaceInformation()));
  } else {
    OMPL_ERROR("Invalid type of planner!");
  }
}

// Set the valid state sampler
void ompl_planner::setValidStateSampler() {
  if (req_.validSamplerId == "Uniform") {
    // Uniform sampler
    OMPL_INFORM("Using Uniform valid state sampler");

    ss_->getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
          return std::make_shared<ob::UniformValidStateSampler>(si);
        });
  } else if (req_.validSamplerId == "Gaussian") {
    // Gaussian sampler
    OMPL_INFORM("Using Gaussian valid state sampler");

    ss_->getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
          return std::make_shared<ob::GaussianValidStateSampler>(si);
        });
  } else if (req_.validSamplerId == "OB") {
    // Obstacle-based sampler
    OMPL_INFORM("Using Obstacle-based valid state sampler");

    ss_->getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
          return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
        });
  } else if (req_.validSamplerId == "MC") {
    // Maximum-clearance sampler
    OMPL_INFORM("Using Max-clearance valid state sampler");

    ss_->getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
          auto vss =
              std::make_shared<ob::MaximizeClearanceValidStateSampler>(si);
          vss->setNrImproveAttempts(5);
          return vss;
        });
  } else if (req_.validSamplerId == "Bridge") {
    // Bridge-test sampler
    OMPL_INFORM("Using Bridge-test valid state sampler");

    ss_->getSpaceInformation()->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation *si) -> ob::ValidStateSamplerPtr {
          return std::make_shared<ob::BridgeTestValidStateSampler>(si);
        });
  } else {
    OMPL_ERROR("Invalid type of valid state sampler!");
  }
}

void ompl_planner::setStartAndGoalState() {
  OMPL_INFORM("Setting start and goal configurations");

  ob::ScopedState<ob::RealVectorStateSpace> startState(ss_->getStateSpace());
  setStateFromVector(&req_.start, &startState);
  startState.enforceBounds();

  ob::ScopedState<ob::RealVectorStateSpace> goalState(ss_->getStateSpace());
  setStateFromVector(&req_.goal, &goalState);
  goalState.enforceBounds();

  ss_->setStartAndGoalStates(startState, goalState);
}

bool ompl_planner::isStateValid(const ob::State *state) const {
  bool isCollide = false;
  checkCollision(transformRobot(state), obstacle_, &isCollide);

  return !isCollide;
}

// Get pose info and transform the robot
MultiBodyTree3D ompl_planner::transformRobot(const ob::State *state) const {
  const std::vector<double> stateVar = setVectorFromState(state);

  // Set robot base to be identity
  Eigen::Matrix4d gBase = Eigen::Matrix4d::Identity();

  // Set joint values to the robot
  Eigen::VectorXd jointConfig(numJoint_);
  for (uint i = 0; i < numJoint_; ++i) {
    jointConfig(i) = stateVar.at(i);
  }

  // Transform the robot using auxiliary
  MultiBodyTree3D robotAux = robot_;
  robotAux.robotTF(*kdl_, &gBase, &jointConfig);

  return robotAux;
}

void ompl_planner::saveVertexEdgeInfo(const std::string filenamePrefix) {
  ob::PlannerData pd(ss_->getSpaceInformation());

  // Write the output to .csv files
  std::ofstream fileState;

  fileState.open(filenamePrefix + "state_3D.csv");
  for (auto state : res_.vertex) {
    for (size_t j = 0; j < state.size(); ++j) {
      fileState << state[j];
      if (j == state.size() - 1) {
        fileState << '\n';
      } else {
        fileState << ',';
      }
    }
  }
  fileState.close();

  std::ofstream fileEdge;
  fileEdge.open(filenamePrefix + "edge_3D.csv");
  for (auto edge : res_.edge) {
    fileEdge << edge.first << "," << edge.second << "\n";
  }
  fileEdge.close();
}

void ompl_planner::savePathInfo(const std::string filenamePrefix) {
  std::vector<double> state;

  std::ofstream fileTraj;
  fileTraj.open(filenamePrefix + "path_3D.csv");
  for (size_t i = 0; i < res_.path.size(); ++i) {
    state = res_.path.at(i);

    for (size_t j = 0; j < state.size(); ++j) {
      fileTraj << state.at(j);
      if (j == state.size() - 1) {
        fileTraj << '\n';
      } else {
        fileTraj << ',';
      }
    }
  }
  fileTraj.close();

  // Smoothed path
  std::ofstream fileTrajSmoothed;
  fileTrajSmoothed.open(filenamePrefix + "smooth_path_3D.csv");
  for (size_t i = 0; i < res_.path_smooth.size(); ++i) {
    state = res_.path_smooth.at(i);

    for (size_t j = 0; j < state.size(); ++j) {
      fileTrajSmoothed << state.at(j);
      if (j == state.size() - 1) {
        fileTrajSmoothed << '\n';
      } else {
        fileTrajSmoothed << ',';
      }
    }
  }
  fileTrajSmoothed.close();
}

void ompl_planner::setStateFromVector(
    const std::vector<double> *stateVariables,
    ob::ScopedState<ob::RealVectorStateSpace> *state) const {
  ob::ScopedState<ob::RealVectorStateSpace> stateTemp(ss_->getStateSpace());

  for (size_t i = 0; i < stateVariables->size(); ++i) {
    stateTemp.get()->values[i] = stateVariables->at(i);
  }

  stateTemp >> *state;
}

std::vector<double>
ompl_planner::setVectorFromState(const ob::State *state) const {
  std::vector<double> stateVariables(numJoint_, 0.0);

  // Store state in a vector
  for (size_t i = 0; i < numJoint_; ++i) {
    stateVariables[i] =
        state->as<ob::RealVectorStateSpace::StateType>()->values[i];
  }

  return stateVariables;
}
