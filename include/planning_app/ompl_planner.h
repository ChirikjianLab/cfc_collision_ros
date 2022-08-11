#pragma once

#include "collision_common.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/config.h"
#include "ompl/geometric/SimpleSetup.h"
#include "ompl/util/PPM.h"

#include "ompl/base/samplers/BridgeTestValidStateSampler.h"
#include "ompl/base/samplers/GaussianValidStateSampler.h"
#include "ompl/base/samplers/MaximizeClearanceValidStateSampler.h"
#include "ompl/base/samplers/ObstacleBasedValidStateSampler.h"
#include "ompl/base/samplers/UniformValidStateSampler.h"

#include "ompl/geometric/planners/est/EST.h"
#include "ompl/geometric/planners/kpiece/KPIECE1.h"
#include "ompl/geometric/planners/prm/LazyPRM.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/geometric/planners/rrt/RRT.h"
#include "ompl/geometric/planners/rrt/RRTConnect.h"
#include "ompl/geometric/planners/sbl/SBL.h"

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include <math.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * \brief Planning request
 *
 * \param lowBound, highBound lower and upper bounds for state space
 *
 * \param plannerId set the planner
 *  "PRM", "LazyPRM", "RRT", "RRTConnect", "EST", "SBL", "KPIECE1"
 *
 * \param validSamplerId
 *  "Uniform": uniform valid state sampler
 *  "Gaussian": Gaussian valid state sampler
 *  "OB": obstacle-based valid state sampler
 *  "MC": maximum-clearance valid state sampler
 *  "Bridge": bridge-test valid state sampler
 *
 * \param start, goal start and goal poses
 *
 * \param maxTimeInSec maximum planning time in seconds
 */
struct planning_request {
    std::vector<double> lowBound;
    std::vector<double> highBound;

    std::string plannerId = "RRTConnect";
    std::string validSamplerId = "Uniform";

    std::vector<double> start;
    std::vector<double> goal;

    double maxTimeInSec = 60.0;
};

struct planning_result {
    bool isSolved = false;
    double totalTime = 0.0;

    unsigned int numCollisionChecks = 0;
    unsigned int numValidStates = 0;
    unsigned int numGraphVertex = 0;
    unsigned int numGraphEdges = 0;
    size_t lengthPath = 0;

    std::vector<std::vector<double>> vertex;
    std::vector<std::pair<int, int>> edge;
    std::vector<std::vector<double>> path;
    std::vector<std::vector<double>> path_smooth;
};

class ompl_planner {
  public:
    ompl_planner(const MultiBodyTree3D& robot, const std::string urdfFile,
                 const std::vector<cfc::SuperQuadrics>& obstacle,
                 const planning_request& req);

    virtual ~ompl_planner();

  public:
    /** \brief Getter function for solved and interpolated path */
    std::vector<std::vector<double>> getSolutionPath() const {
        return res_.path;
    }

    std::vector<std::vector<double>> getSmoothedPath() const {
        return res_.path_smooth;
    }

    /** \brief Indicator of solution */
    bool isSolved() const { return res_.isSolved; }

    /** \brief Getter function of total planning time */
    double getPlanningTime() const { return res_.totalTime; }

    /** \brief Get the number of total collision checks, including both sampling
     * and connecting processes */
    unsigned int getNumCollisionChecks() const {
        return res_.numCollisionChecks;
    }

    /** \brief Get the number of valid states */
    unsigned int getNumValidStates() const { return res_.numValidStates; }

    /** \brief Get the number of valid vertices in graph */
    unsigned int getNumVertex() const { return res_.numGraphVertex; }

    /** \brief Get the number of valid edges connecting two milestones */
    unsigned int getNumEdges() const { return res_.numGraphEdges; }

    /** \brief Get the length of the solved path */
    size_t getPathLength() const { return res_.lengthPath; }

    /** \brief Get all the milestones */
    std::vector<std::vector<double>> getVertices() const { return res_.vertex; }

    /** \brief Get all the valid connection pairs */
    std::vector<std::pair<int, int>> getEdges() const { return res_.edge; }

    /** \brief Set up parameters for OMPL */
    void setup();

    /** \brief Start to plan and retrieve results */
    bool plan();

    /** \brief Store planning results to .csv file */
    void saveVertexEdgeInfo(const std::string filename_prefix);
    void savePathInfo(const std::string filename_prefix);

  protected:
    void getSolution();

    void setStateSpace();
    void setPlanner();
    void setValidStateSampler();
    void setStartAndGoalState();

    // Collision detection module
    bool isStateValid(const ob::State* state) const;
    MultiBodyTree3D transformRobot(const ob::State* state) const;

    void setStateFromVector(
        const std::vector<double>* stateVariables,
        ob::ScopedState<ob::RealVectorStateSpace>* state) const;
    std::vector<double> setVectorFromState(const ob::State* state) const;

    // Variables
  protected:
    og::SimpleSetupPtr ss_;

    // Robot geometry and URDF parser
    const MultiBodyTree3D& robot_;
    const std::string urdfFile_;
    ParseURDF* kdl_;
    int numJoint_;

    // Obstacle geometry
    const std::vector<cfc::SuperQuadrics>& obstacle_;

    // Planning request
    planning_request req_;

    // Planning results
    planning_result res_;
};