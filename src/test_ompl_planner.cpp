#include "planning_app/ompl_planner.h"
#include "util/ParsePlanningSettings.h"

#include "ros/publisher.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"

using namespace std;

trajectory_msgs::JointTrajectory load_trajectory(
    const std::vector<std::vector<double>>& result) {
    trajectory_msgs::JointTrajectory traj;

    for (size_t i = 0; i < result.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        traj_point.positions = result.at(i);

        traj.points.push_back(traj_point);
    }

    return traj;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ompl_planner");
    ros::NodeHandle nh;

    ROS_INFO("===========================================");
    ROS_INFO("Motion planning using OMPL + CFC-FP checker");
    ROS_INFO("===========================================");

    /**
     * \brief Load input parameters
     * \param Planner ID: PRM, LazyPRM, RRT, RRTconnect, EST, SBL, KPIECE
     * \param Sampler ID: Uniform, Gaussian, OB, MC, Bridge
     */
    int N = 1;
    string ID_PLANNER = "RRTConnect";
    string ID_SAMPLER = "Uniform";
    string ROBOT_NAME;
    double MAX_PLAN_TIME = 60.0;
    string CONFIG_FILE_PREFIX;
    string ENV_TYPE;
    string URDF_FILE_PREFIX;
    string RESULT_FILE_PREFIX;

    nh.getParam("num_trials", N);
    nh.getParam("planner_id", ID_PLANNER);
    nh.getParam("sampler_id", ID_SAMPLER);
    nh.getParam("robot_name", ROBOT_NAME);
    nh.getParam("max_plan_time", MAX_PLAN_TIME);
    nh.getParam("config_file_prefix", CONFIG_FILE_PREFIX);
    nh.getParam("env_type", ENV_TYPE);
    nh.getParam("urdf_file_prefix", URDF_FILE_PREFIX);
    nh.getParam("result_file_prefix", RESULT_FILE_PREFIX);

    // Read and setup environment config
    const vector<cfc::SuperQuadrics>& obs = loadVectorGeometry(
        CONFIG_FILE_PREFIX + "obstacle_" + ENV_TYPE + ".csv");

    // Read end points config file
    auto end_points = parse2DCsvFile(CONFIG_FILE_PREFIX + "end_points_" +
                                     ROBOT_NAME + ".csv");

    // Setup robot
    const std::string configFile =
        CONFIG_FILE_PREFIX + "robot_" + ROBOT_NAME + ".csv";
    const std::string urdfFile = URDF_FILE_PREFIX + ROBOT_NAME + ".urdf";

    MultiBodyTree3D robot = loadRobotMultiBody3D(configFile);

    // Only PRM use different samplers
    if (ID_PLANNER != "PRM" && ID_SAMPLER != "Uniform") {
        ID_SAMPLER = "Uniform";
    }

    // Set up planning request
    planning_request req;
    req.plannerId = ID_PLANNER;
    req.validSamplerId = ID_SAMPLER;
    req.start = end_points.at(0);
    req.goal = end_points.at(1);
    req.maxTimeInSec = MAX_PLAN_TIME;

    // Initiate result files
    const string RESULT_FILENAME_PREFIX =
        RESULT_FILE_PREFIX + ROBOT_NAME + "_" + ENV_TYPE + "_ompl_";

    // Path initial: only contains start and goal
    std::ofstream fileTrajSmoothed;
    fileTrajSmoothed.open(RESULT_FILENAME_PREFIX + "smooth_path_3D.csv");
    for (auto state : end_points) {
        for (size_t j = 0; j < state.size(); ++j) {
            fileTrajSmoothed << state[j];
            if (j == state.size() - 1) {
                fileTrajSmoothed << '\n';
            } else {
                fileTrajSmoothed << ',';
            }
        }
    }
    fileTrajSmoothed.close();

    // Planning info: planner, sampler, status, planning time, etc
    std::ofstream outfile;
    outfile.open(RESULT_FILENAME_PREFIX + "time_ompl_3D.csv");
    outfile << "PLANNER" << ',' << "SAMPLER" << ',' << "SUCCESS" << ','
            << "TOTAL_TIME" << ',' << "GRAPH_NODES" << ',' << "GRAPH_EDGES"
            << ',' << "PATH_CONFIG" << ',' << "CHECKED_NODES" << ','
            << "VALID_NODES" << endl;

    ROS_INFO("Start testing...");
    ROS_INFO_STREAM("Planner: " << ID_PLANNER);
    ROS_INFO_STREAM("Sampler: " << ID_SAMPLER);
    ROS_INFO_STREAM("Max planning time: " << MAX_PLAN_TIME << " seconds");

    // Initiate planner
    ompl_planner tester(robot, urdfFile, obs, req);
    tester.setup();

    for (int i = 0; i < N; ++i) {
        ROS_INFO_STREAM("Num of trials: " << i + 1);

        // OMPL planning
        tester.plan();
        ROS_INFO("Finished planning!");

        // Store results
        ROS_INFO("Start storing planning result...");
        outfile << ID_PLANNER << ',' << ID_SAMPLER << ',' << tester.isSolved()
                << ',' << tester.getPlanningTime() << ','
                << tester.getNumVertex() << "," << tester.getNumEdges() << ","
                << tester.getPathLength() << ","
                << tester.getNumCollisionChecks() << ','
                << tester.getNumValidStates() << endl;

        if (tester.isSolved()) {
            tester.saveVertexEdgeInfo(RESULT_FILENAME_PREFIX);
            tester.savePathInfo(RESULT_FILENAME_PREFIX);
        }

        ROS_INFO("============================================");
    }
    outfile.close();

    // Publish planned joint trajectory
    ros::Publisher joint_traj_pub =
        nh.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 1);
    ros::Publisher joint_traj_smooth_pub =
        nh.advertise<trajectory_msgs::JointTrajectory>(
            "joint_trajectory_smooth", 1);

    ROS_INFO("Publishing solved joint trajectory...");

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        auto traj = load_trajectory(tester.getSolutionPath());
        joint_traj_pub.publish(traj);

        auto traj_smooth = load_trajectory(tester.getSmoothedPath());
        joint_traj_smooth_pub.publish(traj_smooth);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    ros::shutdown();
    return 0;
}
