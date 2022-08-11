#include "util/Parse2dCsvFile.h"
#include "util/ParsePlanningSettings.h"
#include "util/VisualizationUtils.h"

#include <moveit_msgs/DisplayTrajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <kdl_parser/kdl_parser.hpp>

#include <string>

class RobotStates {
  public:
    RobotStates(const ros::NodeHandle& nh, MultiBodyTree3D robot,
                const std::string urdf_file,
                const std::vector<std::vector<double>>& end_points,
                const std::string state_file)
        : nh_(nh),
          robot_(robot),
          urdf_file_(urdf_file),
          end_points_(end_points),
          state_file_(state_file) {
        joint_pub_ =
            nh_.advertise<sensor_msgs::JointState>("robot_joint_states", 1);
        joint_traj_pub_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(
            "robot_joint_trajectory", 1);

        robot_sq_ = robot_.getBodyShapes();

        robot_sq_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("robot_sq", 1);
        robot_sq_surf_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("robot_sq_surf", 1);

        robot_goal_sq_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("robot_goal_sq", 1);
        robot_goal_sq_surf_pub_ =
            nh_.advertise<visualization_msgs::MarkerArray>("robot_goal_sq_surf",
                                                           1);

        joint_sub_ = nh_.subscribe("joint_states", 1,
                                   &RobotStates::getJointNameCallBack, this);

        sleep(3.0);
    }

    void getJointNameCallBack(const sensor_msgs::JointStatePtr& joint_state) {
        joint_state_ = *joint_state;
    }

    void publishGoalState() {
        states_ = parse2DCsvFile(state_file_);

        // Set and broadcast robot base
        tf::Transform transform;
        transform.setIdentity();
        br_.sendTransform(
            tf::StampedTransform(transform, ros::Time::now(), "world", "base"));

        // Publish robot goal state
        joint_state_.header.stamp = ros::Time::now();
        joint_state_.position = states_.back();
        joint_pub_.publish(joint_state_);

        publishRobotSQ(states_.back(), robot_goal_sq_pub_,
                       robot_goal_sq_surf_pub_);
    }

    void publishJointState() {
        // Publish robot body SQ shapes
        for (auto state : states_) {
            publishRobotSQ(state, robot_sq_pub_, robot_sq_surf_pub_);

            sleep(0.8);
        }
    }

    void publishRobotSQ(const std::vector<double>& state,
                        ros::Publisher robot_point_pub,
                        ros::Publisher robot_surf_pub) {
        // Set robot base to be identity
        const Eigen::Matrix4d g_base = Eigen::Matrix4d::Identity();

        // Set joint values to the robot
        Eigen::VectorXd joint_config(state.size());
        for (size_t i = 0; i < state.size(); ++i) {
            joint_config(i) = state.at(i);
        }

        // Transform the robot using auxiliary
        robot_.robotTF(urdf_file_, &g_base, &joint_config);
        robot_sq_ = robot_.getBodyShapes();

        // Publish point cloud of robot bodies
        plotShapePoint(robot_sq_, robot_point_pub, 1.0, 1.0, 0.0);
        plotShapeSurf(robot_sq_, robot_surf_pub, 1.0, 1.0, 0.0, 1.0);
    }

    void publishRobotJointTrajectory() {
        moveit_msgs::RobotTrajectory robot_traj;
        robot_traj.joint_trajectory.joint_names = joint_state_.name;

        for (auto state : states_) {
            trajectory_msgs::JointTrajectoryPoint joint_traj_point;

            for (auto s : state) {
                joint_traj_point.positions.push_back(s);
            }

            robot_traj.joint_trajectory.points.push_back(joint_traj_point);
        }

        moveit_msgs::DisplayTrajectory display_joint_traj;
        display_joint_traj.trajectory.push_back(robot_traj);
        display_joint_traj.trajectory_start.is_diff = false;
        display_joint_traj.trajectory_start.joint_state = joint_state_;

        joint_traj_pub_.publish(display_joint_traj);
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber joint_sub_;
    ros::Publisher joint_pub_;
    ros::Publisher joint_traj_pub_;

    ros::Publisher robot_sq_pub_;
    ros::Publisher robot_sq_surf_pub_;
    ros::Publisher robot_goal_sq_pub_;
    ros::Publisher robot_goal_sq_surf_pub_;

    tf::TransformBroadcaster br_;

    std::string state_file_;
    std::vector<std::vector<double>> end_points_;

    sensor_msgs::JointState joint_state_;
    std::vector<double> state_;
    std::vector<std::vector<double>> states_;

    MultiBodyTree3D robot_;
    std::string urdf_file_;
    std::vector<cfc::SuperQuadrics> robot_sq_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_visualizer");
    ros::NodeHandle nh;

    // robot state
    std::string CONFIG_FILE_PREFIX;
    std::string RESULT_FILE_PREFIX;
    std::string ENV_TYPE;
    std::string ROBOT_NAME;
    std::string URDF_FILE_PREFIX;

    nh.getParam("config_file_prefix", CONFIG_FILE_PREFIX);
    nh.getParam("result_file_prefix", RESULT_FILE_PREFIX);
    nh.getParam("robot_name", ROBOT_NAME);
    nh.getParam("urdf_file_prefix", URDF_FILE_PREFIX);
    nh.getParam("env_type", ENV_TYPE);

    const std::string config_file =
        CONFIG_FILE_PREFIX + "robot_" + ROBOT_NAME + ".csv";
    const std::string urdf_file = URDF_FILE_PREFIX + ROBOT_NAME + ".urdf";

    MultiBodyTree3D robot = loadRobotMultiBody3D(config_file);

    const std::vector<std::vector<double>>& end_points = parse2DCsvFile(
        CONFIG_FILE_PREFIX + "end_points_" + ROBOT_NAME + ".csv");

    const std::string state_file = RESULT_FILE_PREFIX + ROBOT_NAME + '_' +
                                   ENV_TYPE + "_ompl_smooth_path_3D.csv";

    // Read and set robot joint states
    RobotStates state_pub(nh, robot, urdf_file, end_points, state_file);

    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok()) {
        state_pub.publishGoalState();
        state_pub.publishJointState();
        state_pub.publishRobotJointTrajectory();

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    return 0;
}
