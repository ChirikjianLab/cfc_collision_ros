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
        std::string CONFIG_FILE_PREFIX;
        std::string ENV_TYPE;

        nh_.getParam("config_file_prefix", CONFIG_FILE_PREFIX);
        nh_.getParam("env_type", ENV_TYPE);

        // Read and setup environment config
        ROS_INFO("Loading obstacles...");
        obstacle_ = loadVectorGeometry(CONFIG_FILE_PREFIX + "obstacle_" +
                                       ENV_TYPE + ".csv");

        //        nh_.getParam("obstaclePointCloud", env_config.obstacle_pcl);
        //        for (size_t i = 0; i < obstacle_.size(); ++i) {
        //            pcl::PointCloud<pcl::PointXYZ> cloud;
        //            if (pcl::io::loadPLYFile<pcl::PointXYZ>(
        //                    CONFIG_FILE_PREFIX + "obstacle_pcl_" + ENV_TYPE +
        //                    "_" +
        //                        std::to_string(i) + ".ply",
        //                    cloud) == -1) {
        //                PCL_ERROR("Couldn't read file1 \n");
        //            }

        //            obstacle_pcl_.push_back(cloud);
        //        }
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

        //            obstacle_pcl_pub_.push_back(nh_.advertise<sensor_msgs::PointCloud2>(
        //                "obstacle_pcl_" + std::to_string(i), 1));
    }

    /** \brief Visualize 2D arena and obstacles */
    void visualize() {
        plotShapePoint(obstacle_, obstacle_pub_, 1.0, 0.0, 0.0);
        plotShapeSurf(obstacle_, obstacle_surf_pub_, 1.0, 0.0, 0.0, 1.0);

        //            plotPointCloud(&obstacle_pcl_.at(i),
        //            obstacle_pcl_pub_.at(i));
    }

  private:
    ros::NodeHandle nh_;
    std::string name_;

    std::vector<cfc::SuperQuadrics> obstacle_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacle_pcl_;

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
