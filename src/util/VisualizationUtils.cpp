#include "util/VisualizationUtils.h"

void plotPoints(const std::vector<std::vector<geometry_msgs::Point>> &points,
                const ros::Publisher &publisher, const double r, const double g,
                const double b) {
    const double t_wait = 0.5;

    // Marker points for each object
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        marker_array.markers.at(i).header.frame_id = "world";
        marker_array.markers.at(i).ns = "points";
        marker_array.markers.at(i).action = visualization_msgs::Marker::ADD;
        marker_array.markers.at(i).pose.orientation.w = 1.0;
        marker_array.markers.at(i).id = i;
        marker_array.markers.at(i).type = visualization_msgs::Marker::POINTS;
        marker_array.markers.at(i).scale.x = 0.005;
        marker_array.markers.at(i).scale.y = 0.005;
        marker_array.markers.at(i).scale.z = 0.005;
        marker_array.markers.at(i).color.a = 1.0;
        marker_array.markers.at(i).color.r = r;
        marker_array.markers.at(i).color.g = g;
        marker_array.markers.at(i).color.b = b;

        // Assign points
        marker_array.markers.at(i).points = points.at(i);

        marker_array.markers.at(i).header.stamp = ros::Time::now();
        marker_array.markers.at(i).lifetime = ros::Duration();
    }

    // Publish the points marker array
    publisher.publish(marker_array);
    sleep(t_wait);
}

void plotMesh(const std::vector<EMesh> &mesh_list,
              const ros::Publisher &publisher, const double r, const double g,
              const double b, const double a) {
    const double t_wait = 0.5;

    // Marker points for each object
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(mesh_list.size());

    for (size_t i = 0; i < mesh_list.size(); ++i) {
        marker_array.markers.at(i).header.frame_id = "world";
        marker_array.markers.at(i).ns = "points";
        marker_array.markers.at(i).action = visualization_msgs::Marker::ADD;
        marker_array.markers.at(i).pose.orientation.w = 1.0;
        marker_array.markers.at(i).id = i;
        marker_array.markers.at(i).type =
            visualization_msgs::Marker::TRIANGLE_LIST;
        marker_array.markers.at(i).scale.x = 1;
        marker_array.markers.at(i).scale.y = 1;
        marker_array.markers.at(i).scale.z = 1;
        marker_array.markers.at(i).color.a = a;
        marker_array.markers.at(i).color.r = r;
        marker_array.markers.at(i).color.g = g;
        marker_array.markers.at(i).color.b = b;

        // Assign mesh
        std::vector<geometry_msgs::Point> triangle_list;
        for (auto triangle : mesh_list.at(i).triangles) {
            for (size_t j = 0; j < 3; ++j) {
                geometry_msgs::Point point;
                point.x = mesh_list.at(i).vertices.at(triangle[j])(0);
                point.y = mesh_list.at(i).vertices.at(triangle[j])(1);
                point.z = mesh_list.at(i).vertices.at(triangle[j])(2);

                triangle_list.push_back(point);
            }
        }

        marker_array.markers.at(i).points = triangle_list;

        marker_array.markers.at(i).header.stamp = ros::Time::now();
        marker_array.markers.at(i).lifetime = ros::Duration();
    }

    // Publish the points marker array
    publisher.publish(marker_array);
    sleep(t_wait);
}

std::vector<geometry_msgs::Point> getShapeBoundary(
    const cfc::SuperQuadrics &shape) {
    const int num = 20;
    cfc::ParametricPoints boundary =
        getBoundary3D<cfc::SuperQuadrics>(&shape, num);
    std::vector<geometry_msgs::Point> points;

    points.resize(num + 1);
    for (int i = 0; i < num; ++i) {
        points[i].x = boundary.x.at(i);
        points[i].y = boundary.y.at(i);
        points[i].z = boundary.z.at(i);
    }
    points[num] = points[0];

    return points;
}

void plotShapePoint(const std::vector<cfc::SuperQuadrics> &shapes,
                    const ros::Publisher &publisher, const double r,
                    const double g, const double b) {
    // Boundary surface points
    std::vector<std::vector<geometry_msgs::Point>> points(shapes.size());

    for (size_t i = 0; i < shapes.size(); ++i) {
        points.at(i) = getShapeBoundary(shapes.at(i));
    }

    plotPoints(points, publisher, r, g, b);
}

void plotShapeSurf(const std::vector<cfc::SuperQuadrics> &shapes,
                   const ros::Publisher &publisher, const double r,
                   const double g, const double b, const double a) {
    // Boundary surface meshes
    std::vector<EMesh> mesh(shapes.size());
    const int num = 20;

    for (size_t i = 0; i < shapes.size(); ++i) {
        mesh.at(i) = generateMesh<cfc::SuperQuadrics>(&shapes.at(i), num);
    }

    plotMesh(mesh, publisher, r, g, b, a);
}

void plotPlan(MultiBodyTree3D *robot,
              const trajectory_msgs::MultiDOFJointTrajectoryPoint &path,
              const ros::Publisher &path_pub) {
    const double t_wait = 0.5;

    for (size_t i = 0; i < path.transforms.size(); i++) {
        plotRobot(robot, &path.transforms.at(i), path_pub);

        sleep(t_wait);
    }
}

void plotRobot(MultiBodyTree3D *robot,
               const geometry_msgs::Transform *transform,
               const ros::Publisher &pub) {
    transformRobot(robot, transform);
    auto robotBody = robot->getBodyShapes();

    plotShapePoint(robotBody, pub, 1.0, 1.0, 0.0);
    plotShapeSurf(robotBody, pub, 1.0, 1.0, 0.0, 0.5);
}

void transformRobot(MultiBodyTree3D *robot,
                    const geometry_msgs::Transform *transform) {
    // Set rotation
    Eigen::Quaterniond quat(transform->rotation.x, transform->rotation.y,
                            transform->rotation.z, transform->rotation.w);

    // Set translation
    Eigen::Matrix4d robot_tf = Eigen::Matrix4d::Identity();
    robot_tf.topRightCorner(3, 1) =
        Eigen::Vector3d(transform->translation.x, transform->translation.y,
                        transform->translation.z);
    robot_tf.topLeftCorner(3, 3) = quat.toRotationMatrix();

    // Transform robot
    robot->robotTF(robot_tf);
}

void plotPointCloud(const pcl::PointCloud<pcl::PointXYZ> *cloud,
                    const ros::Publisher &pcl_pub) {
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);

    output.header.frame_id = "/world_frame";

    pcl_pub.publish(output);
}

geometry_msgs::Transform poseToGeomMsgs(const std::vector<double> &pose) {
    // From vector of double to geometric message
    geometry_msgs::Transform transform;
    transform.translation.x = pose[0];
    transform.translation.y = pose[1];
    transform.translation.z = pose[2];

    tf2::Quaternion quat;
    quat.setW(pose[3]);
    quat.setW(pose[4]);
    quat.setW(pose[5]);
    quat.setW(pose[6]);

    transform.rotation = tf2::toMsg(quat);

    // Show the path
    ROS_INFO("(%f, %f, %f)", pose[0], pose[1], pose[2]);

    return transform;
}
