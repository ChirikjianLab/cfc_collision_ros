#pragma once

#include "geometry/SuperQuadrics.h"
#include "util/ParseURDF.h"

#include <eigen3/Eigen/Geometry>

class MultiBodyTree3D {
  public:
    MultiBodyTree3D(cfc::SuperQuadrics base);

  public:
    cfc::SuperQuadrics getBase() const { return base_; }
    std::vector<cfc::SuperQuadrics> getLinks() const { return link_; }
    std::vector<cfc::SuperQuadrics> getBodyShapes() const {
        std::vector<cfc::SuperQuadrics> body;

        body.push_back(base_);
        for (auto link : link_) {
            body.push_back(link);
        }

        return body;
    }

    double getNumLinks() const { return numLinks_; }
    std::vector<Eigen::Matrix4d> getTF() const { return tf_; }

    void addBody(cfc::SuperQuadrics link);
    void addBody(cfc::SuperQuadrics link, const urdf::Model& urdfModel);

    void robotTF(Eigen::Matrix4d tf);
    void robotTF(const std::string urdfFile, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);
    void robotTF(ParseURDF kdl, const Eigen::Matrix4d* gBase,
                 const Eigen::VectorXd* jointConfig);

  public:
    cfc::SuperQuadrics base_;
    double numLinks_ = 0;
    std::vector<cfc::SuperQuadrics> link_;
    std::vector<Eigen::Matrix4d> tf_;
};
