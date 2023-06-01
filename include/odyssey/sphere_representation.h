//
// Created by mincheul on 23. 5. 16..
//

#ifndef ODYSSEY_SPHERE_REPRESENTATION_H
#define ODYSSEY_SPHERE_REPRESENTATION_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <eigen3/Eigen/LU>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace odyssey{
    struct SphereData {
        // transform
        // radius
        Eigen::Affine3d T_par_;
        Eigen::Affine3d T_base_;
        uint num_parent_joint_;
        float rad_;
    };

    static inline std::vector<SphereData> fetch_sphere_representation() {
        std::vector<SphereData> sd;

        SphereData s0_0;
        s0_0.num_parent_joint_ = 0;
        s0_0.rad_ = 0.07;
        s0_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s0_0);

        SphereData s0_1;
        s0_1.num_parent_joint_ = 0;
        s0_1.rad_ = 0.09;
        s0_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.05, 0, 0.05));
        sd.push_back(s0_1);

        SphereData s1_0;
        s1_0.num_parent_joint_ = 1;
        s1_0.rad_ = 0.1;
        s1_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s1_0);

        SphereData s1_1;
        s1_1.num_parent_joint_ = 1;
        s1_1.rad_ = 0.09;
        s1_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.1, 0, 0));
        sd.push_back(s1_1);

        SphereData s2_0;
        s2_0.num_parent_joint_ = 2;
        s2_0.rad_ = 0.09;
        s2_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s2_0);

        SphereData s3_0;
        s3_0.num_parent_joint_ = 3;
        s3_0.rad_ = 0.09;
        s3_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s3_0);

        SphereData s3_1;
        s3_1.num_parent_joint_ = 3;
        s3_1.rad_ = 0.075;
        s3_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.08, 0, 0));
        sd.push_back(s3_1);

        SphereData s4_0;
        s4_0.num_parent_joint_ = 4;
        s4_0.rad_ = 0.1;
        s4_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s4_0);

        SphereData s5_0;
        s5_0.num_parent_joint_ = 5;
        s5_0.rad_ = 0.1;
        s5_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, -0.01, 0));
        sd.push_back(s5_0);

        SphereData s6_0;
        s6_0.num_parent_joint_ = 6;
        s6_0.rad_ = 0.1;
        s6_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s6_0);

        SphereData s6_1;
        s6_1.num_parent_joint_ = 6;
        s6_1.rad_ = 0.09;
        s6_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.11, 0, 0));
        sd.push_back(s6_1);

        return sd;
    }

    static inline std::vector<SphereData> iiwa_sphere_representation() {
        std::vector<SphereData> sd;

        SphereData s0_0;
        s0_0.num_parent_joint_ = 0;
        s0_0.rad_ = 0.15;
        s0_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, -0.075));
        sd.push_back(s0_0);

        SphereData s0_1;
        s0_1.num_parent_joint_ = 0;
        s0_1.rad_ = 0.125;
        s0_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.0, -0.04, 0.1));
        sd.push_back(s0_1);

        SphereData s1_0;
        s1_0.num_parent_joint_ = 1;
        s1_0.rad_ = 0.125;
        s1_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s1_0);

        SphereData s1_1;
        s1_1.num_parent_joint_ = 1;
        s1_1.rad_ = 0.1;
        s1_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0.1, 0.02));
        sd.push_back(s1_1);

        SphereData s2_0;
        s2_0.num_parent_joint_ = 2;
        s2_0.rad_ = 0.125;
        s2_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s2_0);

        SphereData s2_1;
        s2_1.num_parent_joint_ = 2;
        s2_1.rad_ = 0.1;
        s2_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0.05, 0.1));
        sd.push_back(s2_1);

        SphereData s3_0;
        s3_0.num_parent_joint_ = 3;
        s3_0.rad_ = 0.125;
        s3_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s3_0);

        SphereData s3_1;
        s3_1.num_parent_joint_ = 3;
        s3_1.rad_ = 0.1;
        s3_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.0, 0.1, 0.05));
        sd.push_back(s3_1);

        SphereData s4_0;
        s4_0.num_parent_joint_ = 4;
        s4_0.rad_ = 0.125;
        s4_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s4_0);

        SphereData s4_1;
        s4_1.num_parent_joint_ = 4;
        s4_1.rad_ = 0.1;
        s4_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0.05, 0.1));
        sd.push_back(s4_1);

        SphereData s5_0;
        s5_0.num_parent_joint_ = 5;
        s5_0.rad_ = 0.125;
        s5_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, 0.05));
        sd.push_back(s5_0);

        SphereData s6_0;
        s6_0.num_parent_joint_ = 6;
        s6_0.rad_ = 0.1;
        s6_0.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0));
        sd.push_back(s6_0);

        SphereData s6_1;
        s6_1.num_parent_joint_ = 6;
        s6_1.rad_ = 0.09;
        s6_1.T_par_ = Eigen::Translation3d(Eigen::Vector3d(0, 0, 0.1));
        sd.push_back(s6_1);

        return sd;
    }
}

#endif