/* Author: Mincheul Kang */

#ifndef ODYSSEY_VISUALIZATION_H
#define ODYSSEY_VISUALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <odyssey/sphere_representation.h>
#include <odyssey/kinematics.h>

namespace odyssey
{
    class Visualization
    {
    public:
        Visualization(std::string frame_id);
        virtual ~Visualization(){};

        void visualizeConfiguration(std::vector<std::string> &indices, std::vector<double> &conf);
        void visualizeTrajectory(std::vector<std::string> &indices, std::vector<std::vector<double> > &traj);

        void addLineForVisualization(geometry_msgs::Point p1, geometry_msgs::Point p2, int color);
        void visualizeLine();

        void visualizeFigure(uint32_t shape, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color);
        void visualizeStateSpheres(std::vector<SphereData> &state_spheres);
    private:
        int num_joints_;
        std::string frame_id_;

        ros::Publisher pub_js_;
        ros::Publisher pub_display_path_;
        ros::Publisher pub_marker_;

        sensor_msgs::JointState js_;
        moveit_msgs::DisplayTrajectory display_trajectory_;
        moveit_msgs::RobotTrajectory robot_traj_;
        visualization_msgs::Marker line_list_;

        uint num_figure_;
    };
}

#endif
