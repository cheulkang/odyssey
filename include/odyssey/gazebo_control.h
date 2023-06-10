//
// Created by mincheul on 4/11/23.
//

#ifndef ODYSSEY_GAZEBO_CONTROL_H
#define ODYSSEY_GAZEBO_CONTROL_H

#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetLinkProperties.h>
#include <gazebo_msgs/GetLinkProperties.h>

#include <iostream>
#include <sstream>
#include <tinyxml.h>

namespace odyssey{
    class GazeboControl {
    public:
        GazeboControl();
        ~GazeboControl();

        bool spawnModel(std::string &name, std::string &model_path, geometry_msgs::Pose &model_pose);
        bool spawnModel(std::string &name, std::ostringstream &model_xml, geometry_msgs::Pose &model_pose);
        bool deleteModel(std::string &name);
        bool setModelState(std::string &name, geometry_msgs::Pose p, geometry_msgs::Twist t);
        bool getModelState(gazebo_msgs::GetModelState &srv_gms);
        bool getLinkProperties(gazebo_msgs::GetLinkProperties &srv_glp);
        bool setLinkProperties(gazebo_msgs::SetLinkProperties &srv_slp);
    private:
        ros::NodeHandle nh_;
        ros::ServiceClient cli_spawn_model_;
        ros::ServiceClient cli_delete_model_;

        ros::ServiceClient cli_set_model_state_;
        ros::ServiceClient cli_get_model_state_;
        ros::ServiceClient cli_set_link_properties_;
        ros::ServiceClient cli_get_link_properties_;
    };
}

#endif