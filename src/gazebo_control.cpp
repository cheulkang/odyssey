//
// Created by mincheul on 4/11/23.
//

#include <odyssey/gazebo_control.h>

namespace odyssey{
    GazeboControl::GazeboControl() {
        cli_set_model_state_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        cli_spawn_model_ = nh_.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
        cli_delete_model_ = nh_.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

        cli_get_model_state_ = nh_.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
        cli_get_link_properties_ = nh_.serviceClient<gazebo_msgs::GetLinkProperties>("/gazebo/get_link_properties");
        cli_set_link_properties_ = nh_.serviceClient<gazebo_msgs::SetLinkProperties>("/gazebo/set_link_properties");

        ros::Duration(1.0).sleep();
    }

    GazeboControl::~GazeboControl() {
    }

    // model control
    bool GazeboControl::spawnModel(std::string &name, std::string &model_path, geometry_msgs::Pose &model_pose){
        gazebo_msgs::SpawnModel srv_sm;

        TiXmlDocument xml_in(model_path);
        xml_in.LoadFile();
        std::ostringstream stream;
        stream << xml_in;

        srv_sm.request.model_name.assign(name);
        srv_sm.request.model_xml = stream.str(); // load xml file
        srv_sm.request.initial_pose = model_pose;

        if(cli_spawn_model_.call(srv_sm)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::spawnModel(std::string &name, std::ostringstream &model_xml, geometry_msgs::Pose &model_pose){
        gazebo_msgs::SpawnModel srv_sm;

        srv_sm.request.model_name.assign(name);
        srv_sm.request.model_xml = model_xml.str(); // load xml file
        srv_sm.request.initial_pose = model_pose;

        if(cli_spawn_model_.call(srv_sm)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::deleteModel(std::string &name){
        gazebo_msgs::DeleteModel srv_dm;
        srv_dm.request.model_name.assign(name);

        if(cli_delete_model_.call(srv_dm)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::setModelState(std::string &name, geometry_msgs::Pose p, geometry_msgs::Twist t){
        gazebo_msgs::SetModelState srv_sms;
        gazebo_msgs::ModelState ms;

        ms.model_name.assign(name);
        ms.pose = p;
        ms.twist = t;
        ms.reference_frame = "";

        srv_sms.request.model_state = ms;

        if(cli_set_model_state_.call(srv_sms)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::getModelState(gazebo_msgs::GetModelState &srv_gms){
        if(cli_get_model_state_.call(srv_gms)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::setLinkProperties(gazebo_msgs::SetLinkProperties &srv_slp){
        if(cli_set_link_properties_.call(srv_slp)){
            return true;
        }
        else{
            return false;
        }
    }

    bool GazeboControl::getLinkProperties(gazebo_msgs::GetLinkProperties &srv_glp){
        if(cli_get_link_properties_.call(srv_glp)){
            return true;
        }
        else{
            return false;
        }
    }
}

