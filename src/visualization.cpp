/* Author: Mincheul Kang */

#include <odyssey/visualization.h>

namespace odyssey
{
    Visualization::Visualization(std::string frame_id) {
        ros::NodeHandle node_handle("~");
        pub_js_ = node_handle.advertise<sensor_msgs::JointState>("/joint_states", 10);
        pub_display_path_ = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
        pub_marker_ = node_handle.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

        frame_id_ = frame_id;
        num_figure_ = 0;

        line_list_.header.frame_id = frame_id;
        line_list_.header.stamp = ros::Time::now();
        line_list_.ns = "odyssey";
        line_list_.action = visualization_msgs::Marker::ADD;
        line_list_.pose.orientation.w = 1.0;
        line_list_.id = 2;
        line_list_.type = visualization_msgs::Marker::LINE_LIST;
        line_list_.scale.x = 0.01;
        line_list_.scale.y = 0.01;
        line_list_.color.r = 1.0;
        line_list_.color.a = 0.5;
        line_list_.colors.clear();
    }

    void Visualization::visualizeFigure(uint32_t shape, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale){
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = frame_id_;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "basic_shapes";
        marker.id = num_figure_++;

        marker.type = shape;
//        visualization_msgs::Marker::CYLINDER
//        visualization_msgs::Marker::ARROW
//        visualization_msgs::Marker::CYLINDER
//        visualization_msgs::Marker::CUBE

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose = pose;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale = scale;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        for (int i = 0; i < 5; i++)
            pub_marker_.publish(marker);

        ros::Duration(0.1).sleep();
    }

    void Visualization::visualizeConfiguration(std::vector<std::string> &indices, std::vector<double> &conf) {
        js_.position.clear();
        js_.name.clear();
        for(int j = 0; j < indices.size(); j++){
            js_.name.push_back(indices[j]);
            js_.position.push_back(conf[j]);
        }
        js_.header.stamp = ros::Time::now();
        pub_js_.publish(js_);
        ros::Duration(0.1).sleep();
    }

    void Visualization::visualizeTrajectory(std::vector<std::string> &indices, std::vector<std::vector<double> > &traj) {
        display_trajectory_.trajectory.clear();

        robot_traj_.joint_trajectory.joint_names = indices;
        robot_traj_.joint_trajectory.header.stamp = ros::Time::now();
        robot_traj_.joint_trajectory.points.clear();
        robot_traj_.joint_trajectory.points.resize(traj.size());

        for (uint i = 0; i < robot_traj_.joint_trajectory.points.size(); i++) {
            robot_traj_.joint_trajectory.points[i].positions.resize(indices.size());
            for(uint j = 0; j < indices.size(); j++){
                robot_traj_.joint_trajectory.points[i].positions[j] = traj[i][j];
            }
        }
        display_trajectory_.trajectory.push_back(robot_traj_);
        pub_display_path_.publish(display_trajectory_);
        ros::Duration(1.0).sleep();
    }

    void Visualization::addLineForVisualization(geometry_msgs::Point p1, geometry_msgs::Point p2, int color) {
        // 0: red, 1: green, 2: blue
        std_msgs::ColorRGBA lineColor;
        lineColor.a = 0.7;

        if(color == 0){
            lineColor.r = 1.0;
        }
        else if(color == 1){
            lineColor.g = 1.0;
        }
        else if(color == 2){
            lineColor.b = 1.0;
        }
        else{
            lineColor.r = 1.0;
            lineColor.g = 1.0;
            lineColor.b = 1.0;
        }

        line_list_.points.push_back(p1);
        line_list_.colors.push_back(lineColor);

        line_list_.points.push_back(p2);
        line_list_.colors.push_back(lineColor);
    }

    void Visualization::visualizeLine() {
        for(uint i = 0; i < 10; i++){
            pub_marker_.publish(line_list_);
        }
    }

    void Visualization::visualizeStateSpheres(std::vector<odyssey::SphereData> &state_spheres){
        for(uint i = 0; i < state_spheres.size(); i++){
            visualization_msgs::Marker marker;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();

            marker.ns = "state_spheres";
            marker.id = i;

            marker.type = visualization_msgs::Marker::SPHERE;

            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
            marker.action = visualization_msgs::Marker::ADD;

            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = state_spheres[i].T_base_.translation().x();
            marker.pose.position.y = state_spheres[i].T_base_.translation().y();
            marker.pose.position.z = state_spheres[i].T_base_.translation().z();
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = state_spheres[i].rad_ * 2;
            marker.scale.y = state_spheres[i].rad_ * 2;
            marker.scale.z = state_spheres[i].rad_ * 2;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 1.0f;
            marker.color.a = 0.5f;

            marker.lifetime = ros::Duration();

            for (int i = 0; i < 5; i++)
                pub_marker_.publish(marker);

            ros::Duration(0.1).sleep();
        }
    }
}
