/* Author: Mincheul Kang */

#ifndef ODYSSEY_OCCUPANCY_GRID_H
#define ODYSSEY_OCCUPANCY_GRID_H

#include <ros/ros.h>

#include <odyssey/sphere_representation.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/collision_distance_field/collision_world_hybrid.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>
#include <moveit/robot_state/conversions.h>

#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <gridmap3D/Grid3D.h>
#include <std_msgs/String.h>

namespace odyssey{
    class OccupancyGrid {
    public:
        OccupancyGrid(std::string fixed_frame_id, double resolution, gridmap3D::Grid3D* occ_gridmap,
                    std::vector<std::vector<double> > &box_condition, std::vector<SphereData> &state_spheres,
                    std::string name_sub_pc, std::string name_pub_occ="");
        ~OccupancyGrid();

        void update_occupancy_map(const sensor_msgs::PointCloud2ConstPtr& src_pc);

    protected:
        // Node handle
        ros::NodeHandle nh;
        // Pointcloud subscribers
        message_filters::Subscriber<sensor_msgs::PointCloud2>*  sub_pc_;
        tf::MessageFilter<sensor_msgs::PointCloud2>*            tf_pc_;
        tf::TransformListener                                   tf_listener_;

        // Map publisher
        ros::Publisher                                          pub_occ_;
        // Occupancy grid
        gridmap3D::Grid3D*  occ_gridmap_;

        std::string fixed_frame_id_;
        double max_range_;

        std::vector<std::vector<double> > &box_condition_;
        std::vector<SphereData> &state_spheres_;

        void filter_pointcloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_pointcloud, const tf::StampedTransform& transform, gridmap3D::Pointcloud& dst_pc, gridmap3D::Pointcloud& free_pc);
        void filter_pointcloud(const sensor_msgs::PointCloud2& src_pc, const tf::StampedTransform& transform, gridmap3D::Pointcloud& dst_pc, gridmap3D::Pointcloud& free_pc);
        void down_sampling(const sensor_msgs::PointCloud2& src_pc, pcl::PointCloud<pcl::PointXYZ>& dst_pc);
        void publish_occupied_cells();
    };
}


#endif