#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

// This is to save on typing
typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;
void cloud_cb (const sensor_msgs::PointCloud2& ros_pc){
    // See http://wiki.ros.org/hydro/Migration for the source of this magic.
    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
    pcl_conversions::toPCL(ros_pc, pcl_pc);
    // Convert point cloud to PCL native point cloud
    point_cloud_t::Ptr input_ptr(new point_cloud_t());
    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);
    // Set up VoxelGrid filter to bin into 10cm grid
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(input_ptr);
    sor.setLeafSize(0.01, 0.01, 0.01);
    // Create output point cloud
    point_cloud_t::Ptr output_ptr(new point_cloud_t());
    // Run filter
    sor.filter(*output_ptr);
    // Now covert output back from PCL native type to ROS
    sensor_msgs::PointCloud2 ros_output;
    pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
    pcl_conversions::fromPCL(pcl_pc, ros_output);
    // Publish the data
    pub.publish(ros_output);
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "pcl_voxel");

    std::string sub_topic_name = "/out_cam/depth/color/points";
    std::string pub_topic_name = "/out_cam/depth/color/down_points";
    if (argc > 1){
        sub_topic_name = argv[1];
        pub_topic_name = argv[2];
    }

    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe(sub_topic_name, 1, cloud_cb);
    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>(pub_topic_name, 1);

    // Spin
    ros::spin ();

    return 0;
}
