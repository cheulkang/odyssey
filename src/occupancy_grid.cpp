#include <odyssey/occupancy_grid.h>

namespace odyssey
{
    OccupancyGrid::OccupancyGrid(std::string fixed_frame_id, double max_range, gridmap3D::Grid3D* occ_gridmap,
                     std::vector<std::vector<double> > &box_condition, std::vector<std::vector<double> > &spheres,
                     std::string name_sub_pc, std::string name_pub_occ):
            nh(),
            fixed_frame_id_(fixed_frame_id),
            max_range_(max_range),
            occ_gridmap_(occ_gridmap),
            box_condition_(box_condition),
            spheres_(spheres)
    {
        sub_pc_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh, name_sub_pc, 1);
        tf_pc_ = new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub_pc_, tf_listener_, fixed_frame_id, 1);
        tf_pc_->registerCallback(boost::bind(&OccupancyGrid::update_occupancy_map, this, _1));
        if(name_pub_occ != "")
            pub_occ_ = nh.advertise<sensor_msgs::PointCloud2>(name_pub_occ, 1);
    }

    OccupancyGrid::~OccupancyGrid() {
        delete sub_pc_;
        delete tf_pc_;
    }

    void OccupancyGrid::update_occupancy_map(const sensor_msgs::PointCloud2ConstPtr& src_pc) {
        // Pose of the sensor frame
        tf::StampedTransform sensorToWorldTf;
        try{
            tf_listener_.lookupTransform(fixed_frame_id_, src_pc->header.frame_id, src_pc->header.stamp, sensorToWorldTf);
        }
        catch(tf::TransformException& e){
            ROS_ERROR_STREAM("Transform error of sensor data: " << e.what() << ", quitting callback");
            return;
        }

        gridmap3D::point3d origin(sensorToWorldTf.getOrigin().x(), sensorToWorldTf.getOrigin().y(), sensorToWorldTf.getOrigin().z());
        pcl::PointCloud<pcl::PointXYZ> filtered_pc;
        down_sampling(*src_pc, filtered_pc);

        gridmap3D::Pointcloud pointcloud;
        gridmap3D::Pointcloud free_pointcloud;
        filter_pointcloud(filtered_pc, sensorToWorldTf, pointcloud, free_pointcloud);
//    filter_pointcloud(*src_pc, sensorToWorldTf, pointcloud);

        occ_gridmap_->insertPointCloudRays(pointcloud, origin);

        gridmap3D::KeyRay keyray;
        for(auto i = 0; i < free_pointcloud.size(); i++) {
            if(occ_gridmap_->computeRayKeys(origin, free_pointcloud[i], keyray)) {
                for(auto it = keyray.begin(); it != keyray.end(); it++) {
                    occ_gridmap_->updateNode(*it, false);
                }
            }
        }

        for(const auto& cell : *occ_gridmap_->getGrid()) {
            occ_gridmap_->updateNode(cell.first, false);
        }

        if(pub_occ_.getNumSubscribers() > 0)
            publish_occupied_cells();
    }

    void OccupancyGrid::down_sampling(const sensor_msgs::PointCloud2& src_pc, pcl::PointCloud<pcl::PointXYZ>& dst_pc){
        pcl::PCLPointCloud2::Ptr cloudPtr(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(src_pc, *cloudPtr);
        pcl::PCLPointCloud2::Ptr cloudfiltered(new pcl::PCLPointCloud2);

        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        sor.setInputCloud(cloudPtr);
        sor.setLeafSize(0.01f, 0.01f, 0.01f);
        sor.filter(*cloudfiltered);

        pcl::fromPCLPointCloud2(*cloudfiltered, dst_pc);
    }

    void OccupancyGrid::filter_pointcloud(const pcl::PointCloud<pcl::PointXYZ>& pcl_pointcloud, const tf::StampedTransform& transform,
                                    gridmap3D::Pointcloud& dst_pc, gridmap3D::Pointcloud& free_pc) {
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_sensor_coordinate;
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_free_in_sensor_coordinate;
        for(const auto& point : pcl_pointcloud) {
            // Remove NaN data
            if(std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z))
                continue;

            // Remove out of sensing range
            double l = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if(l > max_range_){
                pcl_pointcloud_free_in_sensor_coordinate.push_back(point);
                continue;
            }

            pcl_pointcloud_in_sensor_coordinate.push_back(point);
        }

        // in world coordinate =========================================================================================
        Eigen::Matrix4f sensorToWorld;
        pcl_ros::transformAsMatrix(transform, sensorToWorld);

        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud_in_world_coordinate;
        pcl::transformPointCloud(pcl_pointcloud_free_in_sensor_coordinate, pcl_pointcloud_in_world_coordinate, sensorToWorld);
        for(const auto& point : pcl_pointcloud_in_world_coordinate) {
            free_pc.push_back(point.x, point.y, point.z);
        }

        pcl::transformPointCloud(pcl_pointcloud_in_sensor_coordinate, pcl_pointcloud_in_world_coordinate, sensorToWorld);
        for(const auto& point : pcl_pointcloud_in_world_coordinate) {
            // box condition
            bool is_available = true;
            for (uint i = 0; i < box_condition_.size(); i++) {
                if(point.x < box_condition_[i][0] || point.x > box_condition_[i][1]
                   || point.y < box_condition_[i][2] || point.y > box_condition_[i][3]
                   || point.z < box_condition_[i][4] || point.z > box_condition_[i][5]){
                    free_pc.push_back(point.x, point.y, point.z);
                    is_available = false;
                    break;
                }
            }

            if(!is_available){
                continue;
            }

            // sphere condition
            for (uint i = 0; i < spheres_.size(); i++) {
                double radius = spheres_[i][0];
                double l = std::sqrt(std::pow(point.x-spheres_[i][1], 2) + std::pow(point.y-spheres_[i][2], 2) + std::pow(point.z-spheres_[i][3], 2));
                if (l < radius){
                    is_available = false;
                    free_pc.push_back(point.x, point.y, point.z);
                    break;
                }
            }

            if(!is_available)
                continue;

            dst_pc.push_back(point.x, point.y, point.z);
        }
    }

    void OccupancyGrid::filter_pointcloud(const sensor_msgs::PointCloud2& src_pc, const tf::StampedTransform& transform,
                                    gridmap3D::Pointcloud& dst_pc, gridmap3D::Pointcloud& free_pc) {
        // in sensor coordinate ========================================================================================
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        pcl::fromROSMsg(src_pc, pcl_pointcloud);

        filter_pointcloud(pcl_pointcloud, transform, dst_pc, free_pc);
    }

    void OccupancyGrid::publish_occupied_cells() {
        pcl::PointCloud<pcl::PointXYZ> pcl_pointcloud;
        for(auto it = occ_gridmap_->getGrid()->begin(); it != occ_gridmap_->getGrid()->end(); it++) {
            if(occ_gridmap_->isNodeOccupied(it->second)) {
                gridmap3D::point3d center = occ_gridmap_->keyToCoord(it->first);
                pcl_pointcloud.push_back(pcl::PointXYZ(center.x(), center.y(), center.z()));
            }
        }

        sensor_msgs::PointCloud2 msg_pointcloud;
        pcl::toROSMsg(pcl_pointcloud, msg_pointcloud);
        msg_pointcloud.header.frame_id = fixed_frame_id_;
        msg_pointcloud.header.stamp = ros::Time::now();
        msg_pointcloud.header.seq = 0;

        // std::cout << pcl_pointcloud.size() << std::endl;
        pub_occ_.publish(msg_pointcloud);
    }
}