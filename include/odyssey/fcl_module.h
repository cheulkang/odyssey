//
// Created by mincheul on 7/31/20.
//

#ifndef ODYSSEY_FCL_H
#define ODYSSEY_FCL_H

#include <ros/ros.h>
#include <ros/package.h>

#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/config.h>
#include <fcl/data_types.h>
#include <fcl/broadphase/broadphase.h>
#include <fcl/collision_data.h>
#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shape_to_BVH_model.h>
#include <fcl/traversal/traversal_node_bvhs.h>
#include <fcl/traversal/traversal_node_setup.h>
#include <fcl/collision_node.h>
#include <fcl/collision.h>
#include <fcl/distance.h>

#include <iostream>
#include <odyssey/npy.hpp>

namespace odyssey {
    struct CollisionData {
        CollisionData() {
            done = false;
        }

        fcl::CollisionRequest request;
        fcl::CollisionResult result;
        bool done;
    };

    struct DistanceData {
        DistanceData() {
            done = false;
        }

        fcl::DistanceRequest request;
        fcl::DistanceResult result;
        bool done;
    };

    class FCLModule {
    public:
        FCLModule();
        ~FCLModule();

//        void initSelfObjs(std::vector<fcl::CollisionObject *> objs);
        void updateOccupancyGrid(std::vector<float> &pos, float res);
        double getDistance(double x, double y, double z, double radius);
        fcl::CollisionObject* getCollisionBox(double x, double y, double z, double sx, double sy, double sz);
        void addCollisionBox(fcl::Transform3f pose, double sx, double sy, double sz);
        void resetCollisionObjs();
        void updateManager();
        double getDistanceCone(double radius, double lz, fcl::Transform3f pose);

        bool checkCollision(fcl::CollisionObject *o);
        bool checkCollisionSphere(double x, double y, double z, double radius);
        bool checkCollisionCone(double radius, double lz, fcl::Transform3f pose);

    private:
        std::vector<fcl::CollisionObject *> objs_;
        fcl::BroadPhaseCollisionManager *manager_;
//        fcl::BroadPhaseCollisionManager *manager_self_;
    };
}

#endif