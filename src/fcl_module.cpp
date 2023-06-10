//
// Created by mincheul on 7/31/20.
//

#include <odyssey/fcl_module.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

namespace odyssey
{
    bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_)
    {
        CollisionData* cdata = static_cast<CollisionData*>(cdata_);
        const fcl::CollisionRequest& request = cdata->request;
        fcl::CollisionResult& result = cdata->result;

        if(cdata->done) return true;

        fcl::collide(o1, o2, request, result);

        if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
            // if(result.isCollision())
            cdata->done = true;

        return cdata->done;
    }

    bool defaultDistanceFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, fcl::FCL_REAL& dist)
    {
        DistanceData* cdata = static_cast<DistanceData*>(cdata_);
        const fcl::DistanceRequest& request = cdata->request;
        fcl::DistanceResult& result = cdata->result;

        if(cdata->done) {
            dist = result.min_distance;
            return true;
        }

        fcl::distance(o1, o2, request, result);
        dist = result.min_distance;

        if(dist <= 0)
            return true;

        return cdata->done;
    }

    bool signedDistanceFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_, fcl::FCL_REAL& dist)
    {
        DistanceData* cdata = static_cast<DistanceData*>(cdata_);
        const fcl::DistanceRequest& request = cdata->request;
        fcl::DistanceResult& result = cdata->result;

        if(cdata->done) {
            dist = result.min_distance;
            return true;
        }

        fcl::distance(o1, o2, request, result);
        dist = result.min_distance;

        if(dist <= 0){
            CollisionData cd;
            cd.request.enable_contact = true;
            const fcl::CollisionRequest& req = cd.request;
            fcl::CollisionResult& res = cd.result;

            fcl::collide(o1, o2, req, res);

            double max_pen_depth = -100000.0;
            for (auto i = 0; i < res.numContacts(); ++i) {
                const auto& contact = res.getContact(i);
                if (max_pen_depth < contact.penetration_depth){
                    max_pen_depth = contact.penetration_depth;
                }
            }
            dist = -max_pen_depth;
            if(result.min_distance == -1 || dist < result.min_distance){
                result.min_distance = dist;
            }
            return true;
        }

        return cdata->done;
    }

    FCLModule::FCLModule() {
        manager_ = new fcl::DynamicAABBTreeCollisionManager();
    }

    FCLModule::~FCLModule() {
        delete manager_;
    }

    fcl::CollisionObject* FCLModule::getCollisionBox(double x, double y, double z, double sx, double sy, double sz){
        fcl::Box b(sx, sy, sz);
        fcl::Vec3f T(x, y, z);
        fcl::Transform3f pose(T);

        std::shared_ptr<fcl::Box> geom = std::make_shared<fcl::Box>(b);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        return o;
    }

    void FCLModule::addCollisionBox(fcl::Transform3f pose, double sx, double sy, double sz){
        fcl::Box b(sx, sy, sz);

        std::shared_ptr<fcl::Box> geom = std::make_shared<fcl::Box>(b);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        objs_.push_back(o);
    }

    void FCLModule::updateManager(){
        manager_->registerObjects(objs_);
        manager_->setup();
        manager_->update();
    }

    void FCLModule::resetCollisionObjs(){
        for (uint p = 0; p < objs_.size(); p++){
            // manager_->unregisterObject(objs_[p]);
            delete objs_[p];
        }
        objs_.erase(objs_.begin(), objs_.end());
        manager_->clear();
    }

    void FCLModule::updateOccupancyGrid(std::vector<float> &pos, float res){
        resetCollisionObjs();

        for (uint p = 0; p < pos.size(); p+=3){
            objs_.push_back(getCollisionBox(pos[p], pos[p+1], pos[p+2], res, res, res));
        }

        updateManager();
    }

    double FCLModule::getDistance(double x, double y, double z, double radius) {
        fcl::Sphere s(radius);
        fcl::Vec3f T(x, y, z);
        fcl::Transform3f pose(T);

        std::shared_ptr<fcl::Sphere> geom = std::make_shared<fcl::Sphere>(s);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        DistanceData dd;

        manager_->distance(o, &dd, signedDistanceFunction);

        delete o;

        return dd.result.min_distance;
    }

    bool FCLModule::checkCollision(fcl::CollisionObject *o) {
        CollisionData cd;
        manager_->collide(o, &cd, defaultCollisionFunction);

        return cd.result.isCollision();
    }

    bool FCLModule::checkCollisionSphere(double x, double y, double z, double radius) {
        fcl::Sphere s(radius);
        fcl::Vec3f T(x, y, z);
        fcl::Transform3f pose(T);

        std::shared_ptr<fcl::Sphere> geom = std::make_shared<fcl::Sphere>(s);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        CollisionData cd;

        manager_->collide(o, &cd, defaultCollisionFunction);

        delete o;
        return cd.result.isCollision();
    }

    double FCLModule::getDistanceCone(double radius, double lz, fcl::Transform3f pose) {
        fcl::Cone c(radius, lz);
        // fcl::Vec3f T(x, y, z);
        // fcl::Transform3f pose(T);

        std::shared_ptr<fcl::Cone> geom = std::make_shared<fcl::Cone>(c);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        DistanceData dd;

        manager_->distance(o, &dd, signedDistanceFunction);

        delete o;

        return dd.result.min_distance;
    }

    bool FCLModule::checkCollisionCone(double radius, double lz, fcl::Transform3f pose) {
        fcl::Cone c(radius, lz);
        // fcl::Vec3f T(x, y, z);
        // fcl::Transform3f pose(T);

        std::shared_ptr<fcl::Cone> geom = std::make_shared<fcl::Cone>(c);
        fcl::CollisionObject *o = new fcl::CollisionObject(geom, pose);

        CollisionData cd;

        manager_->collide(o, &cd, defaultCollisionFunction);

        delete o;
        return cd.result.isCollision();
    }

}
