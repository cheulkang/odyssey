/* Author: Mincheul Kang */

#ifndef ODYSSEY_KINEMATICS_H
#define ODYSSEY_KINEMATICS_H

#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <trac_ik/trac_ik.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <odyssey/utils.h>

namespace odyssey
{
    class Kinematics
    {
    public:
        Kinematics(std::string planning_group, const planning_scene::PlanningSceneConstPtr& planning_scene,
                const std::string base_link, const std::string tip_link);
        virtual ~Kinematics(){};

        double fRand(int i) const;
        void getRandomConfiguration(KDL::JntArray& q);
        void setCollisionChecker();
        bool collisionChecking(std::vector<double> values);
        bool selfCollisionChecking(std::vector<double> values);

        bool ikSolver(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolver(const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolverCollFree(const KDL::Frame& p_in, KDL::JntArray& q_out);
        bool ikSolverCollFree(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out);
        void fkSolver(const KDL::JntArray& q_init, KDL::Frame& p_in);
        void vikSolver(const KDL::JntArray& q, const KDL::Twist& delta_twist, KDL::JntArray& delta_q);
        void getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac);
        uint getDoF();
        double manipPenalty(const KDL::JntArray& arr);
        double manipValue1(const KDL::JntArray& arr);
        double manipValue2(const KDL::JntArray& arr);
        double manipValue3(const KDL::JntArray& arr);
        double extendManipValue(const KDL::JntArray& arr);
        double jointPositionLimitPotentialFunction(double value, uint i);
    private:
        KDL::Chain chain_;
        KDL::JntArray ll_, ul_; //lower joint limits, upper joint limits
        std::vector<KDL::BasicJointType> types_;
        uint num_joint_;
        std::string planning_group_;
        planning_scene::PlanningSceneConstPtr planning_scene_;

        collision_detection::CollisionRequest collision_request_;
        collision_detection::CollisionResult collision_result_;

        std::unique_ptr<TRAC_IK::TRAC_IK> tracik_solver_;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
        std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver_;
        std::unique_ptr<KDL::ChainIkSolverVel_pinv> vik_solver_;

        uint max_tried_;
    };
}

#endif