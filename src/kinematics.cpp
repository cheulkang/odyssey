/* Author: Mincheul Kang */

#include <ros/ros.h>
#include <odyssey/kinematics.h>

namespace odyssey
{
    Kinematics::Kinematics(std::string planning_group, const planning_scene::PlanningSceneConstPtr& planning_scene,
                               const std::string base_link, const std::string tip_link) {
        planning_group_ = planning_group;
        planning_scene_ = planning_scene;

        tracik_solver_.reset(new TRAC_IK::TRAC_IK (base_link, tip_link, "/robot_description", 0.001, 1e-4));

        setCollisionChecker();

        bool valid = tracik_solver_->getKDLChain(chain_);
        valid = tracik_solver_->getKDLLimits(ll_, ul_);
        num_joint_ = chain_.getNrOfJoints();

        // Set up KDL IK
        KDL::ChainFkSolverPos_recursive fk_solver(chain_);

        fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_)); // Forward kin. solver
        vik_solver_.reset(new KDL::ChainIkSolverVel_pinv(chain_)); // PseudoInverse vel solver
        jacsolver_.reset(new KDL::ChainJntToJacSolver(chain_));

        for (uint i = 0; i < chain_.segments.size(); i++)
        {
            std::string type = chain_.segments[i].getJoint().getTypeName();
            if (type.find("Rot") != std::string::npos) {
                if (ul_(types_.size()) >= std::numeric_limits<float>::max() &&
                    ll_(types_.size()) <= std::numeric_limits<float>::lowest())
                    types_.push_back(KDL::BasicJointType::Continuous);
                else
                    types_.push_back(KDL::BasicJointType::RotJoint);
            }
            else if (type.find("Trans") != std::string::npos)
                types_.push_back(KDL::BasicJointType::TransJoint);
        }

        max_tried_ = 30;
    }

    void Kinematics::getRandomConfiguration(KDL::JntArray& q){
        for(uint j = 0; j < num_joint_; j++){
            q(j) = odyssey::random_number(ll_(j), ul_(j));
        }
    }

    void Kinematics::setCollisionChecker() {
        collision_request_.group_name = planning_group_;
        collision_request_.contacts = true;
        collision_request_.max_contacts = 100;
        collision_request_.max_contacts_per_pair = 1;
        collision_request_.verbose = false;
    }

    bool Kinematics::collisionChecking(std::vector<double> values) {
        collision_result_.clear();
        robot_state::RobotState state = planning_scene_->getCurrentState();
        state.setJointGroupPositions(planning_group_, values);
        planning_scene_->checkCollision(collision_request_, collision_result_, state);

        return !collision_result_.collision;
    }
	
    bool Kinematics::selfCollisionChecking(std::vector<double> values) {
        collision_result_.clear();
        robot_state::RobotState state = planning_scene_->getCurrentState();
        state.setJointGroupPositions(planning_group_, values);
        planning_scene_->checkSelfCollision(collision_request_, collision_result_, state);

        return !collision_result_.collision;
    }

    double Kinematics::fRand(int i) const {
        return odyssey::random_number(ll_(i), ul_(i));
    }

    bool Kinematics::ikSolverCollFree(const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = -1;
        int tried = 0;

        KDL::JntArray q_c(num_joint_);
        std::vector<double> conf(num_joint_);
        while(rc < 0){
            for(uint j = 0; j < num_joint_; j++){
                q_c(j) = odyssey::random_number(ll_(j), ul_(j));
            }
            rc = tracik_solver_->CartToJnt(q_c, p_in, q_out);
            for(uint j = 0; j < num_joint_; j++){
                conf[j] = q_out(j);
            }
            if(!collisionChecking(conf)){
                rc = -1;
            }
            tried++;
            if(tried == max_tried_ && rc < 0){
                return false;
            }
        }
        return true;
    }

    bool Kinematics::ikSolverCollFree(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = tracik_solver_->CartToJnt(q_init, p_in, q_out);
        if(rc < 0){
            return false;
        }
        std::vector<double> conf(num_joint_);
        for(uint j = 0; j < num_joint_; j++){
            conf[j] = q_out(j);
        }
        if(!collisionChecking(conf)){
            return false;
        }
        else{
            return true;
        }
    }

    bool Kinematics::ikSolver(const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = -1;
        KDL::JntArray q_c(num_joint_);

        int tried = 0;
        while(rc < 0){
            for(uint j = 0; j < num_joint_; j++){
                q_c(j) = odyssey::random_number(ll_(j), ul_(j));
            }
            rc = tracik_solver_->CartToJnt(q_c, p_in, q_out);
            tried++;
            if(tried == max_tried_ && rc < 0){
                return false;
            }
        }
        return true;
    }

    bool Kinematics::ikSolver(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out) {
        int rc = tracik_solver_->CartToJnt(q_init, p_in, q_out);
        if(rc < 0){
            return false;
        }
        return true;
    }

    void Kinematics::fkSolver(const KDL::JntArray& q_init, KDL::Frame& p_in){
        fk_solver_->JntToCart(q_init, p_in);
    }

    uint Kinematics::getDoF(){
        return num_joint_;
    }

    void Kinematics::vikSolver(const KDL::JntArray& q, const KDL::Twist& delta_twist, KDL::JntArray& delta_q){
        vik_solver_->CartToJnt(q, delta_twist, delta_q);
    }

    void Kinematics::getJacobian(const KDL::JntArray& q, KDL::Jacobian& jac){
        jacsolver_->JntToJac(q, jac);
    }

    double Kinematics::manipPenalty(const KDL::JntArray& arr)
    {
        double penalty = 1.0;
        for (uint i = 0; i < arr.data.size(); i++)
        {
            if (types_[i] == KDL::BasicJointType::Continuous)
                continue;
            double range = ul_(i) - ll_(i);
            penalty *= ((arr(i) - ll_(i)) * (ul_(i) - arr(i)) / (range * range));
        }
        return std::max(0.0, 1.0 - exp(-1 * penalty)) * 10000;
    }

    double Kinematics::manipValue1(const KDL::JntArray& arr)
    {
        KDL::Jacobian jac(arr.data.size());

        jacsolver_->JntToJac(arr, jac);

        Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
        Eigen::MatrixXd singular_values = svdsolver.singularValues();

        double error = 1.0;
        for (unsigned int i = 0; i < singular_values.rows(); ++i)
            error *= singular_values(i, 0);
        return error;
    }

    double Kinematics::manipValue2(const KDL::JntArray& arr)
    {
        KDL::Jacobian jac(arr.data.size());

        jacsolver_->JntToJac(arr, jac);

        Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
        Eigen::MatrixXd singular_values = svdsolver.singularValues();

        return singular_values.minCoeff() / singular_values.maxCoeff();
    }

    double Kinematics::manipValue3(const KDL::JntArray& arr)
    {
        KDL::Jacobian jac(arr.data.size());

        jacsolver_->JntToJac(arr, jac);

        return std::abs(std::sqrt((jac.data * jac.data.transpose()).determinant()));
    }

    double Kinematics::jointPositionLimitPotentialFunction(double value, uint i){
        if(value == ll_(i)){
            return 0.0;
        }

        return std::abs( (2 * value - ul_(i) - ll_(i)) / ( 4 * std::pow(ul_(i) - value, 2)) );
    }

    double Kinematics::extendManipValue(const KDL::JntArray& arr){
        KDL::Jacobian jac(arr.data.size());
        jacsolver_->JntToJac(arr, jac);

        // compute L(arr)
        KDL::JntArray penalties(num_joint_);
        for (uint i = 0; i < arr.data.size(); i++)
        {
            if (types_[i] == KDL::BasicJointType::Continuous){
                penalties(i) = 1.0;
            }
            else{
                penalties(i) = 1.0 / std::sqrt(1 + jointPositionLimitPotentialFunction(arr(i), i));
            }
        }

        for(uint i = 0; i < 6; i++){
            for (uint j = 0; j < num_joint_; j++){
                jac.data(i, j) *= penalties(j);
            }
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
        Eigen::MatrixXd singular_values = svdsolver.singularValues();

        return singular_values.minCoeff() / singular_values.maxCoeff();
    }
}
