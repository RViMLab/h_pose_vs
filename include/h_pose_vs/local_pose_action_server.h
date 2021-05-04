#pragma once

#include <eigen3/unsupported/Eigen/EulerAngles>

#include <h_pose_vs/base_pose_action_server.h>


class LocalPoseActionServer : public BasePoseActionServer {

    public:
        LocalPoseActionServer(
            ros::NodeHandle nh, std::string action_server, std::string control_client,
            std::vector<double> kp,
            std::string planning_group, std::string link,
            double dt, double alpha
        );

    private:
        virtual bool _computeJacobian(moveit::core::RobotStatePtr robot_state, Eigen::MatrixXd& J);
        virtual Eigen::VectorXd _computeForwardKinematics(std::vector<double>& q);
};


LocalPoseActionServer::LocalPoseActionServer(
            ros::NodeHandle nh, std::string action_server, std::string control_client,
            std::vector<double> kp,
            std::string planning_group, std::string link,
            double dt, double alpha
) : BasePoseActionServer(
    nh, action_server, control_client,
    kp,
    planning_group, link,
    dt, alpha
) { };


bool LocalPoseActionServer::_computeJacobian(moveit::core::RobotStatePtr robot_state, Eigen::MatrixXd& J) {
    // TODO: add rotation
    auto computed = robot_state->getJacobian(
        robot_state->getJointModelGroup(_planning_group),
        robot_state->getLinkModel(_link),
        Eigen::Vector3d::Zero(),
        J
    );

    return computed;
};


Eigen::VectorXd LocalPoseActionServer::_computeForwardKinematics(std::vector<double>& q) {
    
    // Compute forward kinematics
    auto robot_model = _move_group.getRobotModel();
    auto robot_state = moveit::core::RobotState(robot_model);

    robot_state.setJointGroupPositions(robot_state.getJointModelGroup(_move_group.getName()), q);

    // https://eigen.tuxfamily.org/dox/unsupported/classEigen_1_1EulerAngles.html
    auto R = robot_state.getGlobalLinkTransform(_link).rotation();
    auto euler = Eigen::EulerAnglesZXZd(R);

    Eigen::VectorXd t(6); 
    t << robot_state.getGlobalLinkTransform(_link).translation(), euler.alpha(), euler.beta(), euler.gamma();
    return t;
};

