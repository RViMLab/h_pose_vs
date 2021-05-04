#pragma once

#include <vector>
#include <tuple>
#include <Eigen/Core>

#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Twist.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <h_pose_vs/poseAction.h>
#include <h_pose_vs/damped_least_squares.h>


// Note: 
// moveit servo: not working...
// https://github.com/ros-planning/moveit/blob/3361b2d1b6b2feabc2d3e93c75653f5a00e87fa4/moveit_ros/moveit_servo/src/cpp_interface_example
class BasePoseActionServer {

    public:
        BasePoseActionServer(
            ros::NodeHandle nh, std::string action_server, std::string control_client,
            std::vector<double> kp, double th_t, std::vector<double> th_scale,
            std::string planning_group, std::string link,
            double dt, double alpha
        );

    protected:
        virtual bool _computeJacobian(moveit::core::RobotStatePtr robot_state, Eigen::MatrixXd& J) = 0;
        virtual Eigen::VectorXd _computeForwardKinematics(std::vector<double>& q) = 0;

        ros::NodeHandle _nh;

        // Server to handle goals via _goalCB callback
        std::string _action_server;
        actionlib::SimpleActionServer<h_pose_vs::poseAction> _as;

        // Client to request joint angle goals on actual robot
        std::string _control_client;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> _ac;

        // Gain
        Eigen::VectorXd _kp;  // proportional gain

        // Threshold and scale
        double _th_t;
        Eigen::VectorXd _th_t_scale;

        // Robot model
        std::string _planning_group;
        std::string _link;
        moveit::planning_interface::MoveGroupInterface _move_group;
        double _dt, _alpha;  // control interval and velocity scaling

        // Goal callback
        void _goalCB(const h_pose_vs::poseGoalConstPtr& goal);

        // State machine
        actionlib::SimpleClientGoalState _velocityControlStateMachine(Eigen::VectorXd& td);

        // Gain control
        std::tuple<std::vector<double>, std::vector<double>> _computeUpdate(Eigen::VectorXd& td);

        // Execute goal on client robot
        actionlib::SimpleClientGoalState _executeGoal(std::vector<double> q, bool wait_for_result=false);
};


BasePoseActionServer::BasePoseActionServer(
    ros::NodeHandle nh, std::string action_server, std::string control_client,
    std::vector<double> kp, double th_t, std::vector<double> th_scale,
    std::string planning_group, std::string link,
    double dt, double alpha
) : _action_server(action_server), _as(nh, action_server, boost::bind(&BasePoseActionServer::_goalCB, this, _1), false),
    _control_client(control_client), _ac(nh, control_client, false),
    _kp(Eigen::Map<Eigen::VectorXd>(kp.data(), kp.size())), _th_t(th_t), _th_t_scale(Eigen::Map<Eigen::VectorXd>(th_scale.data(), th_scale.size())),
    _planning_group(planning_group), _link(link),
    _move_group(planning_group),
    _dt(dt), _alpha(alpha) {

    _as.start();
    ROS_INFO("BasePoseActionServer: Waiting for action server under %s...", control_client.c_str());
    _ac.waitForServer();
    ROS_INFO("Done.");
    _move_group.setMaxVelocityScalingFactor(alpha);
};


void BasePoseActionServer::_goalCB(const h_pose_vs::poseGoalConstPtr& goal) {
    
    // Desired task from goal
    Eigen::VectorXd td;

    td = Eigen::VectorXd::Map(goal->pose.data(), goal->pose.size());

    // State machines
    _velocityControlStateMachine(td);
};


actionlib::SimpleClientGoalState BasePoseActionServer::_velocityControlStateMachine(Eigen::VectorXd& td) {

    if (_as.isPreemptRequested() || !ros::ok()) {
        ROS_DEBUG("%s: Preempted", _action_server.c_str());
        _as.setPreempted();
        return actionlib::SimpleClientGoalState::PREEMPTED;
    }

    auto [q, qd]  = _computeUpdate(td);

    // add error checks
    auto e = (_th_t_scale.asDiagonal()*(_computeForwardKinematics(q) - _computeForwardKinematics(qd))).norm();

    if (e > _th_t) {
        ROS_WARN("Goal rejected due to threshold limit.");
        _as.setAborted();
        return actionlib::SimpleClientGoalState::REJECTED;
    }

    _executeGoal(qd);

    h_pose_vs::poseResult result;
    result.pose = qd;
    _as.setSucceeded(result);
    return actionlib::SimpleClientGoalState::SUCCEEDED;
};


std::tuple<std::vector<double>, std::vector<double>> BasePoseActionServer::_computeUpdate(Eigen::VectorXd& td) {
    // Compute jacobian
    auto robot_state = _move_group.getCurrentState();
    auto q = _move_group.getCurrentJointValues();
    auto qd = q;

    Eigen::MatrixXd J;
    if (!_computeJacobian(robot_state, J)) return std::make_tuple(q, qd);

    // Gain control
    int n = J.rows();
    if (n != _kp.size()) throw std::runtime_error("Size of _kp must equal Jacobian dimension!");
    auto Kp = _kp.asDiagonal();
    auto J_inverse = dampedLeastSquares(J);
    auto dq = J_inverse*Kp*td;

    for (int i = 0; i < q.size(); i++) {
        qd[i] = q[i] + dq[i]*_dt;
    }

    return std::make_tuple(q, qd);
};


actionlib::SimpleClientGoalState BasePoseActionServer::_executeGoal(std::vector<double> q, bool wait_for_result) {

    // See for example http://wiki.ros.org/pr2_controllers/Tutorials/Moving%20the%20arm%20using%20the%20Joint%20Trajectory%20Action
    
    // Execute motion on client
    trajectory_msgs::JointTrajectoryPoint point;
    point.time_from_start = ros::Duration(_dt/_alpha);
    point.positions = q;

    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names = _move_group.getJointNames();
    goal.trajectory.points.push_back(point);

    _ac.sendGoal(goal);
    if (wait_for_result) _ac.waitForResult();

    return _ac.getState();
};
