#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>

#include <h_pose_vs/poseAction.h>


// forward declare
ros::Subscriber twist_sub;
actionlib::SimpleActionClient<h_pose_vs::poseAction>* ac;


void twistCB(const geometry_msgs::TwistConstPtr& msg) {

    h_pose_vs::poseGoal goal;
    goal.pose.resize(6);

    goal.pose[0] = msg->linear.x;
    goal.pose[1] = msg->linear.y;
    goal.pose[2] = msg->linear.z;
    goal.pose[3] = msg->angular.x;
    goal.pose[4] = msg->angular.y;
    goal.pose[5] = msg->angular.z;

    ac->sendGoal(goal);
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_client_node");
    ros::NodeHandle nh;

    std::string action_server;

    nh.getParam("action_server", action_server);


    twist_sub = nh.subscribe("visual_servo/twist", 1, twistCB);
    ac = new actionlib::SimpleActionClient<h_pose_vs::poseAction>(action_server);

    ros::spin();
    delete ac;
    return 0;
}
