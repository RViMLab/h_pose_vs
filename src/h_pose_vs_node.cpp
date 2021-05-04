
#include <ros/ros.h>

#include <h_pose_vs/local_pose_action_server.h>


int main(int argc, char** argv) {

    ros::init(argc, argv, "h_pose_vs_node");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    spinner.start();

    std::string action_server, control_client;
    std::vector<double> kp;
    std::string planning_group, link;
    double dt, alpha;

    nh.getParam("action_server", action_server);
    nh.getParam("control_client", control_client);
    nh.getParam("kp", kp);
    nh.getParam("planning_group", planning_group);
    nh.getParam("link", link);
    nh.getParam("dt", dt);
    nh.getParam("alpha", alpha);

    LocalPoseActionServer as(
        nh, action_server, control_client,
        kp,
        planning_group, link,
        dt, alpha
    );

    ros::waitForShutdown();

    return 0;
}
