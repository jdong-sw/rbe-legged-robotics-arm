#include "ros/ros.h"
#include "arm_control/SetPoseAction.h"
#include "actionlib/client/simple_action_client.h"

namespace arm_control
{
    class MoveCircleNode
    {
    public:
        MoveCircleNode(ros::NodeHandle *node) : client("pose_action", true)
        {
            this->node = node;

            // Connect to pose action server
            ROS_INFO("Waiting for pose action server...");
            this->client.waitForServer(ros::Duration(30));

            // Set parameters
            this->radius = 0.2;
            this->originX = 0.4;
            this->originY = 0.0;
            this->originZ = 0.75;
            this->angVel = -3.14 / 2;

            // Set update frequency
            ros::Rate rate(10);

            // Get starting time
            double start = ros::Time::now().toSec();

            // Start motion
            ROS_INFO("Starting motion...");
            SetPoseGoal goal;
            goal.eps = 0.1;
            while (ros::ok())
            {
                // Calculate desired positon
                double t = ros::Time::now().toSec() - start;
                double x = radius * cos(angVel * t) + this->originX;
                double y = this->originY;
                double z = radius * sin(angVel * t) + this->originZ;

                // Send goal
                ROS_INFO("Sending goal %f %f %f", x, y, z);
                goal.goal.x = x;
                goal.goal.y = y;
                goal.goal.z = z;
                goal.goal.rotx = std::vector<double>{1.0, 0.0, 0.0};
                goal.goal.roty = std::vector<double>{0.0, 1.0, 0.0};
                goal.goal.rotz = std::vector<double>{0.0, 0.0, 1.0};
                this->client.sendGoal(goal);

                // Wait a bit
                rate.sleep();
            }
        }
    private:
        ros::NodeHandle *node;
        actionlib::SimpleActionClient<arm_control::SetPoseAction> client;
        double radius;
        double originX;
        double originY;
        double originZ;
        double angVel;
    };
}

int main(int argc, char **argv)
{
    ROS_INFO("Initializing...");
    ros::init(argc, argv, "move_circle_node");
    ros::NodeHandle node;

    arm_control::MoveCircleNode solver(&node);

    ros::spin();

    return 0;
}