#include "ros/ros.h"
#include "arm_control/SetPoseAction.h"
#include "actionlib/client/simple_action_client.h"

namespace arm_control
{
    class MoveGaitNode
    {
    public:
        MoveGaitNode(ros::NodeHandle *node) : client("pose_action", true)
        {
            this->node = node;

            // Connect to pose action server
            ROS_INFO("Waiting for pose action server...");
            this->client.waitForServer(ros::Duration(30));

            // Set parameters
            this->radius = 0.2;
            this->stride_height = 0.15;
            this->stride_length = 0.4;
            this->originX = 0.0;
            this->originY = 0.0;
            this->originZ = 0.9;
            this->angVel = 3.14 / 2;

            // Set update frequency
            ros::Rate rate(10);

            // Get starting time
            double start = ros::Time::now().toSec();

            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            bool state = 0;

            // Start motion
            ROS_INFO("Starting motion...");
            SetPoseGoal goal;
            goal.eps = 0.1;
            while (ros::ok()) {
                // Calculate desired positon
                double t = ros::Time::now().toSec() - start;
                if(state == 0) { // in air
                    x = this->stride_length * cos(angVel * t) + this->originX;
                    y = this->originY;
                    z = this->originZ - this->stride_height * sin(angVel * t);

                    if (x <= this->originX - this->stride_length) {
                        state = 1;
                    }
                }
                else { // state = 1, on ground
                    x = this->stride_length * cos(angVel * t) + this->originX;
                    y = this->originY;
                    z = this->originZ;

                    if (x >= this->originX + this->stride_length) {
                        state = 0;
                    }
                }

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
        double stride_height;
        double stride_length;
        bool state;
    };
}

int main(int argc, char **argv)
{
    ROS_INFO("Initializing...");
    ros::init(argc, argv, "move_gait_node");
    ros::NodeHandle node;

    arm_control::MoveGaitNode solver(&node);

    ros::spin();

    return 0;
}