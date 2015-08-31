#include "ros/ros.h"
#include "baxter_core_msgs/EndEffectorCommand.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "baxter_close_gripper");
    ros::NodeHandle node;
    ros::Rate r(0.5);

    ros::Publisher gripper_command_publisher = node.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", 1);
           
    baxter_core_msgs::EndEffectorCommand gripper_command;

    gripper_command.id = 65664;
    gripper_command.command = baxter_core_msgs::EndEffectorCommand::CMD_GO;
    char args_buffer[50];
    std::string position_str("position");
    sprintf(args_buffer, "{\"%s\": -100.0}", position_str.c_str()); //still cannot close all the way through
    gripper_command.args = std::string(args_buffer);
    
    while(node.ok())
    {
        gripper_command_publisher.publish(gripper_command);
    }

    return 0;
}
