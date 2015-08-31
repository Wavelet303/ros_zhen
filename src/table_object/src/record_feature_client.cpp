#include "ros/ros.h"
#include "table_object/record_feature.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "record_feature_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<table_object::record_feature>("record_feature");
    table_object::record_feature srv;
    srv.request.record_name = argv[1];
    
    bool service_succeed = false;
    
    ros::Rate r(1);
    
    while(!service_succeed)
    {
        if (client.call(srv))
        {
            ROS_INFO("succeed: %d", srv.response.succeed);
            service_succeed = srv.response.succeed;
        }
        else
        {
            ROS_ERROR("Failed to call service record_feature");
            return 1;
        }
        r.sleep();
    }
    return 0;
}
