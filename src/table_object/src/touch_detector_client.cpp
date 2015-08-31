#include "ros/ros.h"
#include "table_object/detect_touch.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "touch_detector_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<table_object::detect_touch>("detect_touch");
    table_object::detect_touch srv;
    srv.request.detect = true;
        
    ros::Rate r(1);
    bool service_succeed = false;
    
    while(!service_succeed)
    {
        if (client.call(srv))
        {
            ROS_INFO("detection results: %d, %d, %d", srv.response.bottle_hand, srv.response.bottle_tabletop, srv.response.hand_tabletop);
            service_succeed = srv.response.succeed;
        }
        else
        {
            ROS_ERROR("Failed to call service detect_touch");
            return 1;
        }
        r.sleep();
    }
    
    return 0;
}