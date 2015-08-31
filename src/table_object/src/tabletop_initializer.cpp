/**
 * \file        extract_sec.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 * \brief       localize plane at the 1st frame and store it at config/
 */


#include <ros/ros.h>

// msgs
#include <sensor_msgs/PointCloud2.h>

// pcl pointcloud to pointcloud2 conversions
#include <pcl/conversions.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>

// pointcloud2 to ros msgs conversions
#include <pcl_conversions/pcl_conversions.h>

// from ros_sec
#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"

// std
#include <time.h>
#include <iostream>
#include <fstream>
#include <iomanip>

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

pcl::visualization::PCLVisualizer result_viewer("tabletop");
bool beginning = true;
std::string filename_pcd;

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    CloudPtr scene_cloud(new Cloud);
    TableObject::Segmentation planeSeg;
    CloudPtr cloud_hull(new Cloud);
    pcl::ModelCoefficients coefficients;
    planeSeg.setThreshold(200);
   
    // only takes the first frame
    if(beginning)
    {
        beginning = false;
        //Transform
        pcl::PCLPointCloud2 tempPCL;
        pcl_conversions::toPCL(input, tempPCL);
        pcl::fromPCLPointCloud2(tempPCL, *scene_cloud);
        
        planeSeg.resetCloud(scene_cloud);
        
        // extract plane
        std::clock_t t = std::clock();
        planeSeg.seg_hull(false);
        planeSeg.getCloudHull(cloud_hull);
        planeSeg.getPlaneCoefficients(coefficients);
        t = std::clock() - t;
        ROS_INFO("First frame: Plane segmentation + plane cloud hull: %f seconds", ((float)t)/CLOCKS_PER_SEC);
        
        pcl::io::savePCDFileASCII("config/plane_hull.pcd", *cloud_hull);
        std::ofstream out("config/plane_coef.txt");
        for(int i=0; i<4; i++)
        {
//             out << std::fixed << std::setprecision(10) << coefficients.values[i] << std::endl;
            out << coefficients.values[i] << std::endl;
        }
        out.close();
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene_cloud);
        result_viewer.addPointCloud<RefPointType>(scene_cloud, rgb, "new scene");
        result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
        
        while (!result_viewer.wasStopped ())
        {
            result_viewer.spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }else{
        result_viewer.close();
    }
}

int main(int argc, char **argv)
{
    result_viewer.addCoordinateSystem();

    ros::init(argc, argv, "tabletop_initializer");
    ros::NodeHandle node_handle;
    
    /***************************************
    *  parse arguments
    ***************************************/
    if(argc==1)
    {
        ros::Subscriber scene_subscriber = node_handle.subscribe("camera/depth_registered/points", 100, cloud_cb);
        ros::spin();  
    } else if(argc<3)
    {
        ROS_INFO("Usage: tabletop_initializer DATA_PATH/PCD_FILE_FORMAT INDEX");
        exit(1);
    } else
    {
        std::string basename_cloud=argv[1];
        unsigned int index = std::atoi(argv[2]);
        filename_pcd = cv::format(basename_cloud.c_str(), index);
        ROS_INFO("loading file %s", filename_pcd.c_str());
        
        CloudPtr scene_cloud(new Cloud);
        TableObject::Segmentation planeSeg;
        CloudPtr cloud_hull(new Cloud);
        pcl::ModelCoefficients coefficients;
        planeSeg.setThreshold(200);
    
        // only takes the first frame
        planeSeg.resetCloud(filename_pcd, false);
        //TableObject::Segmentation to CloudPtr
        TableObject::pcdCloud pcdSceneCloud;
        planeSeg.getsceneCloud(pcdSceneCloud);
        scene_cloud = pcdSceneCloud.getCloud();
        
        // extract plane
        std::clock_t t = std::clock();
        planeSeg.seg_hull(false);
        planeSeg.getCloudHull(cloud_hull);
        planeSeg.getPlaneCoefficients(coefficients);
        t = std::clock() - t;
        ROS_INFO("First frame: Plane segmentation + plane cloud hull: %f seconds", ((float)t)/CLOCKS_PER_SEC);
        
        pcl::io::savePCDFileASCII("plane_hull.pcd", *cloud_hull);
        std::ofstream out("config/plane_coef.txt");
        for(int i=0; i<4; i++)
        {
//             out << std::fixed << std::setprecision(10) << coefficients.values[i] << std::endl;
            out << coefficients.values[i] << std::endl;
        }
        out.close();
        
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene_cloud);
        result_viewer.addPointCloud<RefPointType>(scene_cloud, rgb, "scene");
        result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
        
        while (!result_viewer.wasStopped ())
        {
            result_viewer.spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
      

    return 0;
}
