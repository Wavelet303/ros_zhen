/**
 * \file save_cluster.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief save a selected cluster of points into pcd file
 */

#include <sys/stat.h>

//ros
#include <ros/ros.h>

// msgs
#include <sensor_msgs/PointCloud2.h>

// pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/approximate_voxel_grid.h>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// pointcloud2 to ros msgs conversions
#include <pcl_conversions/pcl_conversions.h>

// from ros_sec
#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"
#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include "ros_sec/Visualizer/view2D.h"
#include "ros_sec/Visualizer/view3D.h"
#include "ros_sec/util/util.h"
bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

// publisher, subscriber
ros::Subscriber scene_subscriber;

// pcl::visualization::PCLVisualizer result_viewer("table");
boost::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer (new pcl::visualization::PCLVisualizer ("table"));

// set up variables for tabletop localization;
TableObject::Segmentation initialSeg;
TableObject::pcdCloud pcdSceneCloud;
CloudPtr sceneCloud;
CloudPtr planeCloud(new Cloud);
CloudPtr cloud_objects(new Cloud);
CloudPtr cloud_hull(new Cloud);

std::vector<pcl::PointIndices> clusters;

pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
float leaf_size=0.005;

int counter=0;
int selected_id;
int initial_cluster_size;

bool beginning = true;

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    if(counter < 10){
        counter++; //first few frames are noisy
    }else{
        /***************************************
        *  transform from 
        * -sensor_msgs::PointCloud2 -> pcl::PCLPointCloud2
        * -pcl::PointCloud2 -> Cloud
        ***************************************/
        pcl::PCLPointCloud2 tempPCL;
        CloudPtr scene_cloud(new Cloud);
        pcl_conversions::toPCL(input, tempPCL);
        pcl::fromPCLPointCloud2(tempPCL, *scene_cloud);
        
        /***************************************
        *  set up cloud, segmentation
        ***************************************/
        // set threshold of size of clustered cloud
        initialSeg.setThreshold(200);
        // downsampler
        grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        
        /***************************************
        *  localize the tabletop and objects
        ***************************************/
        
        if(beginning)
        {
            /***************************************
            *  Intialization:
            * -plane localization
            * -object cluster extraction
            * -bottle cluster localization
            ***************************************/
            initialSeg.resetCloud(scene_cloud);            
            initialSeg.seg(true); 
            initialSeg.getCloudHull(cloud_hull);
//             result_viewer->removeCoordinateSystem("reference", 0);
            
            beginning = false;
            
        }else{
            initialSeg.resetCloud(scene_cloud);            
            initialSeg.seg(cloud_hull, false);
        }
        
        
        initialSeg.getTableTopCloud(planeCloud);
        initialSeg.getObjects(cloud_objects, clusters);            
        initialSeg.getsceneCloud(pcdSceneCloud);
        sceneCloud=pcdSceneCloud.getCloud();
            
        result_viewer->removeAllPointClouds();
        result_viewer->removeAllShapes();
        TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters, true);
        pcl::visualization::PointCloudColorHandlerRGBAField<RefPointType> color_table(planeCloud);
        result_viewer->addPointCloud<RefPointType>(planeCloud, color_table, "table", 0);
        
        result_viewer->addCoordinateSystem(0.3, "reference", 0);
        Eigen::Affine3f toPlaneCoordinate;
        std::vector<Eigen::Vector3f> planeCoorInCamCoor = initialSeg.getPlaneCoorInCamCoor();
        pcl::getTransformationFromTwoUnitVectorsAndOrigin(planeCoorInCamCoor[1], planeCoorInCamCoor[2], planeCoorInCamCoor[3], toPlaneCoordinate);
        std::cout << "Here is the plane coordinate system:" << planeCoorInCamCoor[0] << std::endl
                                                            << planeCoorInCamCoor[1] << std::endl
                                                            << planeCoorInCamCoor[2] << std::endl
                                                            << planeCoorInCamCoor[3] << std::endl;
        result_viewer->addCoordinateSystem(0.3, toPlaneCoordinate.inverse(), "plane_reference", 0);
        
        
//         result_viewer->spin();
        result_viewer->spinOnce(100);
        
//         std::cout << "please input the cluster index to be saved\n";
//         std::cin >> selected_id;
//         std::cout << "selected_id set to " << selected_id << std::endl;
        
        
        /***************************************
        *  confirm selected cluster
        ***************************************/
//         result_viewer->addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
//         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
//         
//         std::vector<pcl::PointIndices> cube_cluster;
//         cube_cluster.push_back(clusters[selected_id]);
//         result_viewer->removeAllPointClouds();
//         result_viewer->removeAllShapes();
//         
//         result_viewer->addPointCloud<RefPointType>(sceneCloud, rgb, "new frame");
//         TableObject::view3D::drawClusters(result_viewer, cloud_objects, cube_cluster, true); 
//         
//         result_viewer->spinOnce(100);
//         char ready;
//         std::cout << "is the cluster correctly selected? [y/n]\n";
//         std::cin >> ready;
//         if(ready=='y')
//         {
//             // save pcd file for one selected bottle cluster => for cube fitting test
//             CloudPtr cluster_cube (new Cloud);
//             pcl::copyPointCloud (*cloud_objects, clusters[selected_id], *cluster_cube);
//             pcl::io::savePCDFileASCII ("test_cube_pcd.pcd", *cluster_cube);
//             exit(0);
//         }
    }
}

int main(int argc, char **argv)
{
    /***************************************
    *  ros node initialization
    ***************************************/
    ros::init(argc, argv, "save_cluster");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(20);   //loop rate for any loops appear in this cpp
    sleep(10.0); //wait for other nodes up and running in baxter_moveit/mvoe_group_server.launch
    
    /***************************************
    *  setup visualizer
    ***************************************/
    result_viewer->setCameraPosition(-0.0100231, 0.155521, -1.10094, -0.0136177, 0.1338, -0.953934, -0.00975635, -0.989179, -0.146392);
    result_viewer->setCameraClipDistances(0.00934089, 9.34089);
    result_viewer->setSize(800, 450);
    result_viewer->setPosition(500,300);
//     result_viewer->setCameraPosition(Position, Focal point, View up);
//     result_viewer->setCameraClipDistances(Clipping plane);

    /***************************************
    *  setup publisher, subscriber
    ***************************************/
    scene_subscriber = node_handle.subscribe("camera/depth_registered/points", 1, cloud_cb);
    
    /***************************************
    *  spin ros node
    ***************************************/
    ros::spin();
//     while(ros::ok())
//     {
//         ros::spinOnce();
//     }
    
    
    return 0;
}
