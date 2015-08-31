/**
 * \file robot_scene_tracking.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief localize tabletop and track bottle in the scene as robot manipulates it
 */

#include <sys/stat.h>

//ros
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// msgs
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>
#include "table_object/bottle_feature.h"
#include "table_object/palm_reflex_triggered.h"

// pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

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
#include "ros_sec/Tracker/track3D.h"
#include "ros_sec/util/util.h"
#include "ros_sec/Detector/colorDetector.h"
#include "ros_sec/Detector/touchDetector.h"
#include "ros_sec/Detector/bottleDetector.h"
#include "ros_sec/SEC/mainGraph.h"
#include "ros_sec/util/util.h"
bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

#include "table_object/featureDef.h"

// publisher, subscriber
ros::Publisher collision_object_publisher;
ros::Subscriber scene_subscriber;
ros::Publisher bottle_feature_publisher;
ros::Subscriber palm_relfex_signal_subscriber;

// msgs
moveit_msgs::CollisionObject collision_table;
moveit_msgs::CollisionObject collision_bottle;
table_object::bottle_feature bottle_feature;
table_object::palm_reflex_triggered palm_reflex_signal;

bool beginning = true;
// pcl::visualization::PCLVisualizer result_viewer("table_bottle");
boost::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer (new pcl::visualization::PCLVisualizer ("table_bottle"));

// set up variables for 1) tabletop localization; 2) finger detection; 3) tracking
TableObject::Segmentation tableObjSeg;
TableObject::Segmentation initialSeg;
TableObject::track3D tracker(false);
TableObject::colorDetector finger1Detector(0,100,0,100,100,200);
TableObject::colorDetector finger2Detector(150,250,0,100,0,100);
TableObject::bottleDetector bottleDetector;
TableObject::pcdCloud pcdSceneCloud;
CloudPtr sceneCloud;
CloudPtr planeCloud(new Cloud);
CloudPtr cloud_objects(new Cloud);
CloudPtr cloud_finger1(new Cloud);
CloudPtr cloud_finger2(new Cloud);
CloudPtr cloud_hull(new Cloud);
CloudPtr track_target(new Cloud);
CloudPtr tracked_cloud(new Cloud);

std::vector<pcl::PointIndices> clusters;
pcl::ModelCoefficients coefficients;
pcl::PointIndices f1_indices;
pcl::PointIndices f2_indices;
Eigen::Affine3f toBottleCoordinate;
Eigen::Affine3f transformation;
Eigen::Affine3f coordinateTransform;

pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
float leaf_size=0.005;

int counter=0;
int bottle_id;
Eigen::Vector3f bottle_init_ori;
int initial_cluster_size;

bool palm_reflex_triggered = false;

void
palm_reflex_callback(const table_object::palm_reflex_triggered input)
{
	palm_reflex_triggered = input.palm_reflex_triggered;
}

void 
cloud_cb (const sensor_msgs::PointCloud2 input)
{
    tf::TransformBroadcaster br;
    ros::Time now = ros::Time::now();
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
        *  set up cloud, segmentation, tracker, detectors
        ***************************************/
        // set threshold of size of clustered cloud
        tableObjSeg.setThreshold(30);
        initialSeg.setThreshold(200);
        // downsampler
        grid.setLeafSize (leaf_size, leaf_size, leaf_size);
        
        /***************************************
        *  if beginning==true, localize the tabletop and bottle
        *  otherwise track the bottle
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
            initialSeg.seg(false); 
            initialSeg.getObjects(cloud_objects, clusters);
            initialSeg.getCloudHull(cloud_hull);
            initialSeg.getPlaneCoefficients(coefficients);
            
            initialSeg.getsceneCloud(pcdSceneCloud);
            initialSeg.getTableTopCloud(planeCloud);
            sceneCloud=pcdSceneCloud.getCloud();
            
            result_viewer->removeShape("new frame");
            result_viewer->removeCoordinateSystem("reference", 0);
            result_viewer->removeShape("bottle_arrow");
            TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters, true); 
            result_viewer->spinOnce(100);
            std::cout << "please input the cluster index of bottle\n";
            std::cin >> bottle_id;
            std::cout << "bottle_id set to " << bottle_id << std::endl;
            
            /***************************************
             *  Localizing cylinder
             ***************************************/
            CloudPtr cluster_bottle (new Cloud);
            pcl::copyPointCloud (*cloud_objects, clusters[bottle_id], *cluster_bottle);
            bottleDetector.setInputCloud(cluster_bottle);
            bottleDetector.fit();
            bottle_init_ori= bottleDetector.getOrientation();
            bottleDetector.getTransformation(toBottleCoordinate);
//             bottleDetector.drawOrientation(result_viewer);
            
            /***************************************
             *  confirm localization
             ***************************************/
            result_viewer->addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
            result_viewer->addCoordinateSystem(0.3, toBottleCoordinate.inverse(), "reference", 0);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
            result_viewer->addPointCloud<RefPointType>(sceneCloud, rgb, "new frame");
            result_viewer->spinOnce(100);
            char ready;
            std::cout << "is the bottle correctly localized? [y/n]\n";
            std::cin >> ready;
            if(ready=='y')
            {
                beginning=false;
                initial_cluster_size = clusters.size();
                std::cout << "ready to initialize bottle tracker\n";
                /***************************************
                *  Tracking initialization
                ***************************************/
                {
                    //pcl::ScopeTime t_track("Tracker initialization");
                    tracker.setTarget(cluster_bottle, bottleDetector.getCenter());
                    tracker.initialize();
                }
                
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(toBottleCoordinate.inverse(), x, y, z, roll, pitch, yaw);
                
                /***************************************
                *  setup collision bottle msg
                ***************************************/
                collision_bottle.header.frame_id = "/pcl_camera";
                collision_bottle.header.stamp = now;
                collision_bottle.id = "bottle";
                collision_bottle.operation = collision_bottle.ADD;
                /* Define a cylinder to add to the world. */
                shape_msgs::SolidPrimitive bottle_primitive;
                bottle_primitive.type = bottle_primitive.CYLINDER;
                bottle_primitive.dimensions.resize(2);
                bottle_primitive.dimensions[0] = bottleDetector.getHeight(); //fixed height
                bottle_primitive.dimensions[1] = bottleDetector.getRadius(); //fixed radius: avoid contact found while in grasp

                /* A pose for the bottle (specified relative to frame_id) */
                geometry_msgs::Pose bottle_pose;
                bottle_pose.position.x = x;
                bottle_pose.position.y = y;
                bottle_pose.position.z = z;
                tf::Quaternion ori;
                geometry_msgs::Quaternion ori_quat;
                ori.setRPY(roll, pitch, yaw);
                tf::quaternionTFToMsg(ori, ori_quat);
                bottle_pose.orientation = ori_quat;                     
                
                collision_bottle.primitives.push_back(bottle_primitive);
                collision_bottle.primitive_poses.push_back(bottle_pose);
                collision_object_publisher.publish(collision_bottle);
                sleep(2.0);
                
                /***************************************
                *  setup bottle feature msg
                ***************************************/
                bottle_feature.bottle_cylinder = collision_bottle;
                bottle_feature.rpy.x = roll;
                bottle_feature.rpy.y = pitch;
                bottle_feature.rpy.z = yaw;
                bottle_feature.color.x = bottleDetector.getCenter().r;
                bottle_feature.color.y = bottleDetector.getCenter().g;
                bottle_feature.color.z = bottleDetector.getCenter().b;
                bottle_feature_publisher.publish(bottle_feature);
                
            }else{
                beginning=true;
            }
        }else{
             /***************************************
             *  remove visualization anontation at intialization stage
             ***************************************/
            std::stringstream bottle_viz_id ("cluster");
            bottle_viz_id << bottle_id;
            result_viewer->removeShape(bottle_viz_id.str());
            for(int i=0; i<initial_cluster_size; i++)
            {
                std::stringstream bottle_viz_text_id;
                bottle_viz_text_id << i;
                result_viewer->removeText3D(bottle_viz_text_id.str());
            }            
            
            /***************************************
             *  object cloud extraction
             ***************************************/
//             tableObjSeg.resetCloud(scene_cloud);
//             tableObjSeg.seg(cloud_hull,false);
//             tableObjSeg.getObjects(cloud_objects, clusters);
//             tableObjSeg.getsceneCloud(pcdSceneCloud);
//             sceneCloud=pcdSceneCloud.getCloud();  
            
            /***************************************
             *  Tracking objects
             ***************************************/
            {
//                 pcl::ScopeTime t_track("Tracking");
                grid.setInputCloud (scene_cloud);
                grid.filter (*track_target);
                tracker.track(track_target, transformation);
                tracker.getTrackedCloud(tracked_cloud);
                tracker.getCoordinateTransform(coordinateTransform);
            }
//             tracker.viewTrackedCloud(result_viewer);
//             tracker.drawParticles(result_viewer);
            
            /***************************************
             *  compute tracked <center, orientation>
             ***************************************/
            pcl::PointXYZ bottle_loc_point(0,0,0);
            bottle_loc_point = pcl::transformPoint<pcl::PointXYZ>(bottle_loc_point, transformation);
            result_viewer->removeShape("bottle_center");
//             result_viewer->addSphere<pcl::PointXYZ>(bottle_loc_point, 0.05, "bottle_center");
            
            Eigen::Vector3f bottle_ori;
            pcl::transformVector(bottle_init_ori,bottle_ori,transformation);
            TableObject::view3D::drawArrow(result_viewer, bottle_loc_point, bottle_ori, "bottle_arrow");
            
            /***************************************
             *  calculate toTrackedBottleCoordinate
             ***************************************/
            Eigen::Affine3f toTrackedBottleCoordinate;
            Eigen::Vector3f p( bottle_loc_point.x, bottle_loc_point.y, bottle_loc_point.z ); // position

            // get a vector that is orthogonal to _orientation ( yc = _orientation x [1,0,0]' ) 
            Eigen::Vector3f yc( 0, bottle_ori[2], -bottle_ori[1] ); 
            yc.normalize(); 
            // get a transform that rotates _orientation into z and moves cloud into origin. 
            pcl::getTransformationFromTwoUnitVectorsAndOrigin(yc, bottle_ori, p, toTrackedBottleCoordinate);
            
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(toTrackedBottleCoordinate.inverse(), x, y, z, roll, pitch, yaw);
            
            /***************************************
            *  setup collision bottle msg
            ***************************************/
//             collision_bottle.header.frame_id = "/pcl_camera";
//             collision_bottle.id = "bottle";
			if(!palm_reflex_triggered)
            	collision_bottle.operation = collision_bottle.ADD;
        	else
        		collision_bottle.operation = collision_bottle.REMOVE;
            collision_bottle.header.stamp = now;
            /* A pose for the bottle (specified relative to frame_id) */
            geometry_msgs::Pose bottle_pose;
            bottle_pose.position.x = x;
            bottle_pose.position.y = y;
            bottle_pose.position.z = z;
            tf::Quaternion ori;
            geometry_msgs::Quaternion ori_quat;
            ori.setRPY(roll, pitch, yaw);
            tf::quaternionTFToMsg(ori, ori_quat);
            bottle_pose.orientation = ori_quat;
            
            collision_bottle.primitive_poses[0]=bottle_pose;
        	collision_object_publisher.publish(collision_bottle);
            
            /***************************************
            *  setup bottle feature msg
            ***************************************/
            bottle_feature.bottle_cylinder = collision_bottle;
            bottle_feature.rpy.x = roll;
            bottle_feature.rpy.y = pitch;
            bottle_feature.rpy.z = yaw;
            bottle_feature_publisher.publish(bottle_feature);
            
            /***************************************
            *  setup tf transform from /pcl_camera to /bottle_frame
            ***************************************/
            tf::Transform tf_transform;
            tf::Quaternion tf_quaternion;
            //ROS_INFO("sending transform: %f, %f, %f, %f, %f, %f", x, y, z, roll, pitch, yaw);
            
            tf_transform.setOrigin( tf::Vector3(x,y,z) );
            tf_quaternion.setRPY(roll, pitch, yaw);
            tf_transform.setRotation( tf_quaternion );
            br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "pcl_camera", "bottle_frame")); // parent, then child frame
            
            /***************************************
             *  visualize scene with bottle coordinate
             ***************************************/        
            result_viewer->removeCoordinateSystem("reference");
            result_viewer->addCoordinateSystem(0.3, toTrackedBottleCoordinate.inverse(), "reference", 0);
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene_cloud);
            if(!result_viewer->updatePointCloud<RefPointType>(scene_cloud, rgb, "new frame"))
                result_viewer->addPointCloud<RefPointType>(scene_cloud, rgb, "new frame");
            result_viewer->spinOnce();
        } 
    }
}

int main(int argc, char **argv)
{
    if(argc<2)
    {
      printf("please input phrase: \n");
      printf("    test -- pure robot_scene_tracking test\n");
      printf("    run  -- run the entire framework\n");
      exit(0);
    }
    /***************************************
    *  ros node initialization
    ***************************************/
    ros::init(argc, argv, "robot_scene_tracking");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(20);   //loop rate for any loops appear in this cpp
    sleep(10.0); //wait for other nodes up and running in baxter_moveit/mvoe_group_server.launch
    
    /***************************************
    *  setup visualizer
    ***************************************/
    result_viewer->addCoordinateSystem(0.3, "reference", 0);
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
    palm_relfex_signal_subscriber = node_handle.subscribe("palm_reflex_triggered", 1, palm_reflex_callback);
    collision_object_publisher = node_handle.advertise<moveit_msgs::CollisionObject>("collision_object", 1);
    bottle_feature_publisher = node_handle.advertise<table_object::bottle_feature>("bottle_feature", 1);
    
    /***************************************
    *  setup fixed table msg
    ***************************************/
    collision_table.header.frame_id = "/base";
    collision_table.id = "table";
    
    /* Define a table to add to the world. */
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 0.7; //fixed X
    table_primitive.dimensions[1] = 2; //fixed Y
    table_primitive.dimensions[2] = 1.00; //fixed Z: higher than actual height for collision free motion

    /* A pose for the table (specified relative to frame_id) */
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 1;
    table_pose.position.y = 0.3;
    table_pose.position.z = -0.59;

    collision_table.primitives.push_back(table_primitive);
    collision_table.primitive_poses.push_back(table_pose); 
    collision_table.operation = collision_table.ADD;
    
    std::string option(argv[1]);
    std::string option_test("test");
    std::string option_run("run");
    if(option.compare(option_run)==0)
    {
        printf("waiting for listeners on topic /collision_object\n");
        while(collision_object_publisher.getNumSubscribers() < 1)
        {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();    
        }
    }else if(option.compare(option_test)==0){
        printf("pure test, no rviz, only pcl visualization\n");
    }else {
        printf("no such option %s\n", option.c_str());
        exit(0);
    }
    collision_object_publisher.publish(collision_table);
    sleep(2.0);
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
