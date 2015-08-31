/**
 * \file        extract_sec.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"
#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include "ros_sec/Visualizer/view2D.h"
#include "ros_sec/Visualizer/view3D.h"
#include "ros_sec/Tracker/track3D.h"
#include "ros_sec/Detector/colorDetector.h"
#include "ros_sec/Detector/touchDetector.h"
#include "ros_sec/Detector/bottleDetector.h"
#include "ros_sec/SEC/mainGraph.h"
#include "ros_sec/util/util.h"

#include "table_object/featureDef.h"

#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sys/stat.h>

// #include <ros/ros.h>

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

int
main (int argc, char** argv)
{
//     ros::init(argc, argv, "extract_sec");
//     ros::NodeHandle node_handle;

//     pcl::visualization::PCLVisualizer result_viewer("planar_segmentation");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer (new pcl::visualization::PCLVisualizer ("planar_segmentation"));
    result_viewer->addCoordinateSystem(0.3, "reference", 0);
    result_viewer->setCameraPosition(-0.499437, 0.111597, -0.758007, -0.443141, 0.0788583, -0.502855, -0.034703, -0.992209, -0.119654);
    result_viewer->setCameraClipDistances(0.739005, 2.81526);
//     result_viewer->setCameraPosition(Position, Focal point, View up);
//     result_viewer->setCameraClipDistances(Clipping plane);
    
    /***************************************
    *  parse arguments
    ***************************************/
    if(argc<5)
    {
        pcl::console::print_info("Usage: extract_sec DATA_PATH/PCD_FILE_FORMAT START_INDEX END_INDEX DEMO_NAME (opt)STEP_SIZE(1)");
        exit(1);
    }
    
    int view_id=0;
    int step=1;
    std::string basename_cloud=argv[1];
    unsigned int index_start = std::atoi(argv[2]);
    unsigned int index_end = std::atoi(argv[3]);
    std::string demo_name=argv[4];
    if(argc>5) step=std::atoi(argv[5]);
    
    /***************************************
    *  set up result directory
    ***************************************/
    mkdir("/home/zhen/Documents/Dataset/human_result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    char result_folder[50];
    std::snprintf(result_folder, sizeof(result_folder), "/home/zhen/Documents/Dataset/human_result/%s", demo_name.c_str());
    mkdir(result_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    std::string basename_pcd = (basename_cloud.find(".pcd") == std::string::npos) ? (basename_cloud + ".pcd") : basename_cloud;
    std::string filename_pcd;
    
    std::string mainGraph_file;
    mainGraph_file = std::string(result_folder) + "/mainGraph.txt";
    
    // write video config
    char video_file[100];
    std::snprintf(video_file, sizeof(video_file), "%s/video.txt", result_folder);
    std::ofstream video_config(video_file);
    if (video_config.is_open())
    {
        video_config << index_start << " " << index_end << " " << demo_name << " " << step;
        video_config.close();
    }
    
    /***************************************
    *  set up cloud, segmentation, tracker, detectors, graph, features
    ***************************************/
    TableObject::Segmentation tableObjSeg;
    TableObject::Segmentation initialSeg;
    TableObject::track3D tracker(false);
    TableObject::colorDetector finger1Detector(0,100,0,100,100,200); //right
    TableObject::colorDetector finger2Detector(150,250,0,100,0,100); //left
    TableObject::touchDetector touchDetector(0.01);
    TableObject::bottleDetector bottleDetector;
    
    TableObject::mainGraph mainGraph((int)index_start);
    
    std::vector<manipulation_features> record_features;
    manipulation_features cur_features;
    
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
    
    Eigen::Vector3f bottle_init_ori;
    
    // set threshold of size of clustered cloud
    tableObjSeg.setThreshold(30);
    initialSeg.setThreshold(500);
    
    // downsampler
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
    float leaf_size=0.005;
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    
    /***************************************
    *  start processing
    ***************************************/
    unsigned int idx = index_start;
    int video_id=0;
    bool change = false;
    while( idx <= index_end && !result_viewer->wasStopped())
    { 
        std::cout << std::endl;
        std::cout << "frame id=" << idx << std::endl;
        filename_pcd = cv::format(basename_cloud.c_str(), idx);
        
        if(idx==index_start){
            /***************************************
             *  Intialization:
             * -plane localization
             * -object cluster extraction
             * -bottle cluster localization
             ***************************************/
            initialSeg.resetCloud(filename_pcd, false);
            initialSeg.seg(false);
            initialSeg.getObjects(cloud_objects, clusters);
            initialSeg.getCloudHull(cloud_hull);
            initialSeg.getPlaneCoefficients(coefficients);
            
            initialSeg.getsceneCloud(pcdSceneCloud);
            initialSeg.getTableTopCloud(planeCloud);
            sceneCloud=pcdSceneCloud.getCloud();
                        
            /***************************************
             *  fingertip, hand_arm removal
             ***************************************/
            //opencv color filtering for fingertip_1
            {
                pcl::ScopeTime t_finger1("Finger 1(blue) detection");
                finger1Detector.setInputCloud(cloud_objects, clusters);
                finger1Detector.filter(f1_indices,cloud_finger1);
            }
            finger1Detector.showDetectedCloud(result_viewer, "finger1");
            
            //opencv color filtering for fingertip_2
            {
                pcl::ScopeTime t_finger2("Finger 2(orange) detection");
                finger2Detector.setInputCloud(cloud_objects, clusters);
                finger2Detector.filter(f2_indices,cloud_finger2);
            }
            finger2Detector.showDetectedCloud(result_viewer, "finger2");
            
            // remove hand (include cluster that contains the detected fingertips and also the other clusters that are touching the cluster)
            std::vector<int> hand_arm1=TableObject::findHand(cloud_objects, clusters, f1_indices);
            
            for(int i=hand_arm1.size()-1; i>=0; i--)
            {
                clusters.erase(clusters.begin()+hand_arm1[i]);
                std::cout << "removing hand_arm : cluster index = " << hand_arm1[i] << std::endl;
            }
            std::vector<int> hand_arm2=TableObject::findHand(cloud_objects, clusters, f2_indices);
            for(int i=hand_arm2.size()-1; i>=0; i--)
            {
                clusters.erase(clusters.begin()+hand_arm2[i]);
                std::cout << "removing hand_arm : cluster index = " << hand_arm2[i] << std::endl;
            }
            
// DEBUG
// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> plane(planeCloud);
// result_viewer->addPointCloud<RefPointType>(planeCloud, plane, "tabletop");
// CloudPtr debug(new Cloud);
// initialSeg.getOutPlaneCloud(debug);
// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> out_plane(debug);
// result_viewer->addPointCloud<RefPointType>(debug, out_plane, "out_plane");

// choose bottle_id at 1st frame & confirm fitted model is correct
TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters, true); 

while (!result_viewer->wasStopped ())
{
    result_viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
}
            
            std::cout << "cluster size = " << clusters.size() << std::endl;
            /***************************************
             *  Localizing cylinder
             ***************************************/
            CloudPtr cluster_bottle (new Cloud); 
            int bottle_id = 0;
            pcl::copyPointCloud (*cloud_objects, clusters[bottle_id], *cluster_bottle);
            bottleDetector.setInputCloud(cluster_bottle);
            bottleDetector.fit();
            bottleDetector.getTransformation(toBottleCoordinate);
            bottle_init_ori= bottleDetector.getOrientation();
            
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(toBottleCoordinate.inverse(), x, y, z, roll, pitch, yaw);
            
            result_viewer->removeCoordinateSystem("reference", 0);
            result_viewer->addCoordinateSystem(0.3, toBottleCoordinate.inverse(), "reference", 0);
            bottleDetector.drawOrientation(result_viewer);
            
            /***************************************
             *  Record features
             ***************************************/
            bottle_features cur_bottle_features;
            cur_bottle_features.loc[0] = x;
            cur_bottle_features.loc[1] = y;
            cur_bottle_features.loc[2] = z;
            cur_bottle_features.ori[0] = roll;
            cur_bottle_features.ori[1] = pitch;
            cur_bottle_features.ori[2] = yaw;
            cur_bottle_features.color[0] = bottleDetector.getCenter().r;
            cur_bottle_features.color[1] = bottleDetector.getCenter().g;
            cur_bottle_features.color[2] = bottleDetector.getCenter().b;
            cur_bottle_features.size[0] = bottleDetector.getHeight();
            cur_bottle_features.size[1] = bottleDetector.getRadius();
            cur_features.bottle = cur_bottle_features;
            
            pcl::PointXYZ center_finger1 = TableObject::computeObjCentroid(cloud_finger1);
            pcl::PointXYZ center_finger2 = TableObject::computeObjCentroid(cloud_finger2);
            center_finger1 = pcl::transformPoint<pcl::PointXYZ>(center_finger1, toBottleCoordinate);
            center_finger2 = pcl::transformPoint<pcl::PointXYZ>(center_finger2, toBottleCoordinate);
            cur_features.gripper_1.loc[0] = center_finger1.x;
            cur_features.gripper_1.loc[1] = center_finger1.y;
            cur_features.gripper_1.loc[2] = center_finger1.z;
            cur_features.gripper_2.loc[0] = center_finger2.x;
            cur_features.gripper_2.loc[1] = center_finger2.y;
            cur_features.gripper_2.loc[2] = center_finger2.z;
            
            record_features.push_back(cur_features);
            
            /***************************************
             *  Tracking initialization
             ***************************************/
            {
                pcl::ScopeTime t_track("Tracker initialization");
                tracker.setTarget(cluster_bottle, bottleDetector.getCenter());
                tracker.initialize();
            }
            
            /***************************************
             *  Touch detection
             ***************************************/
            std::vector<CloudPtr> touch_clouds;
            touch_clouds.push_back(cluster_bottle);
            touch_clouds.push_back(cloud_finger1);
            touch_clouds.push_back(cloud_finger2);

            // touch detection between each pair of objects (including fingertips, tabletop objects and tabletop)
            for(int i=0; i<touch_clouds.size(); i++)
            {
                int j;
                bool touch;
                for(j=i+1; j<touch_clouds.size(); j++)
                {                
                    // touch detection between object_i and object_j
                    char relation [50];
                    std::sprintf(relation, "object%d_object%d", i, j);
                    std::cout << relation << std::endl;

                    {
                        pcl::ScopeTime t("Touch detection");
                        touch=touchDetector.detect(touch_clouds[i], touch_clouds[j]);
                    }
//                     touchDetector.showTouch(result_viewer, relation, 100+250*(j-i-1), 40+20*i);
                    
                    // relational scene graph -> main graph
                    if(touch) {
                        mainGraph.addInitialRelationalGraph(2);
                    }else{
                        mainGraph.addInitialRelationalGraph(0);
                    }
                }
                
                // touch detection between each objects and tabletop
                char relation [50];
                std::sprintf (relation, "object%d_object%d", i, (int)touch_clouds.size());
                std::cout << relation << std::endl;
                {
                    pcl::ScopeTime t("Touch detection");
                    touch=touchDetector.detectTableTouch(touch_clouds[i], coefficients);
                }
//                 touchDetector.showTouch(result_viewer, relation, 100+250*(j-i-1), 40+20*i);
                
                // relational scene graph -> main graph
                if(touch) {
                    mainGraph.addInitialRelationalGraph(2);
                }else{
                    mainGraph.addInitialRelationalGraph(0);
                }
            }
             
            /***************************************
             *  Visualization
             ***************************************/              
            // draw extracted object clusters
//             TableObject::view3D::drawClusters(result_viewer, cloud_objects, touch_clusters);  

            // draw extracted plane points
//             pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> plane(planeCloud);
//             result_viewer->addPointCloud<RefPointType>(planeCloud, plane, "tabletop");
//             std::stringstream ss; 
//             ss << (int)touch_clusters.size();
//             result_viewer->addText3D(ss.str(), planeCloud->points.at(334*640+78),0.1);
            
            // draw extracted plane contour polygon
            result_viewer->addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
            
            change = true;
        }else
        {            
            /***************************************
             *  object cloud extraction
             ***************************************/
            tableObjSeg.resetCloud(filename_pcd, false);
            tableObjSeg.seg(cloud_hull,false);
            tableObjSeg.getObjects(cloud_objects, clusters);
            tableObjSeg.getsceneCloud(pcdSceneCloud);
            sceneCloud=pcdSceneCloud.getCloud();            
            
            /***************************************
             *  fingertip extraction
             ***************************************/
            //opencv color filtering for fingertip_1
            {
                pcl::ScopeTime t_finger1("Finger 1(blue) detection");
                finger1Detector.setInputCloud(cloud_objects, clusters);
                finger1Detector.filter(f1_indices,cloud_finger1);
            }
            finger1Detector.showDetectedCloud(result_viewer, "finger1");
            
            //opencv color filtering for fingertip_2
            {
                pcl::ScopeTime t_finger1("Finger 2(orange) detection");
                finger2Detector.setInputCloud(cloud_objects, clusters);
                finger2Detector.filter(f2_indices,cloud_finger2);
            }
            finger2Detector.showDetectedCloud(result_viewer, "finger2");
            
            /***************************************
             *  filter out black glove cluster & gray sleeve, also update cloud_objects with removed cluster indices
             ***************************************/
            for(int i=0; i<clusters.size(); i++)
            {
                pcl::CentroidPoint<RefPointType> color_points;
                for(int j=0; j<clusters[i].indices.size();j++)
                {
                    color_points.add(cloud_objects->at(clusters[i].indices[j]));
                }
                RefPointType mean_color;
                color_points.get(mean_color);
                if(mean_color.r>30 & mean_color.r<70 & mean_color.g>30 & mean_color.g<70 & mean_color.b>30 & mean_color.b<70)
                {
                    clusters.erase(clusters.begin()+ i);
                    i=i-1;
                }
            }
            
            /***************************************
             *  Tracking objects
             ***************************************/
            {
                pcl::ScopeTime t_track("Tracking");
                grid.setInputCloud (sceneCloud);
                grid.filter (*track_target);
                tracker.track(track_target, transformation);
                tracker.getTrackedCloud(tracked_cloud);
            }
            tracker.viewTrackedCloud(result_viewer);
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
            result_viewer->removeCoordinateSystem("reference");
            result_viewer->addCoordinateSystem(0.3, toTrackedBottleCoordinate.inverse(), "reference", 0);
            
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(toTrackedBottleCoordinate.inverse(), x, y, z, roll, pitch, yaw);
            
            /***************************************
            *  setup bottle feature
            ***************************************/
            cur_features = record_features[video_id-1];
            cur_features.bottle.loc[0] = x;
            cur_features.bottle.loc[1] = y;
            cur_features.bottle.loc[2] = z;
            cur_features.bottle.ori[0] = roll;
            cur_features.bottle.ori[1] = pitch;
            cur_features.bottle.ori[2] = yaw;
            
            pcl::PointXYZ center_finger1 = TableObject::computeObjCentroid(cloud_finger1);
            pcl::PointXYZ center_finger2 = TableObject::computeObjCentroid(cloud_finger2);
            center_finger1 = pcl::transformPoint<pcl::PointXYZ>(center_finger1, toTrackedBottleCoordinate);
            center_finger2 = pcl::transformPoint<pcl::PointXYZ>(center_finger2, toTrackedBottleCoordinate);
            cur_features.gripper_1.loc[0] = center_finger1.x;
            cur_features.gripper_1.loc[1] = center_finger1.y;
            cur_features.gripper_1.loc[2] = center_finger1.z;
            cur_features.gripper_2.loc[0] = center_finger2.x;
            cur_features.gripper_2.loc[1] = center_finger2.y;
            cur_features.gripper_2.loc[2] = center_finger2.z;
            
            record_features.push_back(cur_features);
            
            
            
            /***************************************
             *  Touch detection
             ***************************************/
            std::vector<CloudPtr> touch_clouds;
            touch_clouds.push_back(tracked_cloud);
            touch_clouds.push_back(cloud_finger1);
            touch_clouds.push_back(cloud_finger2);

            // touch detection between each pair of objects (including fingertips, tabletop objects and tabletop)
            for(int i=0; i<touch_clouds.size(); i++)
            {
                int j;
                bool touch;
                for(j=i+1; j<touch_clouds.size(); j++)
                {                
                    // touch detection between object_i and object_j
                    char relation [50];
                    std::sprintf(relation, "object%d_object%d", i, j);
                    std::cout << relation << std::endl;

                    {
                        pcl::ScopeTime t("Touch detection");
                        touch=touchDetector.detect(touch_clouds[i], touch_clouds[j]);
                    }
//                     touchDetector.showTouch(result_viewer, relation, 100+250*(j-i-1), 40+20*i);
                    
                    // relational scene graph -> main graph
                    if(touch) {
                        mainGraph.addRelationalGraph(2);
                    }else{
                        mainGraph.addRelationalGraph(0);
                    }
                }
                
                // touch detection between each objects and tabletop
                char relation [50];
                std::sprintf (relation, "object%d_object%d", i, (int)touch_clouds.size());
                std::cout << relation << std::endl;
                {
                    pcl::ScopeTime t("Touch detection");
                    touch=touchDetector.detectTableTouch(touch_clouds[i], coefficients);
                }
//                 touchDetector.showTouch(result_viewer, relation, 100+250*(j-i-1), 40+20*i);
                
                // relational scene graph -> main graph
                if(touch) {
                    mainGraph.addRelationalGraph(2);
                }else{
                    mainGraph.addRelationalGraph(0);
                }
            }
            
            /***************************************
             *  Visualization
             ***************************************/
            // draw extracted point clusters
//             TableObject::view3D::drawText(result_viewer, cloud_objects, touch_clusters);
            
            /***************************************
             *  Main Graph
             ***************************************/
            change = mainGraph.compareRelationGraph((int)idx);
        }       

            // darw original cloud
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
            if(!result_viewer->updatePointCloud<RefPointType>(sceneCloud, rgb, "new frame"))
                result_viewer->addPointCloud<RefPointType>(sceneCloud, rgb, "new frame");

        result_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        
        
        //debug
        std::cout << cur_features.bottle.loc[0] << " " << cur_features.bottle.loc[1] << " " << cur_features.bottle.loc[2]
                << " " << cur_features.bottle.ori[0] << " " << cur_features.bottle.ori[1] << " " << cur_features.bottle.ori[2]
                << " " << cur_features.bottle.color[0] << " " << cur_features.bottle.color[1] << " " << cur_features.bottle.color[2]
                << " " << cur_features.bottle.size[0] << " " << cur_features.bottle.size[1]
                << " " << cur_features.gripper_1.loc[0] << " " << cur_features.gripper_1.loc[1] << " " << cur_features.gripper_1.loc[2]
                << " " << cur_features.gripper_2.loc[0] << " " << cur_features.gripper_2.loc[1] << " " << cur_features.gripper_2.loc[2]
                << std::endl;
        
        if(change)
        {
            char screenshot[100]; // make sure it's big enough
            std::snprintf(screenshot, sizeof(screenshot), "%s/sec_%d.png", result_folder, (int)idx);
            std::cout << screenshot << std::endl;
            result_viewer->saveScreenshot(screenshot);
            
            //record features
            char feature_file[100]; // make sure it's big enough
            std::snprintf(feature_file, sizeof(feature_file), "%s/features_original.txt", result_folder);
            std::ofstream feature_writer(feature_file, std::ofstream::out | std::ofstream::app);
            feature_writer << cur_features.bottle.loc[0] << " " << cur_features.bottle.loc[1] << " " << cur_features.bottle.loc[2]
                    << " " << cur_features.bottle.ori[0] << " " << cur_features.bottle.ori[1] << " " << cur_features.bottle.ori[2]
                    << " " << cur_features.bottle.color[0] << " " << cur_features.bottle.color[1] << " " << cur_features.bottle.color[2]
                    << " " << cur_features.bottle.size[0] << " " << cur_features.bottle.size[1]
                    << " " << cur_features.gripper_1.loc[0] << " " << cur_features.gripper_1.loc[1] << " " << cur_features.gripper_1.loc[2]
                    << " " << cur_features.gripper_2.loc[0] << " " << cur_features.gripper_2.loc[1] << " " << cur_features.gripper_2.loc[2]
                    << std::endl;
            feature_writer.close();
            std::cout << "features saved at " << feature_file << std::endl;
        }
        
        char screenshot[200]; // make sure it's big enough
        std::snprintf(screenshot, sizeof(screenshot), "%s/video/sec_%d.png", result_folder, (int)video_id);
        std::cout << screenshot << std::endl;
        result_viewer->saveScreenshot(screenshot);
        
        idx=idx+step;
        video_id=video_id+1;
    }
    
    mainGraph.displayMainGraph();
    mainGraph.recordMainGraph(mainGraph_file);
    
    
    while (!result_viewer->wasStopped ())
    {
        result_viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}
