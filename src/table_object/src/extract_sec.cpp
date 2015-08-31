/**
 * \file        extract_sec.cpp
 * \author      Zhen Zeng (zengzhen@umich.edu) 
 */

#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"
#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include "ros_sec/Visualizer/view2D.h"
#include "ros_sec/Visualizer/view3D.h"
#include "ros_sec/Tracker/trackRigid.h"
#include "ros_sec/util/util.h"
#include "ros_sec/Detector/colorDetector.h"
#include "ros_sec/Detector/touchDetector.h"
#include "ros_sec/SEC/mainGraph.h"

#include <pcl/filters/conditional_removal.h>

#include <sys/stat.h>

#include <ros/ros.h>

#include <time.h>

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

int
main (int argc, char** argv)
{
    ros::init(argc, argv, "extract_sec");
    ros::NodeHandle node_handle;

    pcl::visualization::PCLVisualizer result_viewer("planar_segmentation");
    result_viewer.addCoordinateSystem();
    result_viewer.setCameraPosition(-0.504623,0.0647437,-0.758519, -0.443141,0.0788583,-0.502855, 0.00533475,-0.998535,0.0538437);
    result_viewer.setCameraClipDistances(0.0136198,13.6198);
    //2.8494,8.48591/0.0712695,0.486438,0.865/-1.77289,1.38452,-4.06431/0.0224361,-0.982044,-0.187315/0.523599/1920,600/1921,52
    //clipDistance  / pos x, y, z              / view x, y, z            / up x, y, z                  / fovy   /win_size/win_pos
    //0.0136198,13.6198/-0.443141,0.0788583,-0.502855/-0.504623,0.0647437,-0.758519/0.00533475,-0.998535,0.0538437/0.523599/800,450/425,277
    
    /***************************************
    *  parse arguments
    ***************************************/
    if(argc<5)
    {
        ROS_INFO("Usage: extract_sec DATA_PATH/PCD_FILE_FORMAT START_INDEX END_INDEX DEMO_NAME (opt)STEP_SIZE(1)");
        exit(1);
    }
    
    int view_id=-1;
    int step=1;
    std::string basename_cloud=argv[1];
    unsigned int index_start = std::atoi(argv[2]);
    unsigned int index_end = std::atoi(argv[3]);
    std::string demo_name=argv[4];
    if(argc>5) step=std::atoi(argv[5]);
//     if(argc>4) view_id=std::atoi(argv[4]);
//     if(argc>5) VERBOSE = (bool)argv[5];
    
    /***************************************
    *  set up result directory
    ***************************************/
    mkdir("../../../result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    char result_folder[50];
    std::snprintf(result_folder, sizeof(result_folder), "../../../result/%s", demo_name.c_str());
    mkdir(result_folder, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    std::string basename_pcd = (basename_cloud.find(".pcd") == std::string::npos) ? (basename_cloud + ".pcd") : basename_cloud;
    std::string filename_pcd;
    
//     boost::filesystem::path p(basename_cloud);
//     boost::filesystem::path dir=p.parent_path();
//     std::string mainGraph_file = dir.string();
//     mainGraph_file = mainGraph_file + "mainGraph.txt";
    
    std::string mainGraph_file;
    mainGraph_file = "../../../result/" + demo_name + "/mainGraph.txt";
    
    char video_file[50];
    std::snprintf(video_file, sizeof(video_file), "../../../result/%s/video.txt", demo_name.c_str());
    std::ofstream video_config(video_file);
    if (video_config.is_open())
    {
        video_config << index_start << " " << index_end << " " << demo_name << " " << step;
        video_config.close();
    }
    
    
    /***************************************
    *  set up segmentation, detectors, graph
    ***************************************/
    TableObject::Segmentation tableObjSeg;
    TableObject::Segmentation initialSeg;
    TableObject::trackRigid tracker;
    TableObject::view2D view2D;
    TableObject::colorDetector finger1Detector(0,100,0,100,100,200);
    TableObject::colorDetector finger2Detector(150,250,0,100,0,100);
    TableObject::colorDetector cupDetector(100,255,100,255,0,100);
    TableObject::colorDetector blockDetector(100,150,140,210,50,120);
    TableObject::touchDetector touchDetector(0.015);
    
    TableObject::mainGraph mainGraph((int)index_start);
    
    TableObject::pcdCloud pcdSceneCloud;
    CloudPtr sceneCloud;
    CloudPtr cloud_objects(new Cloud);
    CloudPtr cloud_finger1(new Cloud);
    CloudPtr cloud_finger2(new Cloud);
    CloudPtr cloud_cup(new Cloud);
    CloudPtr cloud_block(new Cloud);
    CloudPtr cloud_hull(new Cloud);
    std::vector<pcl::PointIndices> clusters;
    pcl::ModelCoefficients coefficients;
    
    tableObjSeg.setThreshold(10);
    initialSeg.setThreshold(200);
    
    pcl::PointIndices f1_indices;
    pcl::PointIndices f2_indices;
    pcl::PointIndices object_indices;
    
    
    /***************************************
    *  start processing
    ***************************************/
    unsigned int idx = index_start;
    int video_id=0;
    bool change = false;
    while( idx <= index_end && !result_viewer.wasStopped())
    { 
        std::cout << std::endl;
        std::cout << "frame id=" << idx << std::endl;
        filename_pcd = cv::format(basename_cloud.c_str(), idx);
        
        CloudPtr planeCloud(new Cloud);
        
        if(idx==index_start){
            /***************************************
             *  object cloud extraction
             ***************************************/
            initialSeg.resetCloud(filename_pcd, false);
            std::clock_t t = std::clock();
            initialSeg.seg(false);
            initialSeg.getObjects(cloud_objects, clusters);
            initialSeg.getCloudHull(cloud_hull);
            initialSeg.getPlaneCoefficients(coefficients);
            t = std::clock() - t;
            ROS_INFO("First frame: Plane segmentation + Object Clustering: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            
            initialSeg.getsceneCloud(pcdSceneCloud);
            initialSeg.getTableTopCloud(planeCloud);
            sceneCloud=pcdSceneCloud.getCloud();
            
            
            /***************************************
             *  fingertip, hand_arm extraction
             ***************************************/
            t = std::clock();
            
//             int rl=0, rh=100, gl=0, gh=100, bl=100, bh=200;
//             pcl::ConditionOr<RefPointType>::Ptr finger1_cond(new pcl::ConditionOr<RefPointType>);
//             // GT: >GT, LT: <LT
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("r",pcl::ComparisonOps::GT, rh)));
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("r",pcl::ComparisonOps::LT, rl)));
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("g",pcl::ComparisonOps::GT, gh)));
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("g",pcl::ComparisonOps::LT, gl)));
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("b",pcl::ComparisonOps::GT, bh)));
//             finger1_cond->addComparison(pcl::PackedRGBComparison<RefPointType>::Ptr (new pcl::PackedRGBComparison<RefPointType>("b",pcl::ComparisonOps::LT, bl)));
//             
//             pcl::ConditionalRemoval<RefPointType> color_rem;
//             color_rem.setCondition(finger1_cond);
//             color_rem.setInputCloud(sceneCloud);
//             color_rem.setKeepOrganized(true);
//             color_rem.filter(*cloud_finger1);
//             f1_indices.indices = *color_rem.getIndices();

            //opencv color filtering for fingertip_1
            finger1Detector.setInputCloud(cloud_objects, clusters);
            finger1Detector.filter(f1_indices,cloud_finger1);
            t = std::clock() - t;
            ROS_INFO("Finger 1 detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            finger1Detector.showDetectedCloud(result_viewer, "finger1");
            
            //opencv color filtering for fingertip_2
            t = std::clock();
            finger2Detector.setInputCloud(cloud_objects, clusters);
            finger2Detector.filter(f2_indices,cloud_finger2);
            t = std::clock() - t;
            ROS_INFO("finger 2 detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            finger2Detector.showDetectedCloud(result_viewer, "finger2");
            
            //opencv color filtering for block
//             cupDetector.setInputCloud(cloud_objects, clusters);
//             cupDetector.filter(object_indices, cloud_block);
//             cupDetector.showDetectedCloud(result_viewer, "block");
            
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
            
            /***************************************
             *  Tracking initialization
             ***************************************/
            t = std::clock();
            tracker.init(cloud_objects, clusters);
            t = std::clock() - t;
            ROS_INFO("tracker initialization: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            
            /***************************************
             *  Touch detection
             ***************************************/
            
            std::vector<pcl::PointIndices> touch_clusters=clusters;
            touch_clusters.push_back(f1_indices);
            touch_clusters.push_back(f2_indices);
            std::cout << "touch_clusters size = " << touch_clusters.size() << std::endl;
            // touch detection between each pair of objects (including fingertips, tabletop objects and tabletop)
            for(int i=0; i<touch_clusters.size(); i++)
            {
                CloudPtr object_i(new Cloud);
                pcl::copyPointCloud(*cloud_objects, touch_clusters[i], *object_i);
                int j;
                bool touch;
                for(j=i+1; j<touch_clusters.size(); j++)
                {
                    CloudPtr object_j(new Cloud);
                    pcl::copyPointCloud(*cloud_objects, touch_clusters[j], *object_j);
                
                    // touch detection between object_i and object_j
                    char relation [50];
                    std::sprintf(relation, "object%d_object%d", i, j);
                    std::cout << relation << std::endl;
                    t = std::clock();
                    touch=touchDetector.detect(object_i, object_j);
                    t = std::clock() - t;
                    ROS_INFO("touch detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
                    
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
                std::sprintf (relation, "object%d_object%d", i, (int)touch_clusters.size());
                std::cout << relation << std::endl;
                t = std::clock();
                touch=touchDetector.detectTableTouch(object_i, coefficients);
                t = std::clock() - t;
                ROS_INFO("touch detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
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
            // darw original cloud
//             pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
//             result_viewer.addPointCloud<RefPointType>(sceneCloud, rgb, "new frame");
            
            // draw extracted object clusters
            TableObject::view3D::drawClusters(result_viewer, cloud_objects, touch_clusters);  

            // draw extracted plane points
//             pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> plane(planeCloud);
//             result_viewer.addPointCloud<RefPointType>(planeCloud, plane, "tabletop");
//             std::stringstream ss; 
//             ss << (int)touch_clusters.size();
//             result_viewer.addText3D(ss.str(), planeCloud->points.at(334*640+78),0.1);
            
            // draw extracted plane contour polygon
            result_viewer.addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon");
            
            change = true;
        }else
        {            
            /***************************************
             *  object cloud extraction
             ***************************************/
            tableObjSeg.resetCloud(filename_pcd, false);
            std::clock_t t = std::clock();
            tableObjSeg.seg(cloud_hull,false);
            tableObjSeg.getObjects(cloud_objects, clusters);
            t = std::clock() - t;
            ROS_INFO("Object Clustering: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            tableObjSeg.getsceneCloud(pcdSceneCloud);
            sceneCloud=pcdSceneCloud.getCloud();
            
            
            /***************************************
             *  fingertip extraction
             ***************************************/
            //opencv color filtering for fingertip_1
            t = std::clock();
            finger1Detector.setInputCloud(cloud_objects, clusters);
            finger1Detector.filter(f1_indices,cloud_finger1);
            t = std::clock() - t;
            ROS_INFO("finger 1 detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            finger1Detector.showDetectedCloud(result_viewer, "finger1");

            //opencv color filtering for fingertip_2
            t = std::clock();
            finger2Detector.setInputCloud(cloud_objects, clusters);
            finger2Detector.filter(f2_indices,cloud_finger2);
            t = std::clock() - t;
            ROS_INFO("finger 2 detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
            finger2Detector.showDetectedCloud(result_viewer, "finger2");
            
            /***************************************
             *  Tracking objects
             ***************************************/
            t = std::clock();
            std::vector<pcl::PointIndices> tracked_clusters;
            tracker.linking(cloud_objects, clusters);
            tracker.track(cloud_objects,tracked_clusters);
            t = std::clock() - t;
            ROS_INFO("tracking: %f seconds", ((float)t)/CLOCKS_PER_SEC);
//             tracker.viewTranformedCloud(result_viewer, view_id);
            for(int i=0; i<tracked_clusters.size(); i++)
            {
                tracker.viewTrackedCloud(result_viewer, i, 255, 0, 0);
            }
            
            /***************************************
             *  Touch detection
             ***************************************/
            std::vector<pcl::PointIndices> touch_clusters=tracked_clusters;
            touch_clusters.push_back(f1_indices);
            touch_clusters.push_back(f2_indices);
            std::cout << "touch_clusters size = " << touch_clusters.size() << std::endl;
            // touch detection between each pair of objects (including fingertips, tabletop objects and tabletop)
            for(int i=0; i<touch_clusters.size(); i++)
            {
                CloudPtr object_i(new Cloud);
                pcl::copyPointCloud(*cloud_objects, touch_clusters[i], *object_i);
                int j;
                bool touch;
                for(j=i+1; j<touch_clusters.size(); j++)
                {
                    CloudPtr object_j(new Cloud);
                    pcl::copyPointCloud(*cloud_objects, touch_clusters[j], *object_j);
                
                    // touch detection between object_i and object_j
                    char relation [50];
                    std::sprintf(relation, "object%d_object%d", i, j);
                    std::cout << relation << std::endl;
                    t = std::clock();
                    touch=touchDetector.detect(object_i, object_j);
                    t = std::clock() - t;
                    ROS_INFO("touch detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
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
                std::sprintf (relation, "object%d_object%d", i, (int)touch_clusters.size());
                std::cout << relation << std::endl;
                t = std::clock();
                touch=touchDetector.detectTableTouch(object_i, coefficients);
                t = std::clock() - t;
                ROS_INFO("touch detection: %f seconds", ((float)t)/CLOCKS_PER_SEC);
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
            TableObject::view3D::drawText(result_viewer, cloud_objects, touch_clusters);
            
            /***************************************
             *  Main Graph
             ***************************************/
            change = mainGraph.compareRelationGraph((int)idx);
        }       
        
             
//         result_viewer.setRepresentationToSurfaceForAllActors();
//         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
//         result_viewer.updatePointCloud<RefPointType>(sceneCloud, rgb, "new frame");

        result_viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        
        if(change)
        {
            char screenshot[50]; // make sure it's big enough
            std::snprintf(screenshot, sizeof(screenshot), "../../../result/%s/sec_%d.png", demo_name.c_str(), (int)video_id);
            std::cout << screenshot << std::endl;
            result_viewer.saveScreenshot(screenshot);
        }
        
        idx=idx+step;
        video_id=video_id+1;
    }
    
    mainGraph.displayMainGraph();
    mainGraph.recordMainGraph(mainGraph_file);
    
    
    while (!result_viewer.wasStopped ())
    {
        result_viewer.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
}
