/**
 * \file save_cluster.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief save a selected cluster of points into pcd file
 */

#include <execinfo.h>
#include <signal.h>

#include <sys/stat.h>
#include <math.h>
#include <time.h>

//ros
#include <ros/ros.h>

// msgs
#include <sensor_msgs/PointCloud2.h>

// boost
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/bind.hpp>

// pcl
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Core>

// pointcloud2 to ros msgs conversions
#include <pcl_conversions/pcl_conversions.h>

// from ros_sec
#include "ros_sec/TableObjectSegmentation/table_obj_seg.h"
#include "ros_sec/TableObjectSegmentation/pcd_cloud.h"
#include "ros_sec/Detector/colorDetector.h"
#include "ros_sec/Detector/bottleDetector.h"
#include "ros_sec/Visualizer/view2D.h"
#include "ros_sec/Visualizer/view3D.h"
#include "ros_sec/Tracker/track3D.h"
#include "ros_sec/Tracker/trackModel.h"
#include "ros_sec/util/util.h"
// #include "ros_sec/util/extract_clusters.h"
#include <tf/tf.h>

#include "table_object/miniBall.hpp"

#include <X11/Xlib.h>

std::stringstream tracked_datafile; 

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;
bool MULTIPLE_VIEWPORTS = true;

typedef float* const* PointIterator; 
typedef const float* CoordIterator;
typedef Miniball::Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> > MB;

// publisher, subscriber
ros::Subscriber scene_subscriber;

// pcl::visualization::PCLVisualizer result_viewer("table");
boost::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer (new pcl::visualization::PCLVisualizer ("table-object"));
int v1(0);
int v2(0);

// set up variables for tabletop localization;
TableObject::Segmentation initialSeg;
TableObject::pcdCloud pcdSceneCloud;
CloudPtr sceneCloud(new Cloud);
CloudPtr cloud_objects(new Cloud);
CloudPtr cloud_hull(new Cloud);
CloudPtr cluster_hull(new Cloud);
CloudPtr track_target(new Cloud);
CloudPtr tracked_cloud(new Cloud);
CloudPtr cloud_finger1(new Cloud);
CloudPtr cloud_finger2(new Cloud);
            

// TableObject::colorDetector finger1Detector(0,80,40,70,50,120); //right: dark blue
// TableObject::colorDetector finger2Detector(120,170,30,70,0,100); //left

TableObject::colorDetector finger1Detector(75,160,60,160,10,70); //right: bright yellow
TableObject::colorDetector finger2Detector(50,110,10,50, 0,90); //right: dark red

pcl::PointIndices f1_indices;
pcl::PointIndices f2_indices;

std::vector<pcl::PointIndices> clusters;

TableObject::bottleDetector bottleDetector;
std::vector<TableObject::track3D> finger_tracker;
std::vector<TableObject::track3D> cylinder_tracker;
std::vector<TableObject::track3D> block_tracker;

std::vector<TableObject::trackModel> model_tracker;
int focus_index = 0;

Eigen::Affine3f toPlaneCoordinate;
std::vector<Eigen::Affine3f> camToFingerCoordinate;
std::vector<Eigen::Affine3f> camToCylinderCoordinate;
std::vector<Eigen::Affine3f> camToBlockCoordinate;
std::vector<Eigen::Affine3f> planeToBlockCoordinate;
Eigen::Affine3f toGripperCoordinate;
Eigen::Affine3f transformation;
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
float leaf_size=0.01;

int counter=0;
int selected_id;
int initial_cluster_size;

bool beginning = true;

std::vector<CloudPtr> fingerWireframe;
CloudPtr fingerWireframeInCamCoor(new Cloud);
std::vector<double> finger_radius;
std::vector<double> finger_height;

std::vector<CloudPtr> cylinderWireframe;
CloudPtr cylinderWireframeInCamCoor(new Cloud);

std::vector<float> block_w;
std::vector<float> block_d;
std::vector<float> block_h;
std::vector<RefPointType> block_model_color;
// float block_w, block_d, block_h;

std::vector<int> cylinder_index;
std::vector<int> block_index;

std::vector<RefPointType> cylinder_center_point;

boost::mutex updateModelMutex;

Eigen::Vector3f gripper_center;
Eigen::Vector3f gripper_ori;

void handler(int sig) {
  void *array[10];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 10);

  // print out all the frames to stderr
  fprintf(stderr, "Error: signal %d:\n", sig);
  backtrace_symbols_fd(array, size, STDERR_FILENO);
  exit(1);
}

static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static bool sort_using_greater_than(float u, float v)
{
   return u > v;
}

bool pause_for_observation = false;
bool wireframe_on = true;
bool particle_on = false;
int particle_index = -1;
float particle_weight = 0.0;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    
    if (event.getKeySym () == "i" && event.keyDown ())
    {
        if(pause_for_observation)
        {
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle" % particle_index), v2);
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
            std::cout << "resume process" << std::endl;
            pause_for_observation = false;
			
// 			//save finger2 point cloud for dubugging
// 			std::cout << "finger2 cylinder radius = " << finger_radius[1] << std::endl;
// 			std::cout << "finger2 cylinder height = " << finger_height[1] << std::endl;
// 			pcl::PCDWriter writer;
// 			writer.write ("cloud_finger2.pcd", *cloud_finger2, false);
// 			exit(1);
        }else{
            std::cout << "pause process" << std::endl;
            pause_for_observation = true;
        }
            
    }else if (event.getKeySym () == "t" && event.keyDown () )
    {
        //boost::mutex::scoped_lock updateLock(updateModelMutex);

// std::cout << "keypress t event occured\n";
        
//boost::this_thread::sleep (boost::posix_time::seconds(2));
// sleep(2);

        if(wireframe_on)
        {
            wireframe_on = false;
            
            for(int i=0; i<camToBlockCoordinate.size(); i++)
            {
                std::stringstream block_wireframe_id; 
                block_wireframe_id << "block_wireframe" <<  i;
                viewer->removeShape(block_wireframe_id.str(), v2);
            }
           
            for(int i=0; i<camToCylinderCoordinate.size(); i++)
            {
                std::stringstream cylinder_wireframe_id; 
                cylinder_wireframe_id << "cylinder_wireframe" << i;
                viewer->removePointCloud(cylinder_wireframe_id.str(), v2);
            }
        }else{
            wireframe_on = true;
            TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate, block_w, block_d, block_h, viewer,v2);
            TableObject::view3D::drawCylinderWireframe(camToCylinderCoordinate, cylinderWireframe, viewer, "cylinder", v2);
        }
// std::cout << "keypress t event ended\n";
        //updateLock.unlock();
        
    }else if (event.getKeySym () == "b" && event.keyDown () )
    {
        particle_on = true;
        viewer->removeShape(boost::str(boost::format("%s%d") % "particle" % particle_index), v2);
        viewer->removeShape(boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
        
        char* user_input = new char[10];
        printf("Which particle do you want to visualize(0-%d)\n", model_tracker[focus_index].getParticlesNum()-1);
        std::cin >> user_input;
        particle_index = atoi(user_input);
        
        particle_weight = model_tracker[focus_index].drawParticleHypothesis(particle_index, viewer, v2);
        viewer->addText( boost::str(boost::format("Particle[%d]: %f") % particle_index %particle_weight), 10, 40, 20, 1.0, 1.0, 1.0, 
                             boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2); 

    }else if (event.getKeySym() == "y" && event.keyDown())
    {
        if(particle_on)
        {
            particle_on = false;
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle" % particle_index), v2);
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
        }else if(particle_index!=-1){
            particle_on = true;
            model_tracker[focus_index].drawParticleHypothesis(particle_index, viewer, v2);
            viewer->addText( boost::str(boost::format("Particle[%d]: %f") % particle_index %particle_weight), 10, 40, 20, 1.0, 1.0, 1.0, 
                             boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
        }
    }else if (event.getKeySym () == "n" && event.keyDown () )
    {
        if(particle_on)
        {
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle" % particle_index), v2);
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
        
            particle_index = std::max(particle_index-1, 0);
            
            particle_weight = model_tracker[focus_index].drawParticleHypothesis(particle_index, viewer, v2);
            viewer->addText( boost::str(boost::format("Particle[%d]: %f") % particle_index %particle_weight), 10, 40, 20, 1.0, 1.0, 1.0, 
                             boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);   
        }
        
    }else if (event.getKeySym () == "m" && event.keyDown () )
    {
        if(particle_on)
        {
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle" % particle_index), v2);
            viewer->removeShape(boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);
        
            particle_index = std::min(particle_index+1, model_tracker[focus_index].getParticlesNum()-1);
            
            particle_weight = model_tracker[focus_index].drawParticleHypothesis(particle_index, viewer, v2);
            viewer->addText( boost::str(boost::format("Particle[%d]: %f") % particle_index %particle_weight), 10, 40, 20, 1.0, 1.0, 1.0, 
                             boost::str(boost::format("%s%d") % "particle_info" % particle_index), v2);   
        }   
    }
}


int main(int argc, char **argv)
{
    signal(SIGSEGV, handler);
    /***************************************
    *  setup visualizer
    ***************************************/
    result_viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&result_viewer);
    result_viewer->setSize(1000, 560);
    result_viewer->setPosition(400, 200);
    
    int viewport;
    if(MULTIPLE_VIEWPORTS)
    {
        result_viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        result_viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
//         result_viewer->addCoordinateSystem(0.3, "reference", viewports[0]);
//         result_viewer->addCoordinateSystem(0.3, "reference", viewports[1]);
        viewport = v2;
    }else{
        viewport = 0;
    }
    
    /*****************************
    * FRONT VIEW
    *****************************/
    result_viewer->setCameraPosition(-0.0100231, 0.155521, -1.10094, -0.0136177, 0.1338, -0.953934, -0.00975635, -0.989179, -0.146392);
    result_viewer->setCameraClipDistances(0, 9.34089);  
    
    /*****************************
    * SIDE VIEW
    *****************************/
//     result_viewer->setCameraPosition(-1.28927, 0.720515, 0.10089, -0.320861, 0.151598, 1.04012, 0.0827512, -0.812292, -0.57735);
//     result_viewer->setCameraClipDistances(0.00993555, 9.93555);
//     result_viewer->setSize(800, 450);
//     result_viewer->setPosition(500,300);

    
    /*****************************
    * TOP DOWN VIEW
    *****************************/
//     result_viewer->setCameraPosition(-0.223067, -1.87858, -0.597875, 0.0212919, -0.00507499, 1.17919, 0.246595, -0.683671, 0.686866);
//     result_viewer->setCameraClipDistances(0.931528, 5.0073);
//     result_viewer->setSize(800, 450);
//     result_viewer->setPosition(500,300);    
    
//     result_viewer->spin();
//     result_viewer->setCameraPosition(Position, Focal point, View up);
//     result_viewer->setCameraClipDistances(Clipping plane);

    /***************************************
    *  parse arguments
    ***************************************/
    if(argc<5)
    {
        pcl::console::print_info("Usage: cylinder_block_tracking DATA_PATH/PCD_FILE_FORMAT START_INDEX END_INDEX DEMO_NAME (opt)STEP_SIZE(1)\n");
        exit(1);
    }
    
    int view_id=0;
    int step=1;
    std::string basename_cloud=argv[1];
    unsigned int index_start = std::atoi(argv[2]);
    unsigned int index_end = std::atoi(argv[3]);
    std::string demo_name=argv[4];
    if(argc>5) step=std::atoi(argv[5]);
    std::string filename_pcd;
	
	// remove old recorded data
	char folder_demo_name[100];
	sprintf(folder_demo_name, "rm -r /home/zengzhen/Desktop/human_teaching/%s", demo_name.c_str());
	system(folder_demo_name);
	sprintf(folder_demo_name, "/home/zengzhen/Desktop/human_teaching/%s", demo_name.c_str());
	mkdir(folder_demo_name, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    /***************************************
    *  set up cloud, segmentation
    ***************************************/
    // set threshold of size of clustered cloud
    initialSeg.setThreshold(100);
    // downsampler
    grid.setLeafSize (leaf_size, leaf_size, leaf_size);
    
    /***************************************
    *  localize the tabletop and objects
    ***************************************/
    unsigned int idx = index_start;
    while( idx <= index_end && !result_viewer->wasStopped())
    { 
        std::cout << std::endl;
        std::cout << "frame id=" << idx << std::endl;
        filename_pcd = cv::format(basename_cloud.c_str(), idx);
        
        if(beginning)
        {            
            /***************************************
            *  Intialization:
            * -plane localization
            * -object cluster extraction
            * -bottle cluster localization
            ***************************************/
            initialSeg.resetCloud(filename_pcd, false);           
            initialSeg.seg(false);
            initialSeg.getCloudHull(cloud_hull);
            
            initialSeg.getObjects(cloud_objects, clusters);
            initialSeg.getPlaneCoefficients(*coefficients);
            initialSeg.getsceneCloud(pcdSceneCloud);
            sceneCloud=pcdSceneCloud.getCloud();
            
//             // shift sceneCloud closer to camera for better visualization
//             Eigen::Affine3f shift(Eigen::Affine3f::Identity());
//             shift.translate(Eigen::Vector3f(0, 0, -1.6));
//             pcl::transformPointCloud<RefPointType>(*sceneCloud, *sceneCloud, shift);

            
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
            result_viewer->addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon", viewport);
            result_viewer->addPointCloud<RefPointType>(sceneCloud, rgb, "new frame", viewport);
            result_viewer->spinOnce(100);
            
            /***************************************
             *  fingertip, hand_arm removal
             ***************************************/
            //opencv color filtering for fingertip_1
            {
                pcl::ScopeTime t_finger1("Finger 1(blue) detection");
                finger1Detector.setInputCloud(cloud_objects, clusters);
                finger1Detector.filter(f1_indices,cloud_finger1);
                
                if(f1_indices.indices.empty())
                {
                    std::cerr << "no finger(blue) detected\n";
                    exit(0);
                }
            }
            
            //fitting cylinder model to finger 1
            bottleDetector.setInputCloud(cloud_finger1);
            bottleDetector.fit();
            Eigen::Affine3f camToFinger1Coordinate;
            bottleDetector.getTransformation(camToFinger1Coordinate);
            camToFingerCoordinate.push_back(camToFinger1Coordinate);
            
            double finger1_r = bottleDetector.getRadius();
            double finger1_length = bottleDetector.getHeight();
			finger_radius.push_back(finger1_r);
			finger_height.push_back(finger1_length);
            
            RefPointType finger1_center = bottleDetector.getCenter();
            Eigen::Vector3f finger1_ori = bottleDetector.getOrientation();
            
//             bottleDetector.drawCenter(result_viewer);
//             bottleDetector.drawOrientation(result_viewer, "finger1_arrow");
            
            
//             result_viewer->addCoordinateSystem(0.3, camToFinger1Coordinate.inverse(), "finger1_reference", 0);
            fingerWireframe.push_back(TableObject::genCylinderWireframe(finger1_r, finger1_length));
            
            TableObject::view3D::drawCylinderWireframe(camToFinger1Coordinate, fingerWireframe[0], 
                                                               result_viewer, "blue_finger", 1, viewport);
            
            //opencv color filtering for fingertip_2
            {
                pcl::ScopeTime t_finger2("Finger 2(orange) detection");
                finger2Detector.setInputCloud(cloud_objects, clusters);
                finger2Detector.filter(f2_indices,cloud_finger2);
                
                if(f2_indices.indices.empty())
                {
                    std::cerr << "no finger(orange) detected\n";
                    exit(0);
                }
            }
            
            //fitting cylilnder model to finger 2
            bottleDetector.setInputCloud(cloud_finger2);
            bottleDetector.fit();
            Eigen::Affine3f camToFinger2Coordinate;
            bottleDetector.getTransformation(camToFinger2Coordinate);
            camToFingerCoordinate.push_back(camToFinger2Coordinate);
            
            double finger2_r = bottleDetector.getRadius();
            double finger2_length = bottleDetector.getHeight();
			finger_radius.push_back(finger2_r);
			finger_height.push_back(finger2_length);
            
            RefPointType finger2_center = bottleDetector.getCenter();
            Eigen::Vector3f finger2_ori = bottleDetector.getOrientation();
            
//             bottleDetector.drawCenter(result_viewer);
//             bottleDetector.drawOrientation(result_viewer, "finger2_arrow");
            
//             result_viewer->addCoordinateSystem(0.3, camToFinger1Coordinate.inverse(), "finger1_reference", 0);
            fingerWireframe.push_back(TableObject::genCylinderWireframe(finger2_r, finger2_length));
            
            TableObject::view3D::drawCylinderWireframe(camToFinger2Coordinate, fingerWireframe[1], 
                                                               result_viewer, "orange_finger", 2, viewport);
                        
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
            
            TableObject::view3D::drawClusters(result_viewer, cloud_objects, clusters, false, viewport);
            
            /***********************************
             * get finger coordinate frame
             ***********************************/
            Eigen::Vector3f center1(finger1_center.x, finger1_center.y, finger1_center.z);
            Eigen::Vector3f center2(finger2_center.x, finger2_center.y, finger2_center.z);
			gripper_ori = (finger1_ori + finger2_ori)/2.0;
			gripper_center = (center1+center2)/2.0;
			gripper_center = gripper_center-gripper_ori*0.135f; // to match endpoint position of Baxter
			
			std::cout << "center1: " << center1.transpose() << "\ncenter2: " << center2.transpose() << "\n";
			std::cout << "finger1_ori: " << finger1_ori.transpose() << "\nfinger2_ori: " << finger2_ori.transpose() << "\ngripper_vector: " << gripper_ori.transpose() << "\n";
			
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(center2-center1, gripper_ori, gripper_center, toGripperCoordinate);
			result_viewer->addCoordinateSystem(0.3, toGripperCoordinate.inverse(), "gripper_reference", viewport);
            
//             std::cout << "please input the cluster index of cylinder\n";
//             std::cin >> cylinder_id;
//             std::cout << "cylinder_id set to " << cylinder_id << std::endl;
//             
//             std::cout << "please input the cluster index of block\n";
//             std::cin >> block_id;
//             std::cout << "block_id set to " << block_id << std::endl;

            std::vector<Eigen::Vector3f> planeCoorInCamCoor = initialSeg.getPlaneCoorInCamCoor();
            pcl::getTransformationFromTwoUnitVectorsAndOrigin(planeCoorInCamCoor[1], planeCoorInCamCoor[2], planeCoorInCamCoor[3], toPlaneCoordinate);
            
            result_viewer->addCoordinateSystem(0.3, toPlaneCoordinate.inverse(), "plane_reference", viewport);
            
            //check each cluster should be modeled as cylinder or block
            for(int i=0; i<clusters.size(); i++)
            {
                CloudPtr clusterInCamCoor (new Cloud);
                pcl::copyPointCloud (*cloud_objects, clusters[i], *clusterInCamCoor);
                
                CloudPtr clusterInPlaneCoor (new Cloud);
                pcl::transformPointCloud(*clusterInCamCoor, *clusterInPlaneCoor, toPlaneCoordinate);
                
                std::cout << "size of clusterInPlaneCoor: " << clusterInPlaneCoor->points.size() << std::endl;
                
                pcl::NormalEstimationOMP<RefPointType, pcl::Normal> nest;
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                pcl::search::KdTree<RefPointType>::Ptr tree (new pcl::search::KdTree<RefPointType> ());
                // Estimate point normals
                nest.setSearchMethod (tree);
                nest.setInputCloud (clusterInPlaneCoor);
                nest.setRadiusSearch (0.02);
//                 nest.setKSearch(20);
                nest.compute (*normals);
                
                //cluster normals
                // Creating the KdTree object for the search method of the extraction
//                 pcl::RegionGrowing<RefPointType, pcl::Normal> reg;
//                 reg.setMinClusterSize (20);
//                 reg.setSearchMethod (tree); 
//                 reg.setNumberOfNeighbours (1000);
//                 reg.setInputCloud (clusterInPlaneCoor);
//                 reg.setInputNormals (normals);
//                 reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
//                 reg.setCurvatureThreshold (1);
                
//                 pcl::RegionGrowing<pcl::Normal> reg;
//                 reg.setMinClusterSize (30);
//                 reg.setSearchMethod (tree); 
//                 reg.setNumberOfNeighbours (100);
//                 reg.setInputCloud (normals);
//                 reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
//                 reg.setCurvatureThreshold (1);
                
                //test pcl vectorize whether works for pcl::Normal
//                 float* data = 0;
//                 pcl::Normal testNormal;
//                 testNormal.normal_x = 0.1;
//                 testNormal.normal_y = 0.8;
//                 testNormal.normal_z = 0.02;
//                 pcl::PointXYZ testNormal;
//                 testNormal.x = 0.1;
//                 testNormal.y = 0.8;
//                 testNormal.z = 0.02;
//                 pcl::PointRepresentation point_representation;
//                 point_representation.vectorize(testNormal, data);
//                 printf("data: %f %f %f\n", data[0], data[1], data[2]);
//                 exit(0);
                
                pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normal_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
                pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
                ec.setClusterTolerance (0.05);
                ec.setMinClusterSize (100);
//                 ec.setMaxClusterSize (25000);
                ec.setSearchMethod (normal_tree);
                
                // transform from pcl::Normal to pcl::PointXYZRGB
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_point_form(new pcl::PointCloud<pcl::PointXYZRGB>);
                normal_point_form->width = normals->points.size();
                normal_point_form->height = 1;
                normal_point_form->points.resize (normal_point_form->width * normal_point_form->height);
                for(int normal_index=0; normal_index<normal_point_form->points.size(); normal_index++)
                {
                    normal_point_form->points[normal_index].x = normals->points[normal_index].normal_x;
                    normal_point_form->points[normal_index].y = normals->points[normal_index].normal_y;
                    normal_point_form->points[normal_index].z = normals->points[normal_index].normal_z;
//                     normal_point_form->points[i].x = 64.0f * rand () / (RAND_MAX + 1.0f);
//                     normal_point_form->points[i].y = 64.0f * rand () / (RAND_MAX + 1.0f);
//                     normal_point_form->points[i].z = 64.0f * rand () / (RAND_MAX + 1.0f);
                    normal_point_form->points[normal_index].r = 255;
                    
//                     printf("%f %f %f\n", normal_point_form->points[normal_index].x, normal_point_form->points[normal_index].y, normal_point_form->points[normal_index].z);
                }
                
                ec.setInputCloud (normal_point_form);
                
                std::vector <pcl::PointIndices> clusters_of_normals;
                ec.extract (clusters_of_normals);

                std::cout << "Number of surface normal clusters in object " << i << " is equal to " << clusters_of_normals.size () << std::endl;

//                 result_viewer->addPointCloud(normal_point_form, "previous frame", viewport);
//                 result_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "previous frame", viewport);
//                 result_viewer->spin();
                
                std::stringstream ss;
                ss << "cloud_object_" << i << ".pcd";
                pcl::PCDWriter writer;
                writer.write<pcl::PointXYZRGBA> (ss.str (), *clusterInPlaneCoor, false);
                
                std::stringstream ss_normal;
                ss_normal << "cloud_object_normal_" << i << ".pcd";
                writer.write<pcl::Normal> (ss_normal.str (), *normals, false);
                
                // fit either cuboid or cylinder model to cluster_i
                if(clusters_of_normals.size()<3){
                    CloudPtr blockInCamCoor (new Cloud);
                    pcl::copyPointCloud (*cloud_objects, clusters[i], *blockInCamCoor);
                    
                    /***************************************
                    *  transform block cluster points from cam coordinates to plane coordinates
                    ***************************************/
                    CloudPtr blockInPlaneCoor (new Cloud);
                    pcl::transformPointCloud(*blockInCamCoor, *blockInPlaneCoor, toPlaneCoordinate);
                    
                    /***************************************
                    *  Project block on to plane
                    ***************************************/
                    // plane axis for projection: x, y, z
                    Eigen::Vector3f u(1,0,0);
                    Eigen::Vector3f v(0,1,0);
                    Eigen::Vector3f w(0,0,1);

                    // project the 3D points of the block onto the 2D tabletop plane 
                    std::vector<cv::Point2f> points; 
                    std::vector<float> block_points_height;
                    
                    // choose a point on the plane 
                    Eigen::Vector3f p0(blockInPlaneCoor->points[0].x, 
                                        blockInPlaneCoor->points[0].y, 
                                        blockInPlaneCoor->points[0].z); 
        //             std::cout << "z coordinates of block points in plane coordinate system: " << std::endl;
                    for(unsigned int ii=0; ii<blockInPlaneCoor->points.size(); ii++) 
                    { 
                        Eigen::Vector3f p3d(blockInPlaneCoor->points[ii].x, 
                                            blockInPlaneCoor->points[ii].y, 
                                            blockInPlaneCoor->points[ii].z); 

                        // subtract all 3D points with a point in the plane 
                        // this will move the origin of the 3D coordinate system 
                        // onto the plane 
                        
                        Eigen::Vector3f p3dh(blockInPlaneCoor->points[ii].x, 
                                            blockInPlaneCoor->points[ii].y, 
                                            blockInPlaneCoor->points[ii].z); 
        //                 block_points_height = block_points_height + std::abs(p3dh.dot(w)+coefficients->values[3])
        //                                 /std::sqrt(w.x()*w.x()
        //                                           +w.y()*w.y()
        //                                           +w.z()*w.z());
                        block_points_height.push_back(p3dh.z());
                        
                        
                        p3d = p3d - p0; 

                        cv::Point2f p2d; 
                        p2d.x = p3d.dot(u); 
                        p2d.y = p3d.dot(v); 
                        points.push_back(p2d); 
                        
        //                 std::cout << p3d.x() << " " << p3d.y() << " ";
                    } 
//                     std::cout << std::endl;

                    std::sort(block_points_height.begin(), block_points_height.end(), sort_using_greater_than);
                    std::cout << "block highest points: ";
                    for(int ii=0; ii<3; ii++) std::cout << block_points_height[ii] << " ";
                    std::cout << std::endl;
                    
                    cv::Mat points_mat(points); 
                    cv::RotatedRect rrect = cv::minAreaRect(points_mat); 
                    cv::Point2f rrPts[4]; 
                    rrect.points(rrPts); 
                    
                    for(int ii=0; ii<4; ii++)
                    {
                        std::cout << rrPts[ii].x << " " << rrPts[ii].y << std::endl;
                    }
                    
                    block_w.push_back(cv::norm(rrPts[0]-rrPts[1]));
                    block_d.push_back(cv::norm(rrPts[1]-rrPts[2]));
        //             double block_h = block_points_height/blockInPlaneCoor->points.size();
                    block_h.push_back(std::accumulate(block_points_height.begin(), block_points_height.begin()+3, 0.0)/3-0.004);
                    std::cout << "block width, depth, height is: " << block_w[block_index.size()] << " " << block_d[block_index.size()] << " " << block_h[block_index.size()] << std::endl;
                    
                    //assign the y-axis of the block coordinate to be the long side, and x-axis being the short side
                    Eigen::Vector3f block_y_axis, block_x_axis;
                    Eigen::Vector3f block_z_axis(0,0,1);
                    if(block_w[block_index.size()]>=block_d[block_index.size()])
                    {
                        block_y_axis.x() = (rrPts[1].x - rrPts[0].x)/block_w[block_index.size()];
                        block_y_axis.y() = (rrPts[1].y - rrPts[0].y)/block_w[block_index.size()];
                        block_y_axis.z() = 0;
                    
                        block_x_axis.x() = (rrPts[1].x - rrPts[2].x)/block_d[block_index.size()];
                        block_x_axis.y() = (rrPts[1].y - rrPts[2].y)/block_d[block_index.size()];
                        block_x_axis.z() = 0;
                    }else{
                        block_y_axis.x() = (rrPts[2].x - rrPts[1].x)/block_d[block_index.size()];
                        block_y_axis.y() = (rrPts[2].y - rrPts[1].y)/block_d[block_index.size()];
                        block_y_axis.z() = 0;
                    
                        block_x_axis.x() = (rrPts[1].x - rrPts[0].x)/block_w[block_index.size()];
                        block_x_axis.y() = (rrPts[1].y - rrPts[0].y)/block_w[block_index.size()];
                        block_x_axis.z() = 0;
                        
                        double temp=block_w[block_index.size()];
                        block_w[block_index.size()] = block_d[block_index.size()];
                        block_d[block_index.size()] = temp;
                    }
                    Eigen::Vector3f block_center;
                    block_center.x() = (rrPts[0].x+rrPts[1].x+rrPts[2].x+rrPts[3].x)/4 + p0.x()-0.005;
                    block_center.y() = (rrPts[0].y+rrPts[1].y+rrPts[2].y+rrPts[3].y)/4 + p0.y();
                    block_center.z() = block_h[block_index.size()]/2;
                    
                    Eigen::Affine3f planeToBlockCoordinateTemp;
        //             block_y_axis.x() = -1; block_y_axis.y() = 0; block_y_axis.z() = 0;
        //             block_center.x() = 0.1; block_center.y() = 0.2; block_center.z() = 0.2;
					pcl::getTransformationFromTwoUnitVectorsAndOrigin(block_y_axis, block_z_axis, block_center, planeToBlockCoordinateTemp);
                    
					camToBlockCoordinate.push_back(planeToBlockCoordinateTemp*toPlaneCoordinate);
					planeToBlockCoordinate.push_back(planeToBlockCoordinateTemp);
                    
                    // adjust the x,y axis of the block coordinate frame based on the camera position
                    pcl::PointXYZ camera_origin(0,0,0);
                    camera_origin=pcl::transformPoint<pcl::PointXYZ>(camera_origin, camToBlockCoordinate[block_index.size()]);
                    Eigen::Vector4f v0(1,0,0,0);
                    Eigen::Vector4f v1(-block_center.x(), -block_center.y(), -block_center.z(), 0);
                    double test_angle = pcl::getAngle3D(v0, v1) > M_PI/2;
                    bool revert = false;
                    if(camera_origin.x<0 & test_angle>M_PI/2)
                        revert = true;
                    if(!revert & camera_origin.y<0 & test_angle<M_PI/2)
                        revert = true;
                    if(revert)
                    {
                        block_x_axis = -block_x_axis;
                        block_y_axis = -block_y_axis;
						pcl::getTransformationFromTwoUnitVectorsAndOrigin(block_y_axis, block_z_axis, block_center, planeToBlockCoordinateTemp);
						camToBlockCoordinate[block_index.size()] = planeToBlockCoordinateTemp*toPlaneCoordinate;
						planeToBlockCoordinate[block_index.size()] = planeToBlockCoordinateTemp;
                    }
                    
                    TableObject::view3D::drawCoordinateSystem(camToBlockCoordinate[block_index.size()], result_viewer,
                                                              "block", block_index.size(), viewport);
                    TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[block_index.size()], block_w[block_index.size()], block_d[block_index.size()],
                                                             block_h[block_index.size()], result_viewer, block_index.size(), viewport);
					
					std::stringstream block_reference_id; 
					block_reference_id << "block_reference" << block_index.size();
					result_viewer->addCoordinateSystem(0.3, camToBlockCoordinate[block_index.size()].inverse(), block_reference_id.str(), viewport);
                    
                    block_index.push_back(i);
                }else{
                    CloudPtr cylinderInCamCoor (new Cloud);
                    pcl::copyPointCloud (*cloud_objects, clusters[i], *cylinderInCamCoor);
                    
                    CloudPtr cylinderInPlaneCoor (new Cloud);
                    pcl::transformPointCloud(*cylinderInCamCoor, *cylinderInPlaneCoor, toPlaneCoordinate);
                    
                    std::vector<float> cylinder_points_height;
                    // locating cylinder: center, radius, height
                    cv::Point2f cylinder2Dcenter;
                    float cylinder_r, cylinder_h;
                    
                    float** ap = new float*[cylinderInPlaneCoor->points.size()];
                    for(unsigned int ii=0; ii<cylinderInPlaneCoor->points.size(); ii++)
                    {
                        float* p = new float[2];
                        p[0] = cylinderInPlaneCoor->points[ii].x;
                        p[1] = cylinderInPlaneCoor->points[ii].y;
                        ap[ii]=p;
                        
                        cylinder_points_height.push_back(cylinderInPlaneCoor->points[ii].z);
                        
        //                 std::cout << p[0] << " " << p[1] << " ";
                    }
                    MB mb (2, ap, ap+cylinderInPlaneCoor->points.size());
                    
                    const float* miniBallcenter = mb.center();
                    cylinder2Dcenter.x = miniBallcenter[0];
                    cylinder2Dcenter.y = miniBallcenter[1];
                    cylinder_r = std::sqrt(mb.squared_radius());
                    cylinder_h = std::accumulate(cylinder_points_height.begin(), cylinder_points_height.begin()+3, 0.0)/3;

                    Eigen::Affine3f planeToCylinderCoordinate(Eigen::Affine3f::Identity());
                    planeToCylinderCoordinate.translate(Eigen::Vector3f(-cylinder2Dcenter.x, -cylinder2Dcenter.y, -cylinder_h/2));
                    camToCylinderCoordinate.push_back(planeToCylinderCoordinate*toPlaneCoordinate);
                    cylinderWireframe.push_back(TableObject::genCylinderWireframe(cylinder_r, cylinder_h));
                    
                    TableObject::view3D::drawCoordinateSystem(camToCylinderCoordinate[cylinder_index.size()], result_viewer,
                                                              "cylinder", cylinder_index.size(), viewport);
                    TableObject::view3D::drawCylinderWireframe(camToCylinderCoordinate[cylinder_index.size()], cylinderWireframe[cylinder_index.size()], 
                                                               result_viewer, "cylinder", cylinder_index.size(), viewport);
                    
                    cylinder_index.push_back(i);
                }
                    
            }
            result_viewer->addText( boost::str(boost::format("Frame: %d") % idx), 10, 10, 20, 1.0, 1.0, 1.0, "v2 text", viewport);
            
            /***************************************
             *  confirm localization
             ***************************************/
            result_viewer->spinOnce();
            char ready;
            std::cout << "are the objects and hand correctly localized? [y/n]\n";
            std::cin >> ready;
            if(ready=='y')
            {
                beginning=false;
                std::cout << "ready to initialize bottle tracker\n";
                /***************************************
                *  Tracking initialization
                ***************************************/
                {
                    pcl::ScopeTime t_track("Tracker initialization");
                    TableObject::track3D tempTracker1(false);
                    tempTracker1.setTarget(cloud_finger1, finger1_center);
                    tempTracker1.initialize(true);
                    finger_tracker.push_back(tempTracker1);
                    
                    TableObject::track3D tempTracker2(false);
                    tempTracker2.setTarget(cloud_finger2, finger2_center);
                    tempTracker2.initialize(true);
                    finger_tracker.push_back(tempTracker2);
                    
                    for(int i=0; i<cylinder_index.size(); i++)
                    {
                        CloudPtr clusterForTracker (new Cloud);
                        pcl::copyPointCloud (*cloud_objects, clusters[cylinder_index[i]], *clusterForTracker);
                        
                        Eigen::Vector4f cylinder_center;
                        pcl::compute3DCentroid<RefPointType>(*cloud_objects, clusters[cylinder_index[i]], cylinder_center);
                        RefPointType cylinder_center_point;
                        cylinder_center_point.x = cylinder_center[0];
                        cylinder_center_point.y = cylinder_center[1];
                        cylinder_center_point.z = cylinder_center[2];
                        
                        TableObject::track3D tempTracker(false);
                        tempTracker.setTarget(clusterForTracker, camToCylinderCoordinate[i]);
//                         tempTracker.setTarget(clusterForTracker, cylinder_center_point);
                        tempTracker.initialize();
                        
                        cylinder_tracker.push_back(tempTracker);
                    }
                    
                    for(int i=0; i<block_index.size(); i++)
                    {
                        CloudPtr clusterForTracker (new Cloud);
                        pcl::copyPointCloud (*cloud_objects, clusters[block_index[i]], *clusterForTracker);
                        
                        Eigen::Vector4f block_center;
                        pcl::compute3DCentroid<RefPointType>(*cloud_objects, clusters[block_index[i]], block_center);
                        RefPointType block_center_point;
                        block_center_point.x = block_center[0];
                        block_center_point.y = block_center[1];
                        block_center_point.z = block_center[2];
                        
                        TableObject::track3D tempTracker(false);
                        tempTracker.setTarget(clusterForTracker, camToBlockCoordinate[i]);
//                         tempTracker.setTarget(clusterForTracker, block_center_point);
                        tempTracker.initialize();
                        
                        block_tracker.push_back(tempTracker);
                        
                        TableObject::trackModel tempNormalTracker(false);
                        RefPointType model_color = TableObject::computeObjColor(cloud_objects, clusters[block_index[i]]);
                        CloudPtr completeBlockModel = TableObject::genTestCube(block_w[i], block_d[i], block_h[i], model_color);
                        CloudPtr completeBlockCam (new Cloud);
                        pcl::transformPointCloud<RefPointType>(*completeBlockModel, *completeBlockCam, camToBlockCoordinate[i].inverse());
                        
                        if(i==focus_index)
                            tempNormalTracker.setWrite(true);
                        tempNormalTracker.setModelColor(model_color);
                        tempNormalTracker.setModelParameters(block_w[i], block_d[i], block_h[i]);
                        tempNormalTracker.setTarget(clusterForTracker, camToBlockCoordinate[i]);
//                         tempNormalTracker.setTarget(completeBlockCam, camToBlockCoordinate[i]);
                        tempNormalTracker.initialize();
                        
                        model_tracker.push_back(tempNormalTracker);
                        
                        std::cout << "block point size = " << clusterForTracker->points.size() << "\n";
                        
                        // record block model parameters and coordinate transformation at 1st frame
                        std::ofstream localization_result;
                        std::stringstream block_datafile; 
                        block_datafile << "localization" << i << ".txt"; 
                        localization_result.open(block_datafile.str().c_str());
                        
                        char line[200];
                        sprintf(line, "%d\n", idx);
                        localization_result << line;
                        
                        sprintf(line, "%f %f %f\n", block_w[i], block_d[i], block_h[i]);
                        localization_result << line;
                        
                        float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
                        pcl::getTranslationAndEulerAngles (camToBlockCoordinate[i].inverse(),
                                          trans_x, trans_y, trans_z,
                                          trans_roll, trans_pitch, trans_yaw);
                        sprintf(line, "%.8f %.8f %.8f %.8f %.8f %.8f\n", trans_x, trans_y, trans_z,
                            trans_roll, trans_pitch, trans_yaw);
                        localization_result << line;
                        
                        localization_result.close();
                        
                        std::cout << camToBlockCoordinate[i].matrix() << std::endl;
                    }
                }
            }
            
            result_viewer->removeAllPointClouds();
            result_viewer->removeAllShapes();
            
            if(MULTIPLE_VIEWPORTS)
            {
                /*******************************
                * adjust viewing angle
                * ****************************/
                viewport = v1;
                
//                 result_viewer->setCameraPosition(-0.582986, 0.702785, 0.273226, 0.100209, -0.0708382, 1.31166, 0.0350891, -0.790231, -0.611803);
//                 result_viewer->setCameraClipDistances(0, 2.72077);
                
                result_viewer->setCameraPosition(-0.443487, 0.40926, 0.0384892, 0.153535, -0.0553094, 1.29202, 0.0766918, -0.92246, -0.378399);
                result_viewer->setCameraClipDistances(0.653173, 1.87804);
            }
            
             time_t t = time(0);   // get time now
            struct tm * now = localtime( & t );

            char buffer [100];
            strftime (buffer,100,"_%m%d%H%M%S",now);

            tracked_datafile << "result/" << demo_name << "/tracking_" << demo_name << "_" << index_start << "_" << index_end << buffer << ".txt";
        }else{
//             boost::mutex::scoped_lock updateLock(updateModelMutex);
// std::cout << "update new frame start\n";            

            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud_objects);
            viewport = 0;
            if(MULTIPLE_VIEWPORTS)
            {
                /*******************************
                * show previous frame result
                * ****************************/
                viewport = v1;
                
                result_viewer->addText( boost::str(boost::format("Frame: %d") % (idx-1)), 10, 10, 20, 1.0, 1.0, 1.0, "v1 text", viewport);
                result_viewer->addPointCloud<RefPointType>(cloud_objects, rgb, "previous frame", viewport); 
                result_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "previous frame", viewport);
                
                TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[focus_index], block_w[focus_index], block_d[focus_index], block_h[focus_index], result_viewer, 100, viewport);
//                 TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[1], block_w[1], block_d[1], block_h[1], result_viewer, 101, viewport);
//                 TableObject::view3D::drawCoordinateSystem(camToBlockCoordinate, result_viewer, "block", viewport, true);
//                 TableObject::view3D::drawCoordinateSystem(camToCylinderCoordinate, result_viewer, "cylinder", viewport, true);                
//                 TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate, block_w, block_d, block_h, result_viewer,viewport,true);
//                 TableObject::view3D::drawCylinderWireframe(camToCylinderCoordinate, cylinderWireframe, result_viewer, "cylinder", viewport, true);
                
                result_viewer->spinOnce();
            }
                
            /*******************************
             * show current frame result
             * ****************************/
            if(MULTIPLE_VIEWPORTS) viewport = v2; else viewport = 0;
            result_viewer->addText( boost::str(boost::format("Frame: %d") % idx), 10, 10, 20, 1.0, 1.0, 1.0, "v2 text", viewport);
            
            {
                pcl::ScopeTime t_finger1("segmentation based on previously detected plane");
                initialSeg.resetCloud(filename_pcd, false);
                initialSeg.seg(cloud_hull,false);
                initialSeg.getObjects(cloud_objects, clusters);
                initialSeg.getsceneCloud(pcdSceneCloud);
                sceneCloud=pcdSceneCloud.getCloud();
            }
            
            result_viewer->addPointCloud<RefPointType>(cloud_objects, rgb, "new frame", viewport);
            result_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "new frame", viewport);
            result_viewer->spinOnce();
            
            /***************************************
             *  fingertip
             ***************************************/
            //opencv color filtering for fingertip_1
            {
                pcl::ScopeTime t_finger1("Finger 1(blue) detection");
                finger1Detector.setInputCloud(cloud_objects, clusters);
                finger1Detector.filter(f1_indices,cloud_finger1);
                
                if(f1_indices.indices.empty())
                {
                    std::cerr << "no finger(blue) detected\n";
                    exit(0);
                }
            }
            
            Eigen::Affine3f camToFinger1Coordinate;
            //track finger 1
//             finger_tracker[0].track(cloud_finger1, transformation);
// 			finger_tracker[0].viewTrackedCloud(result_viewer, boost::str(boost::format("%s%d") % "tracked_finger" % 0));
// 			camToFinger1Coordinate = transformation.inverse();
            
            //fitting cylilnder model to finger 1
            bottleDetector.setInputCloud(cloud_finger1);
            bottleDetector.fit();
            bottleDetector.getTransformation(camToFinger1Coordinate);
            camToFingerCoordinate.push_back(camToFinger1Coordinate);
            
//             double finger1_r = bottleDetector.getRadius();
            double finger1_length = bottleDetector.getHeight();			
            
            RefPointType finger1_center = bottleDetector.getCenter();
			Eigen::Vector3f finger1_ori = bottleDetector.getOrientation();
			finger1_center.x = finger1_center.x - (finger_height[0]-finger1_length)*finger1_ori[0]/2.0;
			finger1_center.y = finger1_center.y - (finger_height[0]-finger1_length)*finger1_ori[1]/2.0;
			finger1_center.z = finger1_center.z - (finger_height[0]-finger1_length)*finger1_ori[2]/2.0;
			
			Eigen::Vector3f yc1( 0, finger1_ori[2], -finger1_ori[1] ); 
			yc1.normalize();
			Eigen::Vector3f center1_vector(finger1_center.x, finger1_center.y, finger1_center.z);
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(yc1, finger1_ori, center1_vector, camToFinger1Coordinate); 
            
// 			float finger1_x, finger1_y, finger1_z, finger1_roll, finger1_pitch, finger1_yaw;
// 			pcl::getTranslationAndEulerAngles(camToFinger1Coordinate, finger1_x, finger1_y, finger1_z, finger1_roll, finger1_pitch, finger1_yaw);
// 			Eigen::Vector3f finger1_ori(finger1_roll, finger1_pitch, finger1_yaw);
            
//             bottleDetector.drawCenter(result_viewer);
//             bottleDetector.drawOrientation(result_viewer, "finger1_arrow");
            
//             fingerWireframe[0] = TableObject::genCylinderWireframe(finger1_r, finger1_length);
            
            TableObject::view3D::drawCylinderWireframe(camToFinger1Coordinate, fingerWireframe[0], 
                                                               result_viewer, "blue_finger", 1, viewport);
// 			result_viewer->addCoordinateSystem(0.3, camToFinger1Coordinate.inverse(), "finger1_reference", viewport);
			pcl::visualization::PointCloudColorHandlerRGBField<RefPointType> rgb_finger1(cloud_finger1);
			result_viewer->addPointCloud<RefPointType>(cloud_finger1, rgb_finger1, "finger1", viewport);
            
            //opencv color filtering for fingertip_2
            {
                pcl::ScopeTime t_finger2("Finger 2(orange) detection");
                finger2Detector.setInputCloud(cloud_objects, clusters);
                finger2Detector.filter(f2_indices,cloud_finger2);
                
                if(f2_indices.indices.empty())
                {
                    std::cerr << "no finger(orange) detected\n";
                    exit(0);
                }
            }
            
            //fitting cylilnder model to finger 2
            bottleDetector.setInputCloud(cloud_finger2);
            bottleDetector.fit();
            Eigen::Affine3f camToFinger2Coordinate;
            bottleDetector.getTransformation(camToFinger2Coordinate);
            camToFingerCoordinate.push_back(camToFinger2Coordinate);
            
//             double finger2_r = bottleDetector.getRadius();
            double finger2_length = bottleDetector.getHeight();
            
            RefPointType finger2_center = bottleDetector.getCenter();
            Eigen::Vector3f finger2_ori = bottleDetector.getOrientation();
			finger2_center.x = finger2_center.x - (finger_height[1]-finger2_length)*finger2_ori[0]/2.0;
			finger2_center.y = finger2_center.y - (finger_height[1]-finger2_length)*finger2_ori[1]/2.0;
			finger2_center.z = finger2_center.z - (finger_height[1]-finger2_length)*finger2_ori[2]/2.0;
			
			Eigen::Vector3f yc2( 0, finger2_ori[2], -finger2_ori[1] ); 
			yc2.normalize();
			Eigen::Vector3f center2_vector(finger2_center.x, finger2_center.y, finger2_center.z);
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(yc2, finger2_ori, center2_vector, camToFinger2Coordinate); 
            
//             bottleDetector.drawCenter(result_viewer);
//             bottleDetector.drawOrientation(result_viewer, "finger2_arrow");
			
//             fingerWireframe[1] = TableObject::genCylinderWireframe(finger2_r, finger2_length);
            
            TableObject::view3D::drawCylinderWireframe(camToFinger2Coordinate, fingerWireframe[1], 
                                                               result_viewer, "orange_finger", 2, viewport);
// 			result_viewer->addCoordinateSystem(0.3, camToFinger2Coordinate.inverse(), "finger2_reference", viewport);
			pcl::visualization::PointCloudColorHandlerRGBField<RefPointType> rgb_finger2(cloud_finger2);
			result_viewer->addPointCloud<RefPointType>(cloud_finger2, rgb_finger2, "finger2", viewport);
            
            /***********************************
             * get finger coordinate frame
             ***********************************/
//             Eigen::Vector3f center1(finger1_center.x, finger1_center.y, finger1_center.z);
			Eigen::Vector3f center1(finger1_center.x+finger1_ori[0]*finger_height[0]/2.0,finger1_center.y+finger1_ori[1]*finger_height[0]/2.0,finger1_center.z+finger1_ori[2]*finger_height[0]/2.0);
			Eigen::Vector3f center2(finger2_center.x+finger2_ori[0]*finger_height[1]/2.0,finger2_center.y+finger2_ori[1]*finger_height[1]/2.0,finger2_center.z+finger2_ori[2]*finger_height[1]/2.0);
			gripper_ori = (finger1_ori + finger2_ori)/2.0;
			gripper_center = (center1+center2)/2.0;
			gripper_center = gripper_center-gripper_ori*0.135f; // to match endpoint position of Baxter
			
			std::cout << "center1: " << center1.transpose() << "\ncenter2: " << center2.transpose() << "\n";
			std::cout << "finger1_ori: " << finger1_ori.transpose() << "\nfinger2_ori: " << finger2_ori.transpose() << "\ngripper_vector: " << gripper_ori.transpose() << "\n";
            
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(center2-center1, gripper_ori, gripper_center, toGripperCoordinate);
			result_viewer->addCoordinateSystem(0.3, toGripperCoordinate.inverse(), "gripper_reference", viewport);
            
			result_viewer->addCoordinateSystem(0.3, toPlaneCoordinate.inverse(), "plane_reference", viewport);
            
            /***************************************
             *  Tracking objects
             ***************************************/
            {
                pcl::ScopeTime t_track("Tracking");
                grid.setInputCloud (cloud_objects);
                grid.filter (*track_target);
                
                pcl::copyPointCloud(*cloud_objects, *track_target);
                
//                 for(int i=0; i<finger_tracker.size(); i++)
//                 {
//                     finger_tracker[i].track(sceneCloud, transformation);
// //                     cylinder_tracker[i].viewTrackedCloud(result_viewer);
//                     
//                     //update cylinder coordinate system
//                     Eigen::Affine3f finger_reference = transformation;
//                     finger_reference.rotate(camToFingerCoordinate[i].inverse().rotation());
//                     
//                     std::stringstream finger_reference_id ("finger_reference"); 
//                     finger_reference_id << i;
//                     result_viewer->addCoordinateSystem(0.3, finger_reference, finger_reference_id.str());
//                     
//                     //update cylinder wireframe
//                     pcl::transformPointCloud<RefPointType>(*fingerWireframe[i], *fingerWireframeInCamCoor, finger_reference);
//                     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb_finger_wireframe(fingerWireframeInCamCoor);
//                     
//                     std::stringstream finger_wireframe_id ("finger_wireframe"); 
//                     finger_wireframe_id << i;
//                     result_viewer->addPointCloud<RefPointType>(fingerWireframeInCamCoor, rgb_finger_wireframe, finger_wireframe_id.str());
//                 }
                
                for(int i=0; i<cylinder_index.size(); i++)
                {
                    cylinder_tracker[i].track(track_target, transformation);
//                     cylinder_tracker[i].viewTrackedCloud(result_viewer, boost::str(boost::format("%s%d") % "tracked_cylinder" % i));
                    camToCylinderCoordinate[i] = transformation.inverse();
                    
                    //update cylinder coordinate system                    
//                     TableObject::view3D::drawCoordinateSystem(transformation.inverse(), result_viewer,
//                                                               "cylinder", i, viewport);
                    
                    //update cylinder wireframe
                    TableObject::view3D::drawCylinderWireframe(transformation.inverse(), cylinderWireframe[i], 
                                                               result_viewer, "cylinder", i, viewport);
                }
                
                for(int i=0; i<block_index.size(); i++)
                {
                    model_tracker[i].track(track_target, transformation);
//                     model_tracker[i].viewTrackedCloud(result_viewer, boost::str(boost::format("%s%d") % "tracked_block" % i));
                    
                     //block_tracker[i].track(track_target, transformation);
//                     block_tracker[i].viewTrackedCloud(result_viewer, boost::str(boost::format("%s%d") % "tracked_block" % i));
                    
                    camToBlockCoordinate[i] = transformation.inverse();
					planeToBlockCoordinate[i] = camToBlockCoordinate[i]*toPlaneCoordinate.inverse();
                    //update block coordinate system                    
//                     TableObject::view3D::drawCoordinateSystem(transformation.inverse(), result_viewer,
//                                                               "block", i, viewport);
                    
                    TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[i], block_w[i], block_d[i], block_h[i],
                                                             result_viewer, i, viewport);
					
					std::stringstream block_reference_id; 
					block_reference_id << "block_reference" << i;
					result_viewer->addCoordinateSystem(0.3, camToBlockCoordinate[i].inverse(), block_reference_id.str(), viewport);
                    
                    if(i==focus_index)
                    {
                        // record tracked result
                        std::ofstream tracked_result;
                        tracked_result.open(tracked_datafile.str().c_str(), ios::app);
                        
                        char line[200];
                        float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
                        pcl::getTranslationAndEulerAngles (transformation,
                                          trans_x, trans_y, trans_z,
                                          trans_roll, trans_pitch, trans_yaw);
                        sprintf(line, "%.8f %.8f %.8f %.8f %.8f %.8f\n", trans_x, trans_y, trans_z,
                            trans_roll, trans_pitch, trans_yaw);
                        tracked_result << line;
                        tracked_result.close();
                    }
                }
//                 model_tracker[focus_index].drawCroppingBox(result_viewer, viewport);
            }
            
//             result_viewer->spin();
            result_viewer->spinOnce(100);
//             boost::this_thread::sleep (boost::posix_time::seconds(1));
            
// std::cout << "update new frame ended\n";
//             updateLock.unlock();
            
            
            
//             if(idx==74)
//                 result_viewer->spin();
        }
        
        while(pause_for_observation)
        {
// std::cout << "pause for observatoin starts\n";
            result_viewer->spinOnce(100);
// std::cout << "pause for obsrevation ended\n";
//             boost::this_thread::sleep (boost::posix_time::microseconds(100));
        }
        
//         boost::mutex::scoped_lock updateLock(updateModelMutex);
// std::cout << "remove all stuff start\n";

        result_viewer->removeAllPointClouds();
        result_viewer->removeAllShapes();
        result_viewer->removeCoordinateSystem("gripper_reference");
		result_viewer->removeCoordinateSystem("finger1_reference");
		result_viewer->removeCoordinateSystem("finger2_reference");
        
        for(int i=0; i<cylinder_index.size(); i++)
        {
            std::stringstream cylinder_reference_id; 
            cylinder_reference_id << "cylinder_reference" << i;
            result_viewer->removeCoordinateSystem(cylinder_reference_id.str());
        }
        
        for(int i=0; i<block_index.size(); i++)
        {
            std::stringstream block_reference_id; 
            block_reference_id << "block_reference" << i;
            result_viewer->removeCoordinateSystem(block_reference_id.str());
        }
        
        result_viewer->removeCoordinateSystem("reference");
        result_viewer->removeCoordinateSystem("plane_reference");

// std::cout << "remove all stuff ended\n";
//         updateLock.unlock();
		
		//record gripper pose in table plane frame, and transformation from plane to block
		std::ofstream record_file1;
		char buffer[100];
		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/gripper_pose_in_plane.txt", demo_name.c_str());
		record_file1.open (buffer, ios::app);
		
		gripper_center = toPlaneCoordinate*gripper_center;
		gripper_ori = toPlaneCoordinate.rotation()*gripper_ori;
		
		tf::Quaternion gripper_pose_euler;
		gripper_pose_euler.setRPY(gripper_ori[0], gripper_ori[1], gripper_ori[2]);
		geometry_msgs::Quaternion gripper_pose_quat;
		tf::quaternionTFToMsg(gripper_pose_euler, gripper_pose_quat);
		
		record_file1 << gripper_center.transpose() << " " << gripper_ori.transpose() << std::endl;
		record_file1.close();
		
		// record plane to gripper transformation in <4x4 affine matrix> format
		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_gripper.txt", demo_name.c_str());
		record_file1.open (buffer, ios::app);
		
		Eigen::Affine3f planeToGripperCooridnate;
		planeToGripperCooridnate = toGripperCoordinate*(toPlaneCoordinate.inverse());
		record_file1 << planeToGripperCooridnate.matrix() << std::endl;
		record_file1.close();
		
		// record plane to gripper in <translation, quaternion> format
		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_gripper_dmp.txt", demo_name.c_str());
		record_file1.open (buffer, ios::app);
		
		Eigen::Vector3f rot2euler = planeToGripperCooridnate.rotation().eulerAngles(0,1,2);
		gripper_pose_euler.setRPY(rot2euler[0], rot2euler[1], rot2euler[2]); // mixing EIGEN and TF is bad, this is probably wrong
		tf::quaternionTFToMsg(gripper_pose_euler, gripper_pose_quat);
		
		record_file1 << planeToGripperCooridnate.translation().transpose() << " " << gripper_pose_quat.x
					 << " " << gripper_pose_quat.y << " " << gripper_pose_quat.z << " " << gripper_pose_quat.w << std::endl;
		record_file1.close();
		
		// record plane to block transformation in <4x4 affine matrix> format
		for(int i=0; i<planeToBlockCoordinate.size(); i++)
		{	
			std::ofstream record_file2;
			sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_block%d.txt", demo_name.c_str(), i);
			record_file2.open (buffer, ios::app);
			
			record_file2 << planeToBlockCoordinate[i].matrix() << "\n";
			record_file2.close();
		}
    
        idx=idx+step;
    }
    
    return 0;
}



