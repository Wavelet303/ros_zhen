/**
 * \file robot_block_tracking.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief localize tabletop and track blocks in the scene as robot manipulates it
 */

#include <sys/stat.h>

//ros
#include <ros/ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

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
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/icp.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>

// eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

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

// from table_obejct
#include "table_object/miniBall.hpp"

bool VERBOSE = false;
bool DEBUG = false;
bool DEBUG_COLOR = false;

typedef float* const* PointIterator; 
typedef const float* CoordIterator;
typedef Miniball::Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> > MB;

// publisher, subscriber
ros::Subscriber scene_subscriber;

boost::shared_ptr<pcl::visualization::PCLVisualizer> result_viewer (new pcl::visualization::PCLVisualizer ("table_block"));

// set up variables for tabletop localization;
TableObject::Segmentation initialSeg;
TableObject::pcdCloud pcdSceneCloud;
CloudPtr sceneCloud(new Cloud);
CloudPtr cloud_objects(new Cloud);
CloudPtr cloud_hull(new Cloud);
CloudPtr cluster_hull(new Cloud);
CloudPtr track_target(new Cloud);
CloudPtr tracked_cloud(new Cloud);

std::vector<pcl::PointIndices> clusters;

TableObject::bottleDetector bottleDetector;
std::vector<TableObject::track3D> cylinder_tracker;
std::vector<TableObject::track3D> block_tracker;
std::vector<TableObject::trackModel> model_tracker;
int focus_index = 0;

Eigen::Affine3f toPlaneCoordinate;
std::vector<Eigen::Affine3f> camToCylinderCoordinate;
std::vector<Eigen::Affine3f> camToBlockCoordinate;
std::vector<Eigen::Affine3f> planeToBlockCoordinate;
Eigen::Affine3f transformation;
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());

pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
float leaf_size=0.01;

int counter=0;
int initial_cluster_size;

bool beginning = true;

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

static bool sort_using_greater_than(float u, float v)
{
	return u > v;
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
			initialSeg.getCloudHull(cloud_hull);
			
			initialSeg.getObjects(cloud_objects, clusters);
			initialSeg.getPlaneCoefficients(*coefficients);
			initialSeg.getsceneCloud(pcdSceneCloud);
			sceneCloud=pcdSceneCloud.getCloud();
			
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(sceneCloud);
			result_viewer->addPolygon<RefPointType>(cloud_hull, 0, 255, 0, "polygon", 0);
			result_viewer->addPointCloud<RefPointType>(sceneCloud, rgb, "new frame", 0);
			result_viewer->spinOnce(100);
			
			std::vector<Eigen::Vector3f> planeCoorInCamCoor = initialSeg.getPlaneCoorInCamCoor();
			pcl::getTransformationFromTwoUnitVectorsAndOrigin(planeCoorInCamCoor[1], planeCoorInCamCoor[2], planeCoorInCamCoor[3], toPlaneCoordinate);
			result_viewer->addCoordinateSystem(0.3, toPlaneCoordinate.inverse(), "plane_reference", 0);
			
			/***************************************
			 *  Localizing blocks
			 ***************************************/
			//check each cluster should be modeled as cylinder or block
			block_index.clear();
			cylinder_index.clear();
			int block_count = 0;
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
				nest.compute (*normals);
				
				pcl::search::KdTree<pcl::PointXYZRGB>::Ptr normal_tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
				pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
				ec.setClusterTolerance (0.05);
				ec.setMinClusterSize (100);
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
					normal_point_form->points[normal_index].r = 255;
				}
				ec.setInputCloud (normal_point_form);
				
				std::vector <pcl::PointIndices> clusters_of_normals;
				ec.extract (clusters_of_normals);
				std::cout << "Number of surface normal clusters in object " << i << " is equal to " << clusters_of_normals.size () << std::endl;
				
				// fit either cuboid or cylinder model to cluster_i
				if(clusters_of_normals.size()<5){
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
					Eigen::Vector4f v1(-block_center.x(), -block_center.y(), 0, 0);
					double test_angle = pcl::getAngle3D(v0, v1);
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
															  "block", block_count);
					TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[block_index.size()], block_w[block_index.size()], block_d[block_index.size()],
															 block_h[block_index.size()], result_viewer, block_count);
					
					std::stringstream block_reference_id; 
					block_reference_id << "block_reference" << block_count;
// 					char buffer[100];
// 					sprintf(buffer, "block_reference%d", block_count);
// 					result_viewer->addCoordinateSystem(0.3, camToBlockCoordinate[(int)block_index.size()].inverse(), block_reference_id.str());
					
					block_index.push_back(i);
					block_count++;
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
					
					TableObject::view3D::drawCoordinateSystem(camToCylinderCoordinate[(int)cylinder_index.size()], result_viewer,
															  "cylinder", (int)cylinder_index.size());
					TableObject::view3D::drawCylinderWireframe(camToCylinderCoordinate[cylinder_index.size()], cylinderWireframe[(int)cylinder_index.size()], 
															   result_viewer, "cylinder", (int)cylinder_index.size());
					
					cylinder_index.push_back(i);
				}
				sleep(0.5); //prevent visualizer from interuptted by the user inquiry step
			}
		
			/***************************************
			 *  confirm localization
			 ***************************************/
			result_viewer->spinOnce(100);
			char ready;
			std::cout << "are the objects and hand correctly localized? [y/n]\n";
			std::cin >> ready;
			if(ready=='y')
			{
				beginning=false;
				std::cout << "ready to initialize object tracker\n";
				/***************************************
				 *  Tracking initialization
				 ***************************************/
				{
					pcl::ScopeTime t_track("Tracker initialization");
					
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
						char coordinate_correct = 'n';
						
						std::cout << "is the coordinate frame of block " << i << "correctly localized? [y/n]\n";
						std::cin >> coordinate_correct;
						while(coordinate_correct!='y')
						{
							// rotate the coordinate frame w.r.t the z-axis by 180 degrees
							Eigen::Affine3f rot_180(Eigen::Affine3f::Identity());
							pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(0,-1,0), Eigen::Vector3f(0,0,1), Eigen::Vector3f(0,0,0), rot_180);
							planeToBlockCoordinate[i] = rot_180*planeToBlockCoordinate[i];
							camToBlockCoordinate[i] = planeToBlockCoordinate[i]*toPlaneCoordinate;
							
							// remove previous coordinate frame
							std::stringstream block_reference_id; 
							block_reference_id << "block_reference" << i;
							result_viewer->removeCoordinateSystem(block_reference_id.str());
							std::stringstream block_wireframe_id;
							block_wireframe_id << "block_wireframe" << i;
							result_viewer->removeShape(block_wireframe_id.str(), 0);
							
							// re-visualize the new coordinate
							TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[i], block_w[i], block_d[i],
																	 block_h[i], result_viewer, i);
							result_viewer->addCoordinateSystem(0.3, camToBlockCoordinate[i].inverse(), block_reference_id.str());
							result_viewer->spinOnce();
							sleep(0.5);
							
							std::cout << "is the coordinate frame of block " << i << "correctly localized? [y/n]\n";
							std::cin >> coordinate_correct;
						}
						
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
						
// 						if(i==focus_index)
// 							tempNormalTracker.setWrite(true);
						tempNormalTracker.setModelColor(model_color);
						tempNormalTracker.setModelParameters(block_w[i], block_d[i], block_h[i]);
						tempNormalTracker.setTarget(clusterForTracker, camToBlockCoordinate[i]);
						//                         tempNormalTracker.setTarget(completeBlockCam, camToBlockCoordinate[i]);
						tempNormalTracker.initialize();
						
						model_tracker.push_back(tempNormalTracker);
						
						std::cout << "block point size = " << clusterForTracker->points.size() << "\n";
						
						// record block model parameters and coordinate transformation at 1st frame
// 						std::ofstream localization_result;
// 						std::stringstream block_datafile; 
// 						block_datafile << "localization" << i << ".txt"; 
// 						localization_result.open(block_datafile.str().c_str());
// 						
// 						char line[200];
// 						sprintf(line, "%d\n", 1);
// 						localization_result << line;
// 						
// 						sprintf(line, "%f %f %f\n", block_w[i], block_d[i], block_h[i]);
// 						localization_result << line;
// 						
// 						float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
// 						pcl::getTranslationAndEulerAngles (camToBlockCoordinate[i].inverse(),
// 														   trans_x, trans_y, trans_z,
// 										 trans_roll, trans_pitch, trans_yaw);
// 						sprintf(line, "%.8f %.8f %.8f %.8f %.8f %.8f\n", trans_x, trans_y, trans_z,
// 								trans_roll, trans_pitch, trans_yaw);
// 						localization_result << line;
// 						
// 						localization_result.close();
						
						std::cout << camToBlockCoordinate[i].matrix() << std::endl;
					}
				}
			}
			result_viewer->removeAllPointClouds();
			result_viewer->removeAllShapes();
		}else{
			/*******************************
			 * show current frame result
			 * ****************************/
			{
				pcl::ScopeTime t_finger1("segmentation based on previously detected plane");
				initialSeg.resetCloud(scene_cloud);
				initialSeg.seg(cloud_hull,false);
				initialSeg.getObjects(cloud_objects, clusters);
				initialSeg.getsceneCloud(pcdSceneCloud);
				sceneCloud=pcdSceneCloud.getCloud();
			}
			result_viewer->addCoordinateSystem(0.3, toPlaneCoordinate.inverse(), "plane_reference", 0);
			
			/***************************************
			 *  Tracking objects
			 ***************************************/
			{
				pcl::ScopeTime t_track("Tracking");
// 				grid.setInputCloud (cloud_objects);
// 				grid.filter (*track_target);
				
				pcl::copyPointCloud(*cloud_objects, *track_target);
				
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(scene_cloud);
				result_viewer->addPointCloud<RefPointType>(scene_cloud, rgb, "new frame", 0);
				result_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3.0, "new frame", 0);
				result_viewer->spinOnce();
				
				for(int i=0; i<cylinder_index.size(); i++)
				{
					cylinder_tracker[i].track(track_target, transformation);
					//                     cylinder_tracker[i].viewTrackedCloud(result_viewer, boost::str(boost::format("%s%d") % "tracked_cylinder" % i));
					camToCylinderCoordinate[i] = transformation.inverse();
					
					//update cylinder coordinate system                    
					//                     TableObject::view3D::drawCoordinateSystem(transformation.inverse(), result_viewer,
					//                                                               "cylinder", i);
					
					//update cylinder wireframe
					TableObject::view3D::drawCylinderWireframe(transformation.inverse(), cylinderWireframe[i], 
															   result_viewer, "cylinder", i);
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
					TableObject::view3D::drawCoordinateSystem(transformation.inverse(), result_viewer,"block", i);
					
					TableObject::view3D::drawCuboidWireframe(camToBlockCoordinate[i], block_w[i], block_d[i], block_h[i],
															 result_viewer, i);
					
// 					std::stringstream block_reference_id; 
// 					block_reference_id << "block_reference" << i;
// 					result_viewer->addCoordinateSystem(0.3, camToBlockCoordinate[i].inverse(), block_reference_id.str());
					
					// publish static transform from /camera_link to /block (elements come from blocktocamera transformation)
					static tf::TransformBroadcaster br;
					Eigen::Affine3f blockToKinectTransformation = camToBlockCoordinate[i].inverse();
					Eigen::Affine3f kinectToCam;
					Eigen::Affine3f kinectToCamTemp;
					pcl::getTransformationFromTwoUnitVectorsAndOrigin(Eigen::Vector3f(-1,0,0), Eigen::Vector3f(0,-1,0), Eigen::Vector3f(-0.04,0,0), kinectToCam);
					Eigen::Affine3f blockToCamTransformation = kinectToCam*blockToKinectTransformation;
					
					Eigen::Vector3f translation = blockToCamTransformation.translation();
					std::stringstream block_link_id; 
					block_link_id << "block_link" << i;
					
					tf::Transform transform(tf::Matrix3x3(blockToCamTransformation(0,0),blockToCamTransformation(0,1),blockToCamTransformation(0,2),
															  blockToCamTransformation(1,0),blockToCamTransformation(1,1),blockToCamTransformation(1,2),
															  blockToCamTransformation(2,0),blockToCamTransformation(2,1),blockToCamTransformation(2,2)),
												tf::Vector3(translation(0), translation(1), translation(2)));
					br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera_link", block_link_id.str()));
					std::cout << blockToCamTransformation.matrix() << std::endl;
				}
			}
			
			result_viewer->spinOnce(100);
		}
		
		result_viewer->removeAllPointClouds();
		result_viewer->removeAllShapes();
		
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
		
		result_viewer->removeCoordinateSystem("reference", 0);
		result_viewer->removeCoordinateSystem("plane_reference", 0);
		
		//record gripper pose in table plane frame, and transformation from plane to block
// 		std::ofstream record_file1;
// 		char buffer[100];
// 		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/gripper_pose_in_plane.txt", demo_name.c_str());
// 		record_file1.open (buffer, ios::app);
// 		
// 		gripper_center = toPlaneCoordinate*gripper_center;
// 		gripper_ori = toPlaneCoordinate.rotation()*gripper_ori;
// 		
// 		tf::Quaternion gripper_pose_euler;
// 		gripper_pose_euler.setRPY(gripper_ori[0], gripper_ori[1], gripper_ori[2]);
// 		geometry_msgs::Quaternion gripper_pose_quat;
// 		tf::quaternionTFToMsg(gripper_pose_euler, gripper_pose_quat);
// 		
// 		record_file1 << gripper_center.transpose() << " " << gripper_ori.transpose() << std::endl;
// 		record_file1.close();
// 		
// 		// record plane to gripper transformation in <4x4 affine matrix> format
// 		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_gripper.txt", demo_name.c_str());
// 		record_file1.open (buffer, ios::app);
// 		
// 		Eigen::Affine3f planeToGripperCooridnate;
// 		planeToGripperCooridnate = toGripperCoordinate*(toPlaneCoordinate.inverse());
// 		record_file1 << planeToGripperCooridnate.matrix() << std::endl;
// 		record_file1.close();
// 		
// 		// record plane to gripper in <translation, quaternion> format
// 		sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_gripper_dmp.txt", demo_name.c_str());
// 		record_file1.open (buffer, ios::app);
// 		
// 		Eigen::Vector3f rot2euler = planeToGripperCooridnate.rotation().eulerAngles(0,1,2);
// 		gripper_pose_euler.setRPY(rot2euler[0], rot2euler[1], rot2euler[2]);
// 		tf::quaternionTFToMsg(gripper_pose_euler, gripper_pose_quat);
// 		
// 		record_file1 << planeToGripperCooridnate.translation().transpose() << " " << gripper_pose_quat.x
// 		<< " " << gripper_pose_quat.y << " " << gripper_pose_quat.z << " " << gripper_pose_quat.w << std::endl;
// 		record_file1.close();
// 		
// 		// record plane to block transformation in <4x4 affine matrix> format
// 		for(int i=0; i<planeToBlockCoordinate.size(); i++)
// 		{	
// 			std::ofstream record_file2;
// 			sprintf(buffer, "/home/zengzhen/Desktop/human_teaching/%s/plane_to_block%d.txt", demo_name.c_str(), i);
// 			record_file2.open (buffer, ios::app);
// 			
// 			record_file2 << planeToBlockCoordinate[i].matrix() << "\n";
// 			record_file2.close();
// 		}
	}
}

int main(int argc, char **argv)
{
	if(argc<2)
	{
		printf("please input phrase: \n");
		printf("    test -- pure robot_block_tracking test\n");
		printf("    run  -- run the entire framework\n");
		exit(0);
	}
	/***************************************
	 *  ros node initialization
	 ***************************************/
	ros::init(argc, argv, "robot_block_tracking");
	ros::NodeHandle node_handle;
	ros::Rate loop_rate(20);   //loop rate for any loops appear in this cpp
// 	sleep(10.0); //wait for other nodes up and running in baxter_moveit/mvoe_group_server.launch
	
	/***************************************
	 *  setup visualizer
	 ***************************************/
// 	result_viewer->addCoordinateSystem(0.3, "reference", 0);
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
	ros::Publisher table_marker_pub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	
	/***************************************
	 *  visualize table (fixed location)
	 ***************************************/
	// Set our initial shape type to be a cube
	visualization_msgs::Marker marker;
	uint32_t shape = visualization_msgs::Marker::CUBE;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/base";
	marker.header.stamp = ros::Time::now();
	
	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "table";
	marker.id = 0;
	
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;
	
	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;
	
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 1;
	marker.pose.position.y = 0.3;
	marker.pose.position.z = -0.605;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.7;
	marker.scale.y = 2.0;
	marker.scale.z = 1.0;
	
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0000;
	marker.color.g = 0.8980f;
	marker.color.b = 0.4941f;
	marker.color.a = 1.0;
	
	marker.lifetime = ros::Duration();
	
	// Publish the marker
// 	while (table_marker_pub.getNumSubscribers() < 1)
// 	{
// 		if (!ros::ok())
// 		{
// 			return 0;
// 		}
// 		ROS_WARN_ONCE("Please create a subscriber to the marker");
// 		sleep(1);
// 	}
// 	ROS_INFO("Subscriber found");
// 	table_marker_pub.publish(marker);
	
	/***************************************
	 *  spin ros node
	 ***************************************/
	ros::spin();
	
	
	return 0;
}
