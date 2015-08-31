#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// Types
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<pcl::PointXYZRGB,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int
main (int argc, char **argv)
{
  // Point clouds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  PointCloudT::Ptr object (new PointCloudT);
//   PointCloudT::Ptr object_aligned (new PointCloudT);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_aligned (new pcl::PointCloud<pcl::PointXYZRGB>);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
  // Get input object and scene
  if (argc != 3)
  {
    pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
    return (1);
  }
  
  // Load object and scene
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *icp_input_cloud) < 0 ||
      pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *icp_target_cloud) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
    return (1);
  }
  
  std::cout << "scene cloud: width = " << icp_target_cloud->width << "; height = " << icp_target_cloud->height << "; totoal = " << icp_target_cloud->width*icp_target_cloud->height << std::endl;
  std::cout << "object cloud: width = " << icp_input_cloud->width << "; height = " << icp_input_cloud->height << "; totoal = " << icp_input_cloud->width*icp_input_cloud->height << std::endl;
  
  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  const float leaf = 0.003f; // smaller, slower
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (icp_input_cloud);
  grid.filter (*icp_input_cloud);
  grid.setInputCloud (icp_target_cloud);
  grid.filter (*icp_target_cloud);
  
  std::cout << "scene is organized: " << scene->isOrganized() << std::endl;
  
  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<pcl::PointXYZRGB,PointNT> nest;
//   pcl::NormalEstimationOMP<pcl::PointXYZRGB, PointNT> nest;
  nest.setRadiusSearch (0.01); // smaller, faster
  nest.setInputCloud (icp_target_cloud);
//   nest.setInputCloud(input_scene);
  nest.compute (*scene);
  nest.setInputCloud(icp_input_cloud);
  nest.compute(*object);
  
//     nest.setSearchMethod (tree);
//     nest.setInputCloud (cluster_large);
//     nest.setKSearch (50);
//     nest.compute (*cloud_normals);
  
  std::cout << "scene is organized: " << scene->isOrganized() << std::endl;
  
  
    pcl::visualization::PCLVisualizer visu("Alignment");
//     visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//     visu.addPointCloud (object, ColorHandlerT (object, 255, 0, 0), "object");
//     visu.addPointCloudNormals<pcl::PointNormal,pcl::PointNormal>(scene, scene);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(icp_target_cloud);
    visu.addPointCloud<pcl::PointXYZRGB>(icp_target_cloud, rgb, "bottle");
    visu.addPointCloudNormals<pcl::PointXYZRGB,pcl::PointNormal>(icp_input_cloud, object);
    while (!visu.wasStopped ())
    {
        visu.spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025); // smaller, faster
  fest.setInputCloud (icp_input_cloud);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (icp_target_cloud);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<pcl::PointXYZRGB,pcl::PointXYZRGB,FeatureT> align;
  align.setInputSource (icp_input_cloud);
  align.setSourceFeatures (object_features);
  align.setInputTarget (icp_target_cloud);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (15000); // Number of RANSAC iterations
  align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (2); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    // Show alignment
//     visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//     visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> aligned(object_aligned, 255, 0, 0);
    visu.addPointCloud<pcl::PointXYZRGB>(object_aligned, aligned, "aligned_bottle");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
    return (1);
  }
  
  return (0);
}
