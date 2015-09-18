#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/time.h>
#include <Eigen/Geometry>

int
 main (int argc, char** argv)
{
    // Get input object and scene
    if (argc < 2)
    {
        pcl::console::print_error ("Syntax is: %s cloud1.pcd (cloud2.pcd)\n", argv[0]);
        return (1);
    }
    
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBA>);

    // Load object and scene
    pcl::console::print_highlight ("Loading point clouds...\n");
    if(argc<3)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud_in) < 0)
            pcl::console::print_error ("Error loading first file!\n");
        *cloud_out = *cloud_in;
        
        //transform cloud
        Eigen::Affine3f transformation;
        transformation.setIdentity();
        transformation.translate(Eigen::Vector3f(0.3,0.02,-0.1));
        float roll, pitch, yaw;
        roll = 0.02; pitch = 1.2; yaw = 0;
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
        Eigen::Quaternion<float> q = rollAngle*pitchAngle*yawAngle;
        transformation.rotate(q);
        
        pcl::transformPointCloud<pcl::PointXYZRGBA>(*cloud_in, *cloud_out, transformation);
        std::cout << "Transformed " << cloud_in->points.size () << " data points:"
            << std::endl;
    }else{
       if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud_in) < 0 ||
        pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *cloud_out) < 0)
        {
            pcl::console::print_error ("Error loading files!\n");
            return (1);
        } 
    }
    
    // Fill in the CloudIn data
//     cloud_in->width    = 100;
//     cloud_in->height   = 1;
//     cloud_in->is_dense = false;
//     cloud_in->points.resize (cloud_in->width * cloud_in->height);
//     for (size_t i = 0; i < cloud_in->points.size (); ++i)
//     {
//         cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//         cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//         cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//     }

    std::cout << "size:" << cloud_out->points.size() << std::endl;
      
    {
        pcl::ScopeTime("icp proces");
        
        pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        pcl::PointCloud<pcl::PointXYZRGBA> Final;
        icp.setMaximumIterations(1000000);
        icp.setRANSACOutlierRejectionThreshold(0.01);
        icp.align(Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        
        //translation, rotation
        Eigen::Matrix4f icp_transformation=icp.getFinalTransformation();
        Eigen::Matrix3f icp_rotation = icp_transformation.block<3,3>(0,0);
        Eigen::Vector3f euler = icp_rotation.eulerAngles(0,1,2);
        std::cout << "rotation: " << euler.transpose() << std::endl;
        std::cout << "translation:" << icp_transformation.block<3,1>(0,3).transpose() << std::endl;
    }
  

 return (0);
}
