
// boost::shared_ptr<pcl::visualization::PCLVisualizer> testViewer (new pcl::visualization::PCLVisualizer ("test Viewer"));


/*
void testVisualizerThread()
{
//     testViewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&testViewer);
    
    // test visualizer thread
    CloudPtr cloud(new Cloud);
    if (pcl::io::loadPCDFile<RefPointType> ("test/cld00012.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file\n");
        exit(1);
    }else{
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
        testViewer->addPointCloud<RefPointType>(cloud, rgb, "test_cloud");
    
//         testViewer.spin();
        
        while (!testViewer->wasStopped ())
        {
            testViewer->spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
        
    }
    
    
}*/

int main(int argc, char** argv)
{
    //     XInitThreads();
    
    
//     boost::thread_group tgroup;
//     tgroup.create_thread(boost::bind(&testVisualizerThread));
//     
// //     testVisualizerThread;
// //     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// //     viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
// //     viewer->spinOnce(500);
//     
//     CloudPtr testCloud(new Cloud);
// //     if (pcl::io::loadPCDFile<RefPointType> ("test/cld00012.pcd", *testCloud) == -1) //* load the file
// //     {
// //         PCL_ERROR ("Couldn't read file\n");
// //         exit(1);
// //     }else{
// //         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> testRGB(testCloud);
// //         viewer->addPointCloud<RefPointType>(testCloud, testRGB, "test_cloud");
// //         viewer->spinOnce(500);
// //     }
//     
//     if (pcl::io::loadPCDFile<RefPointType> ("test/cld00032.pcd", *testCloud) == -1) //* load the file
//     {
//         PCL_ERROR ("Couldn't read file\n");
//         exit(1);
//     }else{
//         while(pause_for_observation)
//         {
//             boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//         }
//         pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> testRGB(testCloud);
// //         testViewer->updatePointCloud<RefPointType>(testCloud, testRGB, "test_cloud");
// //         testViewer->spinOnce(100);
//     }
//     
//     testViewer->spin();
}