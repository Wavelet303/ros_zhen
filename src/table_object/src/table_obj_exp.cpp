/**
 * \file table_obj_exp.cpp
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief construct the home pose and available action set
 */
 
#include "table_object/table_obj_exp.h" 

namespace TableObjectAction{
    Explorer::Explorer(std::vector<std::vector<int> > behavior)
    {
    	_behavior = behavior;
    	
    	// set the home pose
    	tf::Quaternion q;
    	q.setRPY(0,-1.57,1.57);
    	geometry_msgs::Quaternion odom_quat;
    	tf::quaternionTFToMsg(q, odom_quat);

    	_home_pose.orientation = odom_quat;
     	_home_pose.position.x = 0;  
		_home_pose.position.y = 0.4;
     	_home_pose.position.z = 0.02;
    	
    	// set the available poses
    	_grid_x_size = 0.1; // in meters
        _grid_y_size = 0.04;
        _grid_z_size = 0.1;
       
        _grid_x_offset = 0;
        _grid_y_offset = 0.26;
        _grid_z_offset = 0.02;
    	
    	geometry_msgs::Pose available_pose;
    	available_pose.orientation = odom_quat;
    	
    	for(int grid_z=0; grid_z<1; grid_z++)
    	{
			for(int grid_y=0; grid_y<=1; grid_y++)
			{
				for(int grid_x=-1; grid_x<=1; grid_x++)
				{
    				available_pose.position.x = _grid_x_offset + grid_x*_grid_x_size;
    				available_pose.position.y = _grid_y_offset + grid_y*_grid_y_size;
    				available_pose.position.z = _grid_z_offset + grid_z*_grid_z_size;
    				
    				_available_poses.push_back(available_pose);    				    				
    			}
    		}
    	}
    	
    	available_pose.position.x = _grid_x_offset;
		available_pose.position.y = _grid_y_offset;
		available_pose.position.z = _grid_z_offset + _grid_z_size;
		
		_available_poses.push_back(available_pose);
    	
    	
    }
    
    geometry_msgs::Pose Explorer::getHomePose()
    {
    	return _home_pose;
    }
        
        
    geometry_msgs::Pose Explorer::getAvailablePose(int available_pose_index)
    {
    	return _available_poses[available_pose_index];
    }
    
}
