/**
 * \file table_obj_exp.h
 * \author Zhen Zeng (zengzhen@umich.edu)
 * \brief construct the home pose and available action set
 */
 
#ifndef TABLE_OBJECT_EXP_H
#define TABLE_OBJECT_EXP_H

#include <tf/tf.h>

namespace TableObjectAction
{
	class Explorer {
	public:
        /** \brief Constructor.
        *   \param[in] behavior representation of demonstrated behavior
        */
        Explorer(std::vector<std::vector<int> > behavior);
        
        /** \brief get home pose
        */
        geometry_msgs::Pose getHomePose();
        
        /** \brief get available pose
        *   \param[in]  available_pose_index the index of the pose to try out (0~11)
        */
        geometry_msgs::Pose getAvailablePose(int available_pose_index);
        
        /** \brief search within limited action options (home pose -> 12 different target poses) to achieve observed behavior
        *   \param[out] good_poses set of good target poses for completing each transition
        */
        //void searchAction(std::vector<std::vector<geometry_msgs::Pose> >& good_poses);

		/** \brief apply learned actions to acheive given qualitative state (indexed in behavior)
        *   \param[in]  qualStateIndex behavior[qualStateIndex] is the goal qual state
        */
        //void toQualState(int qualStateIndex);

	private:
		std::vector<std::vector<int> > _behavior;
	
        geometry_msgs::Pose _home_pose;
        std::vector<geometry_msgs::Pose> _available_poses;
        //std::vector<std::vector<geometry_msgs::Pose> > _good_poses;
        
        float _grid_x_size;
        float _grid_y_size;
        float _grid_z_size;
        
        float _grid_x_offset;
        float _grid_y_offset;
        float _grid_z_offset;
    };
}

#endif  // TABLE_OBJECT_EXP_H
