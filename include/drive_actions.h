#ifndef DRIVE_ACTIONS_H_
#define DRIVE_ACTIONS_H_

#include "drive_interface.h"
#include <OpenSoT/OpenSoT.h>
#include <trajectory_generator/trajectory_generator.h>
#include <idynutils/cartesian_utils.h>
#include <mutex>

#include <OpenSoT/interfaces/yarp/tasks/YCartesian.h>

namespace walkman
{
    namespace drc
    {
        namespace drive
        {
	    class drive_actions {
	    private:
		
		/**
                 * @brief data of the steering wheel: X Y Z roll pitch yaw radius
                 */
		std::vector<double> drive_data;
		
		// kdl frames for initial and final cartesian poses
		KDL::Frame world_InitialRhand,world_InitialLhand,world_InitialLfoot;
		KDL::Frame world_FinalRhand,world_FinalLhand,world_FinalLfoot;
		
		// local sot tasks
		OpenSoT::tasks::velocity::Cartesian::Ptr left_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr right_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr left_foot_task;
		
		// trajectory generators for all tasks
		trajectory_generator left_arm_generator;
		trajectory_generator right_arm_generator;
		trajectory_generator left_foot_generator;
		
		double initialized_time;
		
		void compute_cartesian_error(KDL::Frame Start, KDL::Frame Target, KDL::Vector& position_error, KDL::Vector& orientation_error);

	    public:
		drive_actions();
		
		// initialization: receive sot task from the thread
		void init( OpenSoT::tasks::velocity::Cartesian::Ptr , 
			   OpenSoT::tasks::velocity::Cartesian::Ptr ,
			   OpenSoT::tasks::velocity::Cartesian::Ptr 
 			);
		
		bool get_drive_data(std::string ReferenceFrame, KDL::Frame drive_data_, double radius, iDynUtils& model_);
		std::string ref_frame;
		                
		void set_controlled_arms(bool left_arm, bool right_arm);
		
		//double steering_angle; //steering wheel angle in DEG
		
		// declaration of cartesian actions
		bool init_turning_left(double angle);
		bool perform_turning_left();
		
		bool init_turning_right(double angle);
		bool perform_turning_right();
		
		bool init_accelerating();
		bool perform_accelerating();
		
		bool init_decelerating();
		bool perform_decelerating();
		
		void get_left_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_right_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_left_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		
                void get_controlled_arms(bool& using_left, bool& using_right);
		
		bool left_arm_controlled;
		bool right_arm_controlled;
		
	    };
	}
    }
}

#endif