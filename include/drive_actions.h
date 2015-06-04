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
		std::vector<double> steering_wheel_data;
		
		// kdl frames for initial and final cartesian poses
		KDL::Frame world_InitialLhand,world_InitialLfoot;
		KDL::Frame world_FinalLhand,world_FinalLfoot;
		KDL::Frame world_LhandHome, world_LfootHome;
		KDL::Frame world_SteeringWheel, world_SteeringWheel_ZERO;
		KDL::Frame world_CenterOfRotation;
		KDL::Frame world_Handle;
		
		// local sot tasks
		OpenSoT::tasks::velocity::Cartesian::Ptr left_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr left_foot_task;
		
		// trajectory generators for all tasks
		trajectory_generator left_arm_generator;
		trajectory_generator left_arm_generator_bis;
		trajectory_generator left_foot_generator_push;
		trajectory_generator left_foot_generator_release;
		
		void compute_cartesian_error(KDL::Frame Start, KDL::Frame Target, KDL::Vector& position_error, KDL::Vector& orientation_error);

	    public:
		drive_actions();
		
		// initialization: receive sot task from the thread
		void init( OpenSoT::tasks::velocity::Cartesian::Ptr , 
			   OpenSoT::tasks::velocity::Cartesian::Ptr 
 			);
		
		bool get_steering_wheel_data(std::string ReferenceFrame, KDL::Frame steering_wheel_data, iDynUtils& model_);
		std::string ref_frame;
		
		// declaration of cartesian actions
		bool init_reaching();
		bool perform_reaching();
		
		bool init_approaching();
		bool perform_approaching();
		
		bool init_turning(double angle, double full_circle_time);
		bool perform_turning();
		
		bool init_accelerating(double gas_time, double gas_angle);
		bool perform_accelerating();
		
		bool init_moving_away_hand();
		bool perform_moving_away_hand();
		
		bool init_moving_away_foot();
		bool perform_moving_away_foot();
		
		bool init_rotating_foot(double foot_rotation);
		bool perform_rotating_foot();
		
		void get_left_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		void get_left_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error);
		
                void get_controlled_end_effector(bool& using_arm, bool& using_foot);
		void set_controlled_end_effector(bool left_arm, bool left_foot);
		
                void get_rotation_radius();
                
		bool left_arm_controlled;
		bool left_foot_controlled;
		
		bool end_of_traj;
		bool move_away_directly;
		
		double hand_traj_time;
		double foot_push_time;
		double foot_release_time;
                double foot_gas_time;
		
		double initialized_time;
		double rotation_radius;
		
		bool steeringwheel_init; // flag to save steeringwheel ZERO position
	    };
	}
    }
}

#endif