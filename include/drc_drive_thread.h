/* Copyright [2014,2015] [Enrico Corvaglia, Luca Muratore, Corrado Pavan, Alessandro Settimi]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#ifndef drc_drive_THREAD_H_
#define drc_drive_THREAD_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <idynutils/yarp_single_chain_interface.h>

#include <idynutils/comanutils.h>

#include <drc_shared/yarp_msgs/drive_msg.h>
#include <drc_shared/state_machine.hpp>

#include "drive_interface.h"
#include "drive_actions.h"

#include <GYM/control_thread.hpp>
#include <GYM/yarp_command_interface.hpp>
#include <GYM/yarp_status_interface.h>

#include <OpenSoT/tasks/velocity/Postural.h>
#include <OpenSoT/tasks/velocity/Cartesian.h>
#include <OpenSoT/tasks/velocity/Manipulability.h>
#include <OpenSoT/utils/VelocityAllocation.h>
#include <OpenSoT/tasks/velocity/MinimizeAcceleration.h>
#include <OpenSoT/SubTask.h>

#include <OpenSoT/OpenSoT.h>
#include <yarp/math/Math.h>

#include "drc_shared/module_statuses/drc_drive_statuses.h"

// SoT task and constraints declaration

namespace OpenSoT
{
    namespace solvers{
        typedef boost::shared_ptr<QPOases_sot> QPOases_sot_Ptr ;
    }
    namespace constraints {
            typedef boost::shared_ptr<Aggregated> Aggregated_Ptr ;
		namespace velocity {
		    typedef boost::shared_ptr<CoMVelocity> CoMVelocity_Ptr ;
		    typedef boost::shared_ptr<ConvexHull> ConvexHull_Ptr ;

		    typedef boost::shared_ptr<JointLimits> JointLimits_Ptr ;
		    typedef boost::shared_ptr<VelocityLimits> VelocityLimits_Ptr ;
    }
    }
}

/**
 * @brief drc_drive control thread
 * 
 **/
namespace walkman
{
    namespace drc
    {
        namespace drive
        {
	    class drc_drive_thread : public control_thread
	    {
	    private:   
		state_machine<state> stateMachine;
		
		drive_actions drive_traj;

		walkman::yarp_custom_command_interface<drive_msg> command_interface;
		int seq_num;
		
		walkman::drc::drive::status_definitions status_definitions;
		walkman::yarp_status_interface status_interface;
		int status_seq_num;
		
		drive_msg drive_cmd;
				
		void control_law();
		
		robot_state_input input;
		
		robot_state_output output;
		
		state current_state;
		
		void init_actions(state new_state, state last_state);
		
		yarp::sig::Vector wb_input_q, wb_output_q;
		yarp::sig::Vector left_hand_input_q, right_hand_input_q;
		yarp::sig::Vector q_hands_desired;

			  
		std::map<state,std::string> state_map;
		
		// SOT tasks and constraints
		OpenSoT::solvers::QPOases_sot_Ptr solver;
                OpenSoT::constraints::Aggregated_Ptr bounds;
                OpenSoT::solvers::QPOases_sot::Stack stack;
		
                OpenSoT::tasks::velocity::Cartesian::Ptr left_arm_task;
		OpenSoT::tasks::velocity::Cartesian::Ptr left_foot_task;
		
		OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr Yleft_arm_task;
		OpenSoT::interfaces::yarp::tasks::YCartesian::Ptr Yleft_foot_task;

                OpenSoT::constraints::velocity::CoMVelocity::Ptr com_velocity_constraint;
                OpenSoT::constraints::velocity::ConvexHull_Ptr com_convex_hull_constraint;

                OpenSoT::constraints::velocity::JointLimits::Ptr joint_bounds;
                OpenSoT::constraints::velocity::VelocityLimits::Ptr velocity_bounds;

                OpenSoT::tasks::Aggregated::Ptr first_cartesian_tasks;
                std::list<OpenSoT::tasks::Aggregated::TaskPtr> first_cartesian_tasks_list;
		
		OpenSoT::tasks::Aggregated::Ptr second_cartesian_tasks;
                std::list<OpenSoT::tasks::Aggregated::TaskPtr> second_cartesian_tasks_list;

	    public:
		
		/**
		* @brief constructor
		* 
		* @param module_prefix the prefix of the module
		* @param rf resource finderce
		* @param ph param helper
		*/
		drc_drive_thread( std::string module_prefix, yarp::os::ResourceFinder rf, std::shared_ptr<paramHelp::ParamHelperServer> ph );  
		
		/**
		* @brief drc_drive control thread initialization
		* 
		* @return true on succes, false otherwise
		*/
		virtual bool custom_init();
		
		/**
		* @brief drc_drive control thread main loop
		* 
		*/
		virtual void run();
		
		virtual void custom_release();

		/**
		  * @brief sense function
		  */
		void sense();

		/**
		  * @brief move function
		  */
		void move();
		
		bool action_completed();
		
		bool move_hands(double close);
                
                bool hands_in_position();
		
		bool sense_hands(yarp::sig::Vector &q_left_hand, yarp::sig::Vector &q_right_hand);
	    };

	}
    }
}

#endif
