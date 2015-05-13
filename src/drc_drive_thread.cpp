#include <yarp/os/all.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>
#include <assert.h>
#include <iostream>

#include "drc_drive_thread.h"
#include "drc_drive_constants.h"
#include <drc_shared/state_machine.hpp>
#include <drc_shared/draw_state_machine.hpp>


using namespace yarp::math;

using namespace yarp::os;
using namespace yarp::sig;
using namespace walkman::drc::drive;

using namespace OpenSoT::tasks::velocity;
using namespace OpenSoT::constraints::velocity;
using namespace OpenSoT::interfaces::yarp::tasks;

drc_drive_thread::drc_drive_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph ),
    command_interface( module_prefix ),
    status_interface( module_prefix ),
    q_left_desired(1),q_right_desired(1),
    drive_traj()
{

    //STATE MACHINE
    std::vector<std::tuple<state,std::string,state>> transition_table{
        //--------------initial state ----------+--------- command --------------------------------+------ final state--------- +
        std::make_tuple( state::idle            ,   WALKMAN_DRC_DRIVE_COMMAND_STEERING_WHEEL_DATA  ,    state::data_received	),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::data_received	,   WALKMAN_DRC_DRIVE_COMMAND_REACH                ,    state::reaching    	),
	//--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::reaching  	,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::reached          ),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::reached		,   WALKMAN_DRC_DRIVE_COMMAND_APPROACH             ,    state::approaching    	),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::approaching  	,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::approached	),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::approached  	,   WALKMAN_DRC_DRIVE_COMMAND_CLOSING_HANDS        ,    state::grasping		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::grasping  	,   WALKMAN_DRC_DRIVE_COMMAND_HAND_DONE		   ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_TURN_LEFT            ,    state::turning_left     ),
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_TURN_RIGHT           ,    state::turning_right    ),
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_ACCELERATE           ,    state::accelerating     ),
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_CLOSING_HANDS        ,    state::ungrasping	),
	//--------------------------------------+--------------------------------------------------+----------------------------+
	std::make_tuple( state::turning_left    ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::turning_right   ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::accelerating    ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::ungrasping	,   WALKMAN_DRC_DRIVE_COMMAND_HAND_DONE            ,    state::ungrasped	),
	//--------------------------------------+--------------------------------------------------+----------------------------+
	std::make_tuple( state::ungrasped	,   WALKMAN_DRC_DRIVE_COMMAND_MOVE_AWAY            ,    state::moving_away      ),
	//--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::moving_away     ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::moved_away	),
    };
    
    state_map[state::idle] = "idle";
    state_map[state::ready] = "ready";
    state_map[state::data_received] = "data_received";
    state_map[state::reaching] = "reaching";
    state_map[state::reached] = "reached";
    state_map[state::approaching] = "approaching";
    state_map[state::approached] = "approached";
    state_map[state::turning_left] = "turning_left";
    state_map[state::turning_right] = "turning_right";
    state_map[state::accelerating] = "accelerating";
    state_map[state::moving_away] = "moving_away";
    state_map[state::moved_away] = "moved_away";
    state_map[state::grasping] = "grasping";
    state_map[state::grasped] = "grasped";
    state_map[state::ungrasping] = "ungrasping";
    state_map[state::ungrasped] = "ungrasped";
    
    stateMachine.insert(transition_table);
    
    walkman::drc::draw_state_machine<state,std::string> drawer;
    drawer.insert(transition_table);
    drawer.draw_on_file("state_machine.gml",state_map);

    current_state = state::idle;

    seq_num = 0;
    status_seq_num = 0;
}

bool drc_drive_thread::custom_init()
{
    //  real time thread
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam ( pthread_self(), SCHED_FIFO, &thread_param );
    
    // start the status chain_interface
    status_interface.start();
    // notify the ready status
    if(status_definitions.status_to_code.count("ready"))
	status_interface.setStatus(status_definitions.status_to_code.at("ready") , status_seq_num++);
    else
	status_interface.setStatus( "ready" );	

    // sense
    robot.sense(input.q, input.q_dot, input.tau);
    auto max = model.iDyn3_model.getJointBoundMax();
    auto min = model.iDyn3_model.getJointBoundMin();
    for (int i=0;i<input.q.size();i++)
    {
        if (input.q[i]>max[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input.q[i]<<") is outside maximum bound: "<<max[i]<<std::endl;
        }
        if (input.q[i]<min[i])
        {
            std::cout<<"error: "<<model.getJointNames().at(i)<<"("<<input.q[i]<<") is outside minimum bound: "<<min[i]<<std::endl;
        }
    }
    // initializing output.q to current position
    output.q = input.q;

    // update model and com_model
    //model.iDyn3_model.setFloatingBaseLink(model.right_leg.end_effector_index);
    model.switchAnchorAndFloatingBase("Waist");
    model.updateiDyn3Model(input.q, true);
    
    // SOT INITIALIZATION
    
    // tasks initialization
    left_foot_task = Cartesian::Ptr( new Cartesian("6d::l_foot::world", input.q, model,"LFoot","world"));
    left_foot_task->setOrientationErrorGain(0.1);
    Yleft_foot_task = YCartesian::Ptr( new YCartesian(model.getRobotName(), get_module_prefix(), left_foot_task));
    
    //TODO check if it's needed to disable any joint to perform the acceleration task without disturbing the robot position in the car 

    left_arm_task = Cartesian::Ptr( new Cartesian("6d::l_arm::world", input.q, model,"LSoftHand","world"));
    left_arm_task->setOrientationErrorGain(0.1);
    Yleft_arm_task = YCartesian::Ptr( new YCartesian(model.getRobotName(), get_module_prefix(), left_arm_task));
    
    std::vector<bool> left_arm_active_joints = left_arm_task->getActiveJointsMask();
    for(unsigned int i = 0; i < left_arm_active_joints.size(); ++i)
    {
      std::vector<unsigned int>::iterator it = std::find(model.left_arm.joint_numbers.begin(), model.left_arm.joint_numbers.end(), i);
      if(it != model.left_arm.joint_numbers.end())
	  left_arm_active_joints[i] = true;
      else
	  left_arm_active_joints[i] = false;
    }
    left_arm_task->setActiveJointsMask(left_arm_active_joints);

    first_cartesian_tasks_list.push_back(left_foot_task);
    first_cartesian_tasks = OpenSoT::tasks::Aggregated::Ptr( new OpenSoT::tasks::Aggregated( first_cartesian_tasks_list,input.q.size()));
    
    second_cartesian_tasks_list.push_back(left_arm_task);
    second_cartesian_tasks = OpenSoT::tasks::Aggregated::Ptr( new OpenSoT::tasks::Aggregated( second_cartesian_tasks_list,input.q.size()));

    // constraints initialization
    joint_bounds = JointLimits::Ptr( new JointLimits(input.q, 
						      model.iDyn3_model.getJointBoundMax(),
						      model.iDyn3_model.getJointBoundMin()));
    
    velocity_bounds = VelocityLimits::Ptr( new VelocityLimits(   0.7, 
								static_cast<double>(get_thread_period())/1000.0, 
								input.q.size()));
    
    com_velocity_constraint = CoMVelocity::Ptr( new CoMVelocity(yarp::sig::Vector(3, 0.3), 
								static_cast<double>(get_thread_period())/1000.0, 
								input.q,
								model));
    
    first_cartesian_tasks->getConstraints().push_back(com_velocity_constraint);
    second_cartesian_tasks->getConstraints().push_back(com_velocity_constraint);
    
    bounds = OpenSoT::constraints::Aggregated_Ptr( new OpenSoT::constraints::Aggregated(joint_bounds, 
											 velocity_bounds,
											 input.q.size())); 
    
    stack.push_back(first_cartesian_tasks);
    stack.push_back(second_cartesian_tasks);
    
    solver = OpenSoT::solvers::QPOases_sot_Ptr( new OpenSoT::solvers::QPOases_sot(stack, bounds) );
    
    // set the robot controlled in position
    robot.left_arm.setPositionDirectMode();
    robot.left_leg.setPositionDirectMode();
    // hands in position with a ref speed
    if(robot.left_hand.isAvailable) { 
		robot.left_hand.setPositionDirectMode();
    }
    if(robot.right_hand.isAvailable) { 
		robot.right_hand.setPositionDirectMode();
    }
    
    drive_traj.init(left_arm_task,left_foot_task);
    
    return true;
}

void drc_drive_thread::init_actions(state new_state)
{
    if ( new_state == state::ready)
    {
    }
    if ( new_state == state::reaching)
    {
	drive_traj.init_reaching();
    }
    if ( new_state == state::approaching)
    {
	drive_traj.init_approaching();
    }
    if ( new_state == state::turning_left)
    {
	drive_traj.init_turning(drive_cmd.angle, drive_cmd.full_circle_time);
    }
    if ( new_state == state::turning_right)
    {
	drive_traj.init_turning(-drive_cmd.angle, drive_cmd.full_circle_time);
    }
    if ( new_state == state::accelerating)
    {
	drive_traj.init_accelerating(drive_cmd.gas_time);
    }
    if ( new_state == state::moving_away)
    {
	drive_traj.init_moving_away();
    }
}

void drc_drive_thread::run()
{   
    // get the command
    drive_cmd.command = WALKMAN_DRC_DRIVE_COMMAND_NONE;
    command_interface.getCommand(drive_cmd,seq_num);
    
    // evolve the state machine accordingly to the received command
    state new_state=stateMachine.evolve_state_machine(current_state,drive_cmd.command);
    if (current_state!=new_state)
    {
	init_actions(new_state);
	current_state = new_state;
    }
    
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_REACH ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Reaching the handle ..." << std::endl;
	drive_traj.set_controlled_end_effector(true,false);
        if(!move_hands(0.1)) std::cout<<"Hands not available "<<std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_APPROACH ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Approaching the handle ..." << std::endl;
	drive_traj.set_controlled_end_effector(true,false);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_ACCELERATE ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Accelerating ..." << std::endl;
	drive_traj.set_controlled_end_effector(false,true);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_TURN_LEFT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Turning LEFT ..." << std::endl;
	drive_traj.set_controlled_end_effector(true,false);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_TURN_RIGHT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Turning RIGHT ..." << std::endl;
	drive_traj.set_controlled_end_effector(true,false);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_MOVE_AWAY ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Moving away from steering wheel ..." << std::endl;
	drive_traj.set_controlled_end_effector(true,false);
    }
    if (drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_STEERING_WHEEL_DATA ) 
    {
	std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Steering wheel data received ..." << std::endl;
	drive_traj.get_steering_wheel_data(drive_cmd.frame, drive_cmd.drive_data, model);
    }
    if (drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_OPENING_HANDS) {
        if(!move_hands(0)) std::cout<<"Hands not available "<<std::endl;
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", opening the hands." << std::endl;
    }
    if (drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_CLOSING_HANDS) {
        if(!move_hands(1)) std::cout<<"Hands not available "<<std::endl;
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", closing the hands." << std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_HAND_DONE ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Hand done" << std::endl;
	drive_traj.get_rotation_radius();
    }
    
    sense();

    control_law();

    move();
    
    if (action_completed())
    {
      current_state=stateMachine.evolve_state_machine(current_state,WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE);
    } 
    
    if(status_definitions.status_to_code.count(state_map.at(current_state)))
	status_interface.setStatus(status_definitions.status_to_code.at(state_map.at(current_state)) , status_seq_num++);
    else
	status_interface.setStatus(state_map[current_state] , status_seq_num++);
}    

void drc_drive_thread::sense()
{
  // open loop, uses values computed by the solver the previous step
  input.q = output.q;
}

void drc_drive_thread::control_law()
{
  // update model
    model.updateiDyn3Model( input.q, true );
    // update SoT
    first_cartesian_tasks->update(input.q);
    second_cartesian_tasks->update(input.q);
    bounds->update(input.q);
    
    bool success=false;
    
    if ( current_state == state::reaching )
    {
	if(!drive_traj.perform_reaching()){ std::cout<<"ERROR REACHING"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::approaching )
    {
	if(!drive_traj.perform_approaching()){ std::cout<<"ERROR APPROACHING"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::turning_left )
    {
	if(!drive_traj.perform_turning()){ std::cout<<"ERROR TURNING LEFT"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::turning_right )
    {
	if(!drive_traj.perform_turning()){ std::cout<<"ERROR TURNING RIGHT"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::accelerating )
    {
	if(!drive_traj.perform_accelerating()){ std::cout<<"ERROR ACCELERATING"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::moving_away )
    {
	if(!drive_traj.perform_moving_away()){ std::cout<<"ERROR MOVING_AWAY"<<std::endl; success=false;}
	else success=true;
    }
    
    if(solver->solve(output.q_dot))
	output.q = input.q + output.q_dot;
    else {
	output.q_dot = 0.0;
	output.q = input.q + output.q_dot;
    }
}

void drc_drive_thread::move()
{   
    yarp::sig::Vector q_torso(3), q_left_arm(7), q_right_arm(7), q_left_leg(6), q_right_leg(6), q_head(2);
    robot.fromIdynToRobot(output.q, q_right_arm, q_left_arm, q_torso, q_right_leg, q_left_leg, q_head);
    
    robot.left_leg.move(q_left_leg);
    robot.left_arm.move(q_left_arm);
}

bool drc_drive_thread::action_completed()
{
    KDL::Vector LHand_position_error, LHand_orientation_error;
    KDL::Vector LFoot_position_error, LFoot_orientation_error;
    bool LHand_done = false;
    bool LFoot_done = false;
    bool using_arm, using_foot;
    
    drive_traj.get_controlled_end_effector(using_arm,using_foot);
   
    // checking the cartesian error according to the current active task and the status of trajectory
    if (using_arm && drive_traj.end_of_traj)
    {
      drive_traj.get_left_arm_cartesian_error(LHand_position_error, LHand_orientation_error);
      if( LHand_position_error.Norm()<0.01 && LHand_orientation_error.Norm()<0.1) 
      {
	  LHand_done = true;
      }
    }
    if (using_foot && drive_traj.end_of_traj)
    {
      drive_traj.get_left_foot_cartesian_error(LFoot_position_error, LFoot_orientation_error);
      if( LFoot_position_error.Norm()<0.01 && LFoot_orientation_error.Norm()<0.1)
      {
	  LFoot_done = true;
      }
    }
    
    return (using_arm?LHand_done:1) && (using_foot?LFoot_done:1);
}

bool drc_drive_thread::sense_hands(Vector& q_left_hand, Vector& q_right_hand)
{
      robot.senseHandsPosition(q_left_hand,q_right_hand);
      return true;
}

void drc_drive_thread::custom_release()
{
generic_thread::custom_release();
}

bool drc_drive_thread::move_hands(double close)
{	
      if(close <= 1.0 && close >= 0.0)
      {
            q_left_desired   = MIN_CLOSURE + close*(MAX_CLOSURE - MIN_CLOSURE); 
	    q_right_desired = MIN_CLOSURE;
            robot.moveHands(q_left_desired,q_right_desired);
            return true;
      }
      return false;
}

bool drc_drive_thread::hands_in_position()
{
    yarp::sig::Vector q_left(1);
    yarp::sig::Vector q_right(1);
    sense_hands(q_left, q_right);

    if ( fabs(q_left[0]-q_left_desired[0]) < 0.1 &&  fabs(q_right[0]-q_right_desired[0]) < 0.1 ) return true;	  
	return false;
}
