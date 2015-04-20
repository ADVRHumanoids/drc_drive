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
        std::make_tuple( state::idle            ,   WALKMAN_DRC_DRIVE_COMMAND_STEERING_WHEEL_DATA  ,    state::aligning_hand   ),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::aligning_hand   ,   WALKMAN_DRC_DRIVE_COMMAND_ALIGN_HAND           ,    state::ready            ),
        //--------------------------------------+--------------------------------------------------+----------------------------+
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_TURN_LEFT            ,    state::turning_left     ),
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_TURN_RIGHT           ,    state::turning_right    ),
	std::make_tuple( state::ready           ,   WALKMAN_DRC_DRIVE_COMMAND_ACCELERATE           ,    state::accelerating     ),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::turning_left    ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::turning_right   ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
        std::make_tuple( state::accelerating    ,   WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE          ,    state::ready		),
        //--------------------------------------+--------------------------------------------------+----------------------------+
    };
    
    state_map[state::idle] = "idle";
    state_map[state::ready] = "ready";
    state_map[state::aligning_hand] = "aligning_hand";
    state_map[state::aligned_hand] = "aligned_hand";
    state_map[state::turning_left] = "turning_left";
    state_map[state::turned_left] = "turned_left";
    state_map[state::turning_right] = "turning_right";
    state_map[state::turned_right] = "turned_right";
    state_map[state::accelerating] = "accelerating";
    state_map[state::accelerated] = "accelerated";
    
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
    status_interface.setStatus( "ready" );	

    // sense
    robot.sense(input.q, input.q_dot, input.tau);
    
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
    
    //TODO see if it's really needed 
//     std::vector<bool> left_foot_active_joints = left_foot_task->getActiveJointsMask();
//     for(unsigned int i = 0; i < left_foot_active_joints.size(); ++i)
//     {
//       std::vector<unsigned int>::iterator it = std::find(model.left_leg.joint_numbers.begin(), model.left_leg.joint_numbers.end(), i);
//       // for the acceleration task, only the pitch joint of the foot is needed
//       if(it == model.left_leg.joint_numbers.end() - 1)
// 	  left_foot_active_joints[i] = true;
//       else
// 	  left_foot_active_joints[i] = false;
//     }
//     left_foot_task->setActiveJointsMask(left_foot_active_joints);

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
    //for(unsigned int i = 0; i < 3; ++i)
        //left_arm_active_joints[model.torso.joint_numbers[i]] = true;
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
    robot.setPositionDirectMode();
    // hands in position with a ref speed
    if(robot.left_hand.isAvailable) { 
		robot.left_hand.setPositionMode();
		robot.left_hand.setReferenceSpeed(0.3);
    }
    if(robot.right_hand.isAvailable) { 
		robot.right_hand.setPositionMode();
		robot.right_hand.setReferenceSpeed(0.3);
    }
    
    drive_traj.init(left_arm_task,right_arm_task,left_foot_task);
    
    return true;
}

void drc_drive_thread::init_actions(state new_state)
{
    if ( new_state == state::ready)
    {
    }
    if ( new_state == state::aligning_hand)
    {
	drive_traj.init_aligning_hand();
    }
    if ( new_state == state::turning_left)
    {
	steering_angle = -drive_cmd.angle;
	drive_traj.init_turning_left(steering_angle);
    }
    if ( new_state == state::turning_right)
    {
	steering_angle = drive_cmd.angle;
	drive_traj.init_turning_right(steering_angle);
    }
    if ( new_state == state::accelerating)
    {
	drive_traj.init_accelerating();
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
    
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_ALIGN_HAND ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Aligning hand with X normal wrt steering wheel ..." << std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_ACCELERATE ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Accelerating ..." << std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_TURN_LEFT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Turning LEFT ..." << std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_TURN_RIGHT ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Turning RIGHT ..." << std::endl;
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_RIGHT_HAND ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Controlling now RIGHT hand ..." << std::endl;
	drive_traj.set_controlled_arms(false,true);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_LEFT_HAND ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Controlling now LEFT hand ..." << std::endl;
	drive_traj.set_controlled_arms(true,false);
    }
    if ( drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_BOTH_HANDS ) {
        std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Controlling now BOTH hands ..." << std::endl;
	drive_traj.set_controlled_arms(true,true);
    }
    if (drive_cmd.command == WALKMAN_DRC_DRIVE_COMMAND_STEERING_WHEEL_DATA ) 
    {
	std::cout << "Command ["<<seq_num<<"]: "<<drive_cmd.command<<", Steering wheel data received ..." << std::endl;
	drive_traj.get_steering_wheel_data(drive_cmd.frame, drive_cmd.drive_data, drive_cmd.radius, model);
    }
    
    sense();

    control_law();

    move();
    
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
    
    if ( current_state == state::aligning_hand )
    {
	if(!drive_traj.perform_aligning_hand()){ std::cout<<"ERROR ALIGNING HAND"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::turning_left )
    {
	if(!drive_traj.perform_turning_left()){ std::cout<<"ERROR TURNING LEFT"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::turning_right )
    {
	if(!drive_traj.perform_turning_right()){ std::cout<<"ERROR TURNING RIGHT"<<std::endl; success=false;}
	else success=true;
    }
    if ( current_state == state::accelerating )
    {
	if(!drive_traj.perform_accelerating()){ std::cout<<"ERROR ACCELERATING"<<std::endl; success=false;}
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
    // NOTE to control the whole robot
    robot.move(output.q);
}

bool drc_drive_thread::move_hands(double close)
{	
  if (close <= 1.0 && close >= 0.0)
      if(robot.hasHands())
      {
            if (drive_traj.left_arm_controlled) q_left_desired   = 0.3+ close*(1.3-0.3); // NOTE these values work for simulation and real hands
            if (drive_traj.right_arm_controlled) q_right_desired = 0.3+ close*(1.3-0.3);
            robot.moveHands(q_left_desired,q_right_desired);
            return true;
      }
  else
  {
    std::cout<<"closing amount is out of feasible bounds"<<std::endl;
  }
   return false;
}

bool drc_drive_thread::sense_hands(Vector& q_left_hand, Vector& q_right_hand)
{
      if(robot.hasHands())
      {
	  robot.senseHandsPosition(q_left_hand,q_right_hand);
	  return true;
      }
      return false;
}

void drc_drive_thread::custom_release()
{
generic_thread::custom_release();
}



bool drc_drive_thread::hands_in_position()
{
    if(robot.hasHands())
    {
	yarp::sig::Vector q_left(1);
	yarp::sig::Vector q_right(1);
	sense_hands(q_left, q_right);

	if ( fabs(q_left[0]-q_left_desired[0]) < 0.1 &&  fabs(q_right[0]-q_right_desired[0]) < 0.1 ) return true;	  
	    return false;
    }
    else return true;
}
