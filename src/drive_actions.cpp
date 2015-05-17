#include "drive_actions.h"
#include <kdl/chain.hpp>
#include <tf_conversions/tf_kdl.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <iCub/iDynTree/yarp_kdl.h>
#include <mutex>
#include "drive_interface.h"

#define X_INDEX 0
#define Y_INDEX 1
#define Z_INDEX 2
#define ROLL_INDEX 3
#define PITCH_INDEX 4
#define YAW_INDEX 5
#define RADIUS_INDEX 6

#define STEERING_WHEEL_RADIUS 0.185

#define HAND_HANDLE_REACHING_OFFSET_X 0.08
#define HAND_HANDLE_REACHING_OFFSET_Y 0.08
#define HAND_HANDLE_REACHING_OFFSET_Z 0

#define HAND_HANDLE_OFFSET_X 0.02
#define HAND_HANDLE_OFFSET_Y 0.005
#define HAND_HANDLE_OFFSET_Z 0.05

#define HANDLE_LENGTH 0.185
#define HANDLE_INNER_RADIUS 0.025
#define HANDLE_OUTER_RADIUS 0.04

#define HANDLE_SAFETY_OFFSET_X 0.10
#define HANDLE_SAFETY_OFFSET_Y 0.04

#define DISTANCE_STEERINGWHEEL_HANDLE 0.115
#define ACCELERATION_SAFETY_ANGLE 5

walkman::drc::drive::drive_actions::drive_actions()
{
    steering_wheel_data.resize(7);
    steering_wheel_data[X_INDEX] = 0.0;
    steering_wheel_data[Y_INDEX] = 0.0;
    steering_wheel_data[Z_INDEX] = 0.0;
    steering_wheel_data[ROLL_INDEX] = 0.0;
    steering_wheel_data[PITCH_INDEX] = 0.0;
    steering_wheel_data[YAW_INDEX] = 0.0;
    steering_wheel_data[RADIUS_INDEX] = STEERING_WHEEL_RADIUS;
    
    left_arm_controlled = false;
    left_foot_controlled = false;
    
    rotation_radius = DISTANCE_STEERINGWHEEL_HANDLE;
    steeringwheel_init = false;
}

void walkman::drc::drive::drive_actions::set_controlled_end_effector(bool left_arm, bool left_foot)
{
    left_arm_controlled = left_arm;
    left_foot_controlled = left_foot;
}

void walkman::drc::drive::drive_actions::init(OpenSoT::tasks::velocity::Cartesian::Ptr l_arm_task,
					      OpenSoT::tasks::velocity::Cartesian::Ptr l_foot_task
					     )
{
    left_arm_task = l_arm_task;
    left_foot_task = l_foot_task;
}

void walkman::drc::drive::drive_actions::get_controlled_end_effector(bool& using_arm, bool& using_foot)
{
    using_arm = left_arm_controlled;
    using_foot = left_foot_controlled;
}

void walkman::drc::drive::drive_actions::compute_cartesian_error(KDL::Frame Start, KDL::Frame Target, KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    quaternion q,qd;
    
    Start.M.GetQuaternion(q.x, q.y, q.z, q.w);
    Target.M.GetQuaternion(qd.x, qd.y, qd.z, qd.w);

    //This is needed to move along the short path in the quaternion error
    if(quaternion::dot(q, qd) < 0.0) q = q.operator *(-1.0);

    KDL::Vector xerr_p; // Cartesian position error
    KDL::Vector xerr_o; // Cartesian orientation error

    position_error = Target.p - Start.p;
    orientation_error = quaternion::error(q, qd);  
}

void walkman::drc::drive::drive_actions::get_left_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{    
    KDL::Frame world_CurrentLarm;
    YarptoKDL(left_arm_task->getActualPose(),world_CurrentLarm);
    compute_cartesian_error(world_CurrentLarm,world_FinalLhand,position_error,orientation_error);
}

void walkman::drc::drive::drive_actions::get_left_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{    
    KDL::Frame world_CurrentLfoot;
    YarptoKDL(left_foot_task->getActualPose(),world_CurrentLfoot);
    // using world_InitialLfoot as target because we want the foot to return to the starting position after pusing gas pedal
    compute_cartesian_error(world_CurrentLfoot,world_InitialLfoot,position_error,orientation_error);
}

bool walkman::drc::drive::drive_actions::get_steering_wheel_data(std::string Frame, KDL::Frame steering_wheel_data_, iDynUtils& model_)
{    
    KDL::Frame SteeringWheel_Handle;
    ref_frame = Frame;
    if (ref_frame != "world")
    {
	KDL::Frame Frame_data, Anchor_World, World_data, Anchor_Frame;
	Frame_data = steering_wheel_data_;
	Anchor_World = model_.getAnchor_T_World();
	Anchor_Frame = model_.iDyn3_model.getPositionKDL(model_.iDyn3_model.getLinkIndex(model_.getAnchor()),model_.iDyn3_model.getLinkIndex(ref_frame));
	World_data = Anchor_World.Inverse() * Anchor_Frame * Frame_data;
	steering_wheel_data[X_INDEX] = World_data.p.x();
	steering_wheel_data[Y_INDEX] = World_data.p.y();
	steering_wheel_data[Z_INDEX] = World_data.p.z();
	double ro,pi,ya;
	World_data.M.GetRPY(ro,pi,ya);
	steering_wheel_data[ROLL_INDEX] = ro;
	steering_wheel_data[PITCH_INDEX] = pi;
	steering_wheel_data[YAW_INDEX] = ya;
    } 
    else
    {
	steering_wheel_data[X_INDEX] = steering_wheel_data_.p.x();
	steering_wheel_data[Y_INDEX] = steering_wheel_data_.p.y();
	steering_wheel_data[Z_INDEX] = steering_wheel_data_.p.z();
	double ro,pi,ya;
	steering_wheel_data_.M.GetRPY(ro,pi,ya);
	steering_wheel_data[ROLL_INDEX] = ro;
	steering_wheel_data[PITCH_INDEX] = pi;
	steering_wheel_data[YAW_INDEX] = ya;
    }
    
    world_SteeringWheel.p = KDL::Vector(steering_wheel_data[X_INDEX],steering_wheel_data[Y_INDEX],steering_wheel_data[Z_INDEX]);
    world_SteeringWheel.M = KDL::Rotation::RPY(steering_wheel_data[ROLL_INDEX],steering_wheel_data[PITCH_INDEX],steering_wheel_data[YAW_INDEX]);
    // now adjusting the axis of the steering wheel to prepare them for the circular trajectories (X-axis facing the driver)
    world_SteeringWheel.M = world_SteeringWheel.M*KDL::Rotation::RotY(-90*DEG2RAD);
    
    SteeringWheel_Handle.p = KDL::Vector(0,0,-DISTANCE_STEERINGWHEEL_HANDLE);
    SteeringWheel_Handle.M = KDL::Rotation::Identity();
    
    world_Handle = world_SteeringWheel*SteeringWheel_Handle;
    
    std::cout<<"Steering Wheel Data Received:"<<std::endl;
    std::cout<<"| x: "<<steering_wheel_data[X_INDEX]<<std::endl;
    std::cout<<"| y: "<<steering_wheel_data[Y_INDEX]<<std::endl;
    std::cout<<"| z: "<<steering_wheel_data[Z_INDEX]<<std::endl;
    std::cout<<"| roll: "<<steering_wheel_data[ROLL_INDEX]<<std::endl;
    std::cout<<"| pitch: "<<steering_wheel_data[PITCH_INDEX]<<std::endl;
    std::cout<<"| yaw: "<<steering_wheel_data[YAW_INDEX]<<std::endl;
    std::cout<<"| radius: "<<steering_wheel_data[RADIUS_INDEX]<<std::endl;
    
    // saving the zero position of the steering wheel 
    if (!steeringwheel_init)
    {
      world_SteeringWheel_ZERO = world_SteeringWheel;
      
      KDL::Frame Hand_translation;
      Hand_translation.p = KDL::Vector(HANDLE_LENGTH+HANDLE_SAFETY_OFFSET_X,STEERING_WHEEL_RADIUS,0);
      Hand_translation.M = KDL::Rotation::Identity();
      
      world_LhandHome = world_SteeringWheel_ZERO*Hand_translation;     
      steeringwheel_init = true;
      
      std::cout<<"Steering Wheel with Upright Axis stored!"<<std::endl;
    }
    
    return true;
}

void walkman::drc::drive::drive_actions::get_rotation_radius()
{
    KDL::Frame world_CurrentLhand, Center_translation, Handle_Hand, world_Handle_uprightAxis;
    YarptoKDL(left_arm_task->getActualPose(), world_CurrentLhand);
    
    world_Handle_uprightAxis = world_Handle;
    world_Handle_uprightAxis.M = world_SteeringWheel_ZERO.M;    
    Handle_Hand = world_Handle_uprightAxis.Inverse()*world_CurrentLhand;
    
    Center_translation.p = Handle_Hand.p;
    Center_translation.M = KDL::Rotation::Identity();
    world_CenterOfRotation = world_SteeringWheel_ZERO*Center_translation;
    
    rotation_radius = (world_CurrentLhand.p - world_CenterOfRotation.p).Norm();
    std::cout<<"Rotation RADIUS: "<<rotation_radius<<std::endl;
  
}

bool walkman::drc::drive::drive_actions::init_reaching()
{
    end_of_traj = false;
    hand_traj_time = 4.0;
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    YarptoKDL(left_foot_task->getActualPose(), world_InitialLfoot);
    
    world_FinalLfoot.p = world_InitialLfoot.p;
    world_FinalLfoot.M = world_InitialLfoot.M*KDL::Rotation::RotY(-ACCELERATION_SAFETY_ANGLE*DEG2RAD);
    left_foot_generator_push.line_initialize(hand_traj_time,world_InitialLfoot,world_FinalLfoot);
    
    KDL::Frame Hand_translation_HIGH, Hand_translation_LOW, world_tempLhand;
    
    world_Handle.M = world_SteeringWheel_ZERO.M;
    
    Hand_translation_HIGH.p = KDL::Vector(HANDLE_LENGTH/2 + HAND_HANDLE_REACHING_OFFSET_X, HAND_HANDLE_REACHING_OFFSET_Y, 0);
    Hand_translation_HIGH.M = KDL::Rotation::Identity();
    world_tempLhand = world_Handle*Hand_translation_HIGH;
    
    Hand_translation_LOW.p = KDL::Vector(HANDLE_LENGTH/2 + HAND_HANDLE_OFFSET_X, HAND_HANDLE_REACHING_OFFSET_Y, 0);
    Hand_translation_LOW.M = KDL::Rotation::Identity();
    world_FinalLhand = world_Handle*Hand_translation_LOW;
    
    left_arm_generator.line_initialize(hand_traj_time,world_InitialLhand,world_tempLhand);
    left_arm_generator_bis.line_initialize(hand_traj_time,world_tempLhand,world_FinalLhand);
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_reaching()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_LH, Xd_LF;
    KDL::Twist dXd_LH, dXd_LF;
    
    if (time <= hand_traj_time){
      left_arm_generator.line_trajectory(time, Xd_LH, dXd_LH);
      left_foot_generator_push.line_trajectory(time, Xd_LF, dXd_LF);
      left_foot_task->setReference(KDLtoYarp_position(Xd_LF));
    }
    else
      left_arm_generator_bis.line_trajectory(time-hand_traj_time, Xd_LH, dXd_LH);
    
    left_arm_task->setReference(KDLtoYarp_position(Xd_LH));

    if (!end_of_traj)
    {
      if (time >= hand_traj_time*2)
        end_of_traj = true;
    }
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_approaching()
{
    end_of_traj = false;
    hand_traj_time = 3.0;
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    KDL::Frame Hand_translation;
    
    world_Handle.M = world_SteeringWheel_ZERO.M;
    
    Hand_translation.p = KDL::Vector(HANDLE_LENGTH/2,HANDLE_INNER_RADIUS,0);
    Hand_translation.M = KDL::Rotation::Identity();
    
    world_FinalLhand = world_Handle*Hand_translation;
    
    left_arm_generator.line_initialize(hand_traj_time,world_InitialLhand,world_FinalLhand);
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_approaching()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_LH;
    KDL::Twist dXd_LH;
   
    left_arm_generator.line_trajectory(time, Xd_LH, dXd_LH);
    left_arm_task->setReference(KDLtoYarp_position(Xd_LH));
    
    if (!end_of_traj)
    {
      if (time >= hand_traj_time)
	end_of_traj = true;
    }
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_moving_away()
{
    end_of_traj = false;
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    KDL::Frame Hand_translation, world_tempLhand, SteeringWheel_Hand;
    
    SteeringWheel_Hand = world_SteeringWheel_ZERO.Inverse()*world_InitialLhand;
    // zeroing the X component is like projecting the hand frame on the steering wheel YZ plane
    SteeringWheel_Hand.p(X_INDEX) = 0;
    
    Hand_translation.p = KDL::Vector(HANDLE_LENGTH+HANDLE_SAFETY_OFFSET_X,HANDLE_SAFETY_OFFSET_Y,0) + SteeringWheel_Hand.p;
    Hand_translation.M = KDL::Rotation::Identity();
    
    world_tempLhand = world_SteeringWheel_ZERO*Hand_translation;
    world_FinalLhand = world_LhandHome;
      
    if (!move_away_directly)
    {
      hand_traj_time = 10.0;
      left_arm_generator.line_initialize(hand_traj_time/2,world_InitialLhand,world_tempLhand);
      left_arm_generator_bis.line_initialize(hand_traj_time/2,world_tempLhand,world_FinalLhand);
    }
    else
    {
      hand_traj_time = 5.0;
      left_arm_generator.line_initialize(hand_traj_time,world_InitialLhand,world_FinalLhand);
    }
    
    initialized_time=yarp::os::Time::now();
    return true;
}

bool walkman::drc::drive::drive_actions::perform_moving_away()
{
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_LH;
    KDL::Twist dXd_LH;
   
    if (!move_away_directly)
    {
      if (time <= (hand_traj_time/2))
	left_arm_generator.line_trajectory(time, Xd_LH, dXd_LH);
      else
	left_arm_generator_bis.line_trajectory(time-(hand_traj_time/2), Xd_LH, dXd_LH);
    }
    else
    {
      left_arm_generator.line_trajectory(time, Xd_LH, dXd_LH);
    }
    
    left_arm_task->setReference(KDLtoYarp_position(Xd_LH));
    
    if (!end_of_traj)
    {
      if (time >= hand_traj_time)
        end_of_traj = true;
    }
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_turning(double angle, double full_circle_time)
{   
    end_of_traj = false;
    hand_traj_time = full_circle_time*abs(angle/360);  // time is parametrized wrt the commanded angle
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    left_arm_generator.circle_initialize(hand_traj_time, rotation_radius, angle*DEG2RAD, world_InitialLhand, world_CenterOfRotation);
    
    // getting the hand target
    KDL::Twist dummy;
    left_arm_generator.circle_trajectory(hand_traj_time,world_FinalLhand,dummy);
    world_FinalLhand.M = world_InitialLhand.M;
    
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_turning()
{   
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_LH;
    KDL::Twist dXd_LH;
   
    left_arm_generator.circle_trajectory(time, Xd_LH, dXd_LH);
    // forcing the initial orientation during the trajectory
    Xd_LH.M = world_InitialLhand.M;   
    left_arm_task->setReference(KDLtoYarp_position(Xd_LH));
    
    if (!end_of_traj)
    {
      if (time >= hand_traj_time)
	end_of_traj = true;
    }
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_accelerating(double gas_time)
{        
    end_of_traj = false;
    foot_push_time = 0.5;
    foot_release_time = 0.5;
    foot_gas_time = gas_time;
    
    double left_foot_pitch = 15*DEG2RAD;
    YarptoKDL(left_foot_task->getActualPose(), world_InitialLfoot);
    
    world_FinalLfoot.p = world_InitialLfoot.p;
    world_FinalLfoot.M = world_InitialLfoot.M*KDL::Rotation::RotY(left_foot_pitch);
    
    left_foot_generator_push.line_initialize(foot_push_time,world_InitialLfoot,world_FinalLfoot);
    left_foot_generator_release.line_initialize(foot_release_time,world_FinalLfoot,world_InitialLfoot);
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_accelerating()
{   
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_LF;
    KDL::Twist dXd_LF;
   
    if (time <= (foot_push_time + foot_gas_time))
      left_foot_generator_push.line_trajectory(time, Xd_LF, dXd_LF);
    else
      left_foot_generator_release.line_trajectory(time-(foot_push_time + foot_gas_time), Xd_LF, dXd_LF);
    
    
    Xd_LF.p = world_InitialLfoot.p;
    left_foot_task->setReference(KDLtoYarp_position(Xd_LF));
    
    if (!end_of_traj)
    {
      if (time >= (foot_push_time + foot_gas_time + foot_release_time))
	end_of_traj = true;
    }
   
    return true;
}

bool walkman::drc::drive::drive_actions::init_moving_foot()
{

}

bool walkman::drc::drive::drive_actions::perform_moving_foot()
{

}


