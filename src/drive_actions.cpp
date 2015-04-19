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

#define STEERING_WHEEL_RADIUS 0.18

walkman::drc::drive::drive_actions::drive_actions()
{
    drive_data.resize(7);
    drive_data[X_INDEX] = 0.0;
    drive_data[Y_INDEX] = 0.0;
    drive_data[Z_INDEX] = 0.0;
    drive_data[ROLL_INDEX] = 0.0;
    drive_data[PITCH_INDEX] = 0.0;
    drive_data[YAW_INDEX] = 0.0;
    drive_data[RADIUS_INDEX] = STEERING_WHEEL_RADIUS;
    
    left_arm_controlled = true;
    right_arm_controlled = false;
}

void walkman::drc::drive::drive_actions::set_controlled_arms(bool left_arm, bool right_arm)
{
    left_arm_controlled = left_arm;
    right_arm_controlled = right_arm;
}

void walkman::drc::drive::drive_actions::init(OpenSoT::tasks::velocity::Cartesian::Ptr l_arm_task, 
					      OpenSoT::tasks::velocity::Cartesian::Ptr r_arm_task,
					      OpenSoT::tasks::velocity::Cartesian::Ptr l_foot_task
					     )
{
    left_arm_task = l_arm_task;
    right_arm_task = r_arm_task;
    left_foot_task = l_foot_task;
}

void walkman::drc::drive::drive_actions::get_controlled_arms(bool& using_left, bool& using_right)
{
    using_left = left_arm_controlled;
    using_right = right_arm_controlled;
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

void walkman::drc::drive::drive_actions::get_right_arm_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    KDL::Frame world_CurrentRarm;
    YarptoKDL(right_arm_task->getActualPose(),world_CurrentRarm);
    compute_cartesian_error(world_CurrentRarm,world_FinalRhand,position_error,orientation_error);
}

void walkman::drc::drive::drive_actions::get_left_foot_cartesian_error(KDL::Vector& position_error, KDL::Vector& orientation_error)
{
    KDL::Frame world_CurrentLfoot;
    YarptoKDL(left_foot_task->getActualPose(),world_CurrentLfoot);
    compute_cartesian_error(world_CurrentLfoot,world_FinalLfoot,position_error,orientation_error);
}

bool walkman::drc::drive::drive_actions::get_drive_data(std::string Frame, KDL::Frame drive_data_, double radius, iDynUtils& model_)
{
    ref_frame = Frame;
    drive_data[RADIUS_INDEX] = radius;
    if (ref_frame != "world")
    {
	KDL::Frame Frame_data, Anchor_World, World_data, Anchor_Frame;
	Frame_data = drive_data_;
	Anchor_World = model_.getAnchor_T_World();
	Anchor_Frame = model_.iDyn3_model.getPositionKDL(model_.iDyn3_model.getLinkIndex(model_.getAnchor()),model_.iDyn3_model.getLinkIndex(ref_frame));
	World_data = Anchor_World.Inverse() * Anchor_Frame * Frame_data;
	drive_data[X_INDEX] = World_data.p.x();
	drive_data[Y_INDEX] = World_data.p.y();
	drive_data[Z_INDEX] = World_data.p.z();
	double ro,pi,ya;
	World_data.M.GetRPY(ro,pi,ya);
	drive_data[ROLL_INDEX] = ro;
	drive_data[PITCH_INDEX] = pi;
	drive_data[YAW_INDEX] = ya;
    } 
    else
    {
	drive_data[X_INDEX] = drive_data_.p.x();
	drive_data[Y_INDEX] = drive_data_.p.y();
	drive_data[Z_INDEX] = drive_data_.p.z();
	double ro,pi,ya;
	drive_data_.M.GetRPY(ro,pi,ya);
	drive_data[ROLL_INDEX] = ro;
	drive_data[PITCH_INDEX] = pi;
	drive_data[YAW_INDEX] = ya;	
    }

    std::cout<<"Drive Data Received:"<<std::endl;
    std::cout<<"| x: "<<drive_data[X_INDEX]<<std::endl;
    std::cout<<"| y: "<<drive_data[Y_INDEX]<<std::endl;
    std::cout<<"| z: "<<drive_data[Z_INDEX]<<std::endl;
    std::cout<<"| roll: "<<drive_data[ROLL_INDEX]<<std::endl;
    std::cout<<"| pitch: "<<drive_data[PITCH_INDEX]<<std::endl;
    std::cout<<"| yaw: "<<drive_data[YAW_INDEX]<<std::endl;
    std::cout<<"| radius: "<<drive_data[RADIUS_INDEX]<<std::endl;
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_turning_left(double angle)
{   
    double time_f = 15.0;
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    KDL::Frame world_SteeringWheel;
    
    world_SteeringWheel.p.data[0] = world_InitialLhand.p.data[0] + 0.05;
    world_SteeringWheel.p.data[1] = world_InitialLhand.p.data[1] - STEERING_WHEEL_RADIUS;
    world_SteeringWheel.p.data[2] = world_InitialLhand.p.data[2];
    world_SteeringWheel.M = KDL::Rotation::Identity();
    
    left_arm_generator.circle_initialize(time_f, STEERING_WHEEL_RADIUS, angle*DEG2RAD, world_InitialLhand, world_SteeringWheel);
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_turning_left()
{   
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L;
    KDL::Twist dXd_L;
   
    left_arm_generator.circle_trajectory(time, Xd_L, dXd_L);
    // forcing the initial orientation during the trajectory
    Xd_L.M = world_InitialLhand.M;   
    left_arm_task->setReference(KDLtoYarp_position(Xd_L));
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_turning_right(double angle)
{        
    double time_f = 15.0;
    YarptoKDL(left_arm_task->getActualPose(), world_InitialLhand);
    
    KDL::Frame world_SteeringWheel;
    
    world_SteeringWheel.p.data[0] = world_InitialLhand.p.data[0] + 0.05;
    world_SteeringWheel.p.data[1] = world_InitialLhand.p.data[1] - STEERING_WHEEL_RADIUS;
    world_SteeringWheel.p.data[2] = world_InitialLhand.p.data[2];
    world_SteeringWheel.M = KDL::Rotation::Identity();
    
    left_arm_generator.circle_initialize(time_f, STEERING_WHEEL_RADIUS, angle*DEG2RAD, world_InitialLhand, world_SteeringWheel);
    initialized_time=yarp::os::Time::now();
    
    return true;
}

bool walkman::drc::drive::drive_actions::perform_turning_right()
{   
    auto time = yarp::os::Time::now()-initialized_time;
    KDL::Frame Xd_L;
    KDL::Twist dXd_L;
   
    left_arm_generator.circle_trajectory(time, Xd_L, dXd_L);
    // forcing the initial orientation during the trajectory
    Xd_L.M = world_InitialLhand.M;   
    left_arm_task->setReference(KDLtoYarp_position(Xd_L));
    
    return true;
}

bool walkman::drc::drive::drive_actions::init_accelerating()
{        
    return true;
}

bool walkman::drc::drive::drive_actions::perform_accelerating()
{   
    return true;
}

bool walkman::drc::drive::drive_actions::init_decelerating()
{        
    return true;
}

bool walkman::drc::drive::drive_actions::perform_decelerating()
{   
    return true;
}
