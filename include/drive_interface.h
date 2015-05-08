#ifndef DRIVE_INTERFACE_H
#define DRIVE_INTERFACE_H

#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#define WALKMAN_DRC_DRIVE_COMMAND_NONE ""
#define WALKMAN_DRC_DRIVE_COMMAND_IDLE "idle"
#define WALKMAN_DRC_DRIVE_COMMAND_READY "ready"

#define WALKMAN_DRC_DRIVE_COMMAND_STEERING_WHEEL_DATA "steeringwheeldatasent"
#define WALKMAN_DRC_DRIVE_COMMAND_ALIGN_HAND "align_hand"
#define WALKMAN_DRC_DRIVE_COMMAND_TURN_LEFT "turn_left"
#define WALKMAN_DRC_DRIVE_COMMAND_TURN_RIGHT "turn_right"
#define WALKMAN_DRC_DRIVE_COMMAND_ACCELERATE "accelerate"

#define WALKMAN_DRC_DRIVE_COMMAND_GRASP "grasp"
#define WALKMAN_DRC_DRIVE_COMMAND_UNGRASP "ungrasp"

#define WALKMAN_DRC_DRIVE_COMMAND_ACTION_DONE "action_done"
#define WALKMAN_DRC_DRIVE_COMMAND_HAND_DONE "hand_done"

#define WALKMAN_DRC_DRIVE_COMMAND_LEFT_HAND "left_hand"
#define WALKMAN_DRC_DRIVE_COMMAND_RIGHT_HAND "right_hand"
#define WALKMAN_DRC_DRIVE_COMMAND_BOTH_HANDS "both_hands"
#define WALKMAN_DRC_DRIVE_COMMAND_CLOSING_HANDS "close_hands"
#define WALKMAN_DRC_DRIVE_COMMAND_OPENING_HANDS "open_hands"

#define WALKMAN_DRC_DRIVE_STATUS_TURN_LEFT "turning_left"
#define WALKMAN_DRC_DRIVE_STATUS_TURN_RIGHT "turning_right"
#define WALKMAN_DRC_DRIVE_STATUS_ACCELERATE "accelerating"
#define WALKMAN_DRC_DRIVE_STATUS_GRASPING "grasping"
#define WALKMAN_DRC_DRIVE_STATUS_GRASPED "grasped"

#define RAD2DEG    (180.0/M_PI)
#define DEG2RAD    (M_PI/180.0)

#define MIN_CLOSURE 60.0 * DEG2RAD
#define MAX_CLOSURE 700.0 * DEG2RAD

namespace walkman
{
    namespace drc
    {
	namespace drive
	{
	    struct robot_state_input
	    {
		yarp::sig::Vector q;
		yarp::sig::Vector q_dot;
		yarp::sig::Vector tau;
	    };

	    struct robot_state_output
	    {
		yarp::sig::Vector q;
		yarp::sig::Vector q_dot;
		yarp::sig::Vector tau;
	    };
	    
	    enum class state {
                idle,
                ready,
		aligning_hand,
		turning_left,
		turning_right,
		accelerating,
		grasping,
		ungrasping
	    };
	}
    }
}


#endif
