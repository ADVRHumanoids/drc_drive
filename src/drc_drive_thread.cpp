#include <yarp/os/all.h>

#include "drc_drive_thread.h"
#include "drc_drive_constants.h"

drc_drive_thread::drc_drive_thread( std::string module_prefix, 
                             			yarp::os::ResourceFinder rf, 
                             			std::shared_ptr< paramHelp::ParamHelperServer > ph) :
    control_thread( module_prefix, rf, ph )
{
    // TODO: skeleton constructor
}

bool drc_drive_thread::custom_init()
{
    // TODO: skeleton function   
    return true;
}

void drc_drive_thread::run()
{   
    // TODO: skeleton function
}    
