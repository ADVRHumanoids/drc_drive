#include <yarp/os/all.h>
#include <GYM/control_module.hpp>
#include <cstdlib>

#include "drc_drive_module.hpp"

// default module period
#define MODULE_PERIOD 1000 //[millisec]

int main(int argc, char* argv[])
{
    // yarp network declaration and check
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
        std::cerr <<"yarpserver not running - run yarpserver"<< std::endl;
        exit(EXIT_FAILURE);
    }
    // yarp network initialization
    yarp.init();

    // create rf
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    // set drc_drive_initial_config.ini as default
    // to specify another config file, run with this arg: --from your_config_file.ini 
    rf.setDefaultConfigFile( "bigman_drc_drive.ini" ); 
    rf.setDefaultContext( "drc_drive" );  
    rf.configure(argc, argv);
    // create my module
    drc_drive_module drc_drive_mod = drc_drive_module( argc, argv, "drc_drive", MODULE_PERIOD, rf );
    
    // run the module
    drc_drive_mod.runModule( rf );
    
    // yarp network deinitialization
    yarp.fini();
    
    exit(EXIT_SUCCESS);
}
