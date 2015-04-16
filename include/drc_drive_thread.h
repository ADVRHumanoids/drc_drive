#ifndef drc_drive_THREAD_H_
#define drc_drive_THREAD_H_

#include <GYM/control_thread.hpp>

/**
 * @brief drc_drive control thread
 * 
 **/
class drc_drive_thread : public control_thread
{
private:   
    
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
    
};

#endif
