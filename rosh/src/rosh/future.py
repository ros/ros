#TODO: disabling until rospy can talk to multiple masters correctly
def __set_master(name='localhost'):
    """
    Switch the ROS master currently in use, e.g. master('prh'). If
    master() is called with no args, defaults to localhost.
    
    @param name: name of master (if running on default port), or full URI
    @type  name: str
    """
    
    # TODO: rospy needs to have built-in multimaster support for this
    # to actually work, or we need to get rid of the node singleton
    
    if name.startswith('http://'):
        ctx.master._reinit(name)
    else:
        # assume its a hostname
        ctx.master._reinit('http://%s:11311'%name)
        
    # update the system-wide environment 
    os.environ[roslib.rosenv.ROS_MASTER_URI] = ctx.master.master_uri
    return ctx.master.is_online()
    
