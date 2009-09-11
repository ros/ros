## Convience methods for manipulating XML-RPC APIs

import socket
import xmlrpclib

import roslib.network
import roslib.scriptutil

## @return [str]: list of roslaunch XML-RPC URIs for roscore that's in
## the current environment, or None if roscore cannot be contacted.
def get_roslaunch_uris():
    try:
        param_server = roslib.scriptutil.get_param_server()
        code, msg, vals = param_server.getParam('/roslaunch-netapi', '/roslaunch/uris')
        if code == 1 and vals:
            return vals.values()
    except socket.error: pass
    return None

class NetProcess(object):
    def __init__(self, name, respawn_count, is_alive, roslaunch_uri):
        self.is_alive = is_alive
        self.respawn_count = respawn_count
        self.name = name
        
        self.roslaunch_uri = roslaunch_uri
        self.machine, _ = roslib.network.parse_http_host_and_port(roslaunch_uri)

## @param roslaunch_uris [str]: (optional) list of XML-RPCS. If none
## are provided, will look up URIs dynamically
## @return [NetProcess]
def list_processes(roslaunch_uris=None):
    if not roslaunch_uris:
        roslaunch_uris = get_roslaunch_uris()
        if not roslaunch_uris:
            return []

    procs = []
    for uri in roslaunch_uris:
        try:
            r = xmlrpclib.ServerProxy(uri)
            code, msg, val = r.list_processes()
            if code == 1:
                active, dead = val
                procs.extend([NetProcess(a[0], a[1], True, uri) for a in active])
                procs.extend([NetProcess(d[0], d[1], False, uri) for d in dead])
        except:
            #import traceback
            #traceback.print_exc()
            # don't have a mechanism for reporting these errors upwards yet
            pass 
    return procs

if __name__ == "__main__":
    import roslib; roslib.load_manifest('roslaunch')
    print list_processes()
    print "done"
    
