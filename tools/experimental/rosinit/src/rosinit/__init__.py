#!/usr/bin/env python

import os
import socket
import sys
import xmlrpclib
    
import roslib.network
import rosnode

from roslib.scriptutil import get_param_server, get_master


class ROSInitException(Exception): pass

#-------------------------------------------------------------------------------
# utilities

import subprocess
## streamlines calls to subprocess Popen to get stdout
def _cmd_output(cmd):
    return subprocess.Popen(cmd, stdout=subprocess.PIPE).communicate()[0]
    
def _create_save_dir(profile_name):
    save_dir = os.path.abspath(profile_name)
    if not os.path.isdir(save_dir):
        try:
            os.makedirs(save_dir)
        except OSError:
            raise ROSInitException("Cannot create directory [%s] to save profile data in"%save_dir)
    return save_dir


#-------------------------------------------------------------------------------
# rosinit save routines

## Get list of parameter names on param server
## @return [str]: list of parameter names
def _rosinit_get_params():
    p = get_param_server()
    try:
        code, msg, params = p.getParamNames('/rosinit')
        if code != 1:
            raise ROSInitException("Cannot read parameter list from parameter server: parameter server returned [%s]"%msg) 
    except: #most likely network issue
        raise ROSInitException("Cannot read parameter list from parameter server")
    return params
    
## Get the host name and process line of all ROS nodes
## @return {str : (callerid, process line) } 
def _rosinit_get_nodes():
    master = get_master()
    nodes = rosnode.get_node_names()
    
    # this is a bit low... 
    socket.setdefaulttimeout(1)

    host_pids = {}
    # reverse map of pids to node names
    # important: key is the string pid, not int
    pid_to_node = {}
    
    for n in nodes:
        print "getting info for node", n
        uri = rosnode.get_api_uri(master, n)
        node = xmlrpclib.ServerProxy(uri)        
        try:
            code, msg, pid = node.getPid('/rosinit')
            pid_to_node[str(pid)] = n
            if code == 1:
                host, _ = roslib.network.parse_http_host_and_port(uri)

                if host in host_pids:
                    host_pids[host].append(str(pid))
                else:
                    host_pids[host] = [str(pid)]
        except:
            #TODOXXX:REMOVE
            import traceback
            traceback.print_exc()
            
            # node is dead
            pass

    nodes = {}
    for host in host_pids:
        nodes[host] = []

    # this is a temporary hack until roslaunch can report this
    # information for us.  it has many problems, including assuming
    # ssh port 22, assuming UNIX, and assuming that it can ssh without
    # login
    for host, pids in host_pids.iteritems():
        #print "PIDS", host, pids
        pidlist = ','.join(pids)
        ps_cmd = ['ssh', host, 'ps', 'u', '-p', pidlist]
        output = _cmd_output(ps_cmd)
        if not output.strip():
            print >> sys.stderr, "ERROR: cannot get process ID info for machine [%s]"%host
            continue

        # process ps output
        #[USER,PID,%CPU,%MEM,VSZ,RSS,TTY,STAT,START,TIME,COMMAND]
        # we want PID + COMMAND
        
        # throw away header
        processes = [l for l in output.split('\n')[1:] if l]
        for l in processes:
            splits = l.split()
            if len(splits) < 11:
                print >> sys.stderr, "ERROR: ps output from [%s] does not match expected: \n[%s]"%(host, output)
            else:
                pid = splits[1]
                if not pid in pids:
                    print >> sys.stderr, "ERROR: ps output from [%s] does not match expected: parsed [%s] as a PID, but that is not an expected PID"%(host, pid)
                else:
                    command = ' '.join(splits[10:])
                    nodes[host].append((pid_to_node[pid], command))
                    
    # { hostname : node process list }            
    return nodes
         
def _rosinit_save_params_file(params, save_dir):
    params_file_path = os.path.join(save_dir, 'params.txt')
    params_file = open(params_file_path, 'w')
    try:
        print "writing params list [%s]"%params_file_path
        params_file.write('\n'.join(params))
    finally:
        params_file.close()

def _rosinit_save_nodes_file(nodes_by_host, save_dir):
    nodes_file_path = os.path.join(save_dir, 'nodes.txt')
    nodes_file = open(nodes_file_path, 'w')
    try:
        print "writing nodes list [%s]"%nodes_file_path
        for host, proc_list in nodes_by_host.iteritems():
            nodes_file.write('\n'.join(["%s %s %s"%(host, callerid, p) for (callerid, p) in proc_list])+'\n')
    finally:
        nodes_file.close()

def rosinit_save(profile_name):
    save_dir = _create_save_dir(profile_name)
    params = _rosinit_get_params()
    nodes_by_host = _rosinit_get_nodes()

    _rosinit_save_params_file(params, save_dir)
    _rosinit_save_nodes_file(nodes_by_host, save_dir)
    print "done saving to [%s]"%save_dir
    

#-------------------------------------------------------------------------------
# rosinit load routines

## @return True if \a param is same as \a param_ns or is in \a param_ns's namespace    
def param_ns_equals(param, param_ns):
    if param == param_ns:
        return True
    if not param_ns.endswith('/'):
        param_ns = param_ns + '/'
    return param.startswith(param_ns)
        
def _rosinit_init_params(params):
    param_server = get_param_server()
    code, msg, curr_params = param_server.getParamNames('/rosinit')
    if code != 1:
        raise ROSInitException("param server refused to return param list: %s"%msg)
    to_delete = []
    for param_key in curr_params:
        if not [p for p in params if param_ns_equals(param_key, p)]:
            to_delete.append(param_key)
            
    for param_key in to_delete:
        code, msg, val = param_server.hasParam('/rosinit', param_key)
        if not val:
            print >> sys.stderr, "lost sync with param server"
            continue
        code, msg, _ = param_server.deleteParam('/rosinit', param_key)
        if code != 1:
            raise ROSInitException("param server refused to delete [%s]: %s"%(param_key, msg))
        print "deleted %s"%param_key

def _rosinit_init_master(nodes_by_host):
    whitelist = []
    for host, l in nodes_by_host.iteritems():
        whitelist.extend([callerid for callerid, _ in l])
        
    print "KEEP LIST", whitelist
    rosnode.cleanup_master_whitelist(get_master(), whitelist)
    
## Kill all nodes not whitelisted in \a nodes_by_host
def _rosinit_init_nodes(nodes_by_host):
    #Scott, nodes_by_host is a dictionary where the key is the
    #hostname and the value is (callerid, process line). I hope this
    #is the appropriate input to ckill TODO: ckill stuff here

    # TODO: ckill

    # scrub the master
    _rosinit_init_master(nodes_by_host)

def _rosinit_load_params_file(save_dir):
    params_file_path = os.path.join(save_dir, 'params.txt')
    params_file = open(params_file_path, 'r')
    try:
        print "reading params list [%s]"%params_file_path
        params = [p.strip() for p in params_file.readlines() if p.strip()]
    finally:
        params_file.close()
    return params

def _rosinit_load_nodes_file(save_dir):
    nodes_file_path = os.path.join(save_dir, 'nodes.txt')
    nodes_file = open(nodes_file_path, 'r')
    nodes_by_host = {}
    try:
        print "reading nodes list [%s]"%nodes_file_path
        for l in nodes_file.readlines():
            splits = l.split(' ')
            if len(splits) < 3:
                raise ROSInitException("cannot parse nodes file")
            host = splits[0]
            callerid = splits[1]
            process_line = ' '.join(splits[2:])
            
            #print "LOADED", host, callerid, process_line
            if not host in nodes_by_host:
                nodes_by_host[host] = []
            nodes_by_host[host].append((callerid, process_line))
    finally:
        nodes_file.close()
    return nodes_by_host

def rosinit_init(profile_name):
    save_dir = _create_save_dir(profile_name)    
    params = _rosinit_load_params_file(save_dir)
    nodes_by_host = _rosinit_load_nodes_file(save_dir)

    #print "PARAMS", params
    _rosinit_init_params(params)

    print "NODES", nodes_by_host
    _rosinit_init_nodes(nodes_by_host)

    
#-------------------------------------------------------------------------------
# command line

def fullusage():
    print """Commands:
\trosinit save\tsave current profile info
\trosnode init\tload profile

Type rosinit <command> -h for more detailed usage, e.g. 'rosinit save -h'
"""
    sys.exit(os.EX_USAGE)

def rosinit_main():
    if len(sys.argv) == 1:
        fullusage()
    try:
        # TODO: proper arg parsing
        if len(sys.argv) != 3:
            fullusage()            
        command = sys.argv[1]
        if command == 'save':
            rosinit_save(sys.argv[2])
        elif command == 'init':
            rosinit_init(sys.argv[2])
        else:
            fullusage()
    except ROSInitException, e:
        print >> sys.stderr, "ERROR: "+str(e)
    except KeyboardInterrupt:
        pass


        
