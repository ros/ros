/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

// author: Rosen Diankov

#include "oct.h"
#include "ov-usr-fcn.h"
#include "ov-fcn-handle.h"
#include "mxarray.h"
#include "mexproto.h"
#include "oct-rl-edit.h"
#include "parse.h"

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>

#include <cstdio>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/node.h>
#include <ros/service.h>
#include <ros/session.h>

#include <signal.h>

//#define SERVICE_SERIALIZE_SEQID
//#define COMPILE_FOR_MATLAB

#ifdef MSVC
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

using namespace ros;
using namespace std;

class RoscppWorker
{
public:
    virtual ~RoscppWorker() {}
    virtual void workerthread(void* userdata) = 0;
};

// executes a worker
class RoscppWorkExecutor
{
public:
    RoscppWorkExecutor(RoscppWorker* pworker, void* userdata) : _pworker(pworker), _userdata(userdata) {}
    virtual ~RoscppWorkExecutor() { _pworker->workerthread(_userdata); }
private:
    RoscppWorker* _pworker;
    void* _userdata;
};

void reset_all();
void AddWorker(RoscppWorker* psub, void* userdata);
void __rosoct_worker(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]);

ros::Node* check_roscpp_nocreate()
{
    ros::Node* pnode = ros::Node::instance();
    return (pnode && pnode->checkMaster()) ? pnode : NULL;
}

string GetString(const mxArray* parr)
{
    string cmd;
    cmd.resize(mxGetNumberOfElements(parr)+1);
    mxGetString(parr,&cmd[0],cmd.size()); cmd.resize(cmd.size()-1);
    return cmd;
}

// returns the size written
int GetData(const mxArray* pmxdata, vector<uint8_t>& vdata)
{
    ROS_ASSERT(pmxdata!=NULL);
    int writesize = mxGetElementSize(pmxdata)*mxGetNumberOfElements(pmxdata);
    vdata.resize(writesize);
    if( writesize > 0 )
        memcpy(&vdata[0], mxGetPr(pmxdata), writesize);
    return writesize;
}

// it is necessary to call the octave serialize function directly since a sequence id is needed
class OctaveMsgSerializer : public Message
{
public:
    OctaveMsgSerializer() : _serlen(0), _pmxfunction(NULL) {}
    OctaveMsgSerializer(const OctaveMsgSerializer& r) : Message()
    {
        _md5sum = r._md5sum;
        _type = r._type;
        _serlen = r._serlen;
        _pmxfunction = NULL;
        SetSerializationFn(r._pmxfunction);
    }
    OctaveMsgSerializer(const string& md5sum, const mxArray* pmxfunction, int serlen, const string& type)
    {
        _serlen = serlen;
        _type = type;
        _md5sum = md5sum;
        _pmxfunction = NULL;
        SetSerializationFn(pmxfunction);
    }
    virtual ~OctaveMsgSerializer() {
        if( _pmxfunction != NULL ) {
            mxDestroyArray(_pmxfunction); _pmxfunction = NULL;
        }
    }

    string _md5sum, _type;
    int _serlen;
    mxArray* _pmxfunction;
    //octave_value ovfn;

    virtual void SetSerializationFn(const mxArray* pfn)
    {
        if( _pmxfunction != NULL ) {
            mxDestroyArray(_pmxfunction);
            _pmxfunction = NULL;
        }

        if( pfn != NULL ) {
            ROS_ASSERT(mxGetClassID(pfn) == mxFUNCTION_CLASS);
            _pmxfunction = mxDuplicateArray(pfn);
            mexMakeArrayPersistent(_pmxfunction);

            mxDestroyArray(_pmxfunction);
            _pmxfunction = mxDuplicateArray(pfn);
            mexMakeArrayPersistent(_pmxfunction);
        }

//        if( _pfn != NULL ) {
//            mxArray* pfn = mxDuplicateArray(_pfn);
//            mexMakeArrayPersistent(pfn);
//            ROS_ASSERT(mxGetClassID(pfn) == mxFUNCTION_CLASS);
//            ROS_ASSERT(pfn->is_octave_value());
//            ovfn = mxArray::as_octave_value((mxArray*)pfn);
//            ROS_ASSERT(ovfn.user_function_value() != NULL);
//            ROS_INFO("name: %s, user: %s\n", ovfn.fcn_handle_value()->fcn_name().c_str(), ovfn.user_function_value()->fcn_file_name().c_str());
//        }
    }

    virtual const string __getDataType() const { return _type; }
    virtual const string __getMD5Sum()   const { return _md5sum; }
    virtual const string __getServerMD5Sum() const { return _md5sum; }

    uint32_t serializationLength() const { return _serlen; }
    virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
    {
//        ROS_ASSERT(ovfn.user_function_value() != NULL);
//        octave_value_list args;
//        args.resize(1); args(0) = uint32NDArray(seqid);
//        octave_value_list retval = ovfn.user_function_value()->do_multi_index_op(1, args);
//
//        ROS_INFO("len: %d\n", retval.length());
//        if( retval.length() < 1 )
//            mexErrMsgTxt("failed to call matlab function");
//
//        int writesize = retval(0).byte_size();
//        ROS_ASSERT(retval(0).mex_get_data() != NULL );
//        memcpy(writePtr, retval(0).mex_get_data(), writesize);


        ROS_ASSERT(_pmxfunction != NULL );
        mxArray* pmxdata;
        mxArray* pmxseqid = mxCreateNumericMatrix(1,1,mxUINT32_CLASS,mxREAL);
        *(uint32_t*)mxGetPr(pmxseqid) = seqid;
        mxArray* prhs[] = {_pmxfunction, pmxseqid };
        int ret = mexCallMATLAB(1, &pmxdata, 2, prhs, "feval");
        if( ret != 0 )
            mexErrMsgTxt("failed to call matlab function");

        if( pmxdata == NULL )
            mexErrMsgTxt("serialize output not filled!");

        int writesize = mxGetElementSize(pmxdata)*mxGetNumberOfElements(pmxdata);
        ROS_ASSERT(_serlen == writesize );

        memcpy(writePtr, mxGetPr(pmxdata), writesize);
        return writePtr+writesize;
    }
    virtual uint8_t *deserialize(uint8_t *readPtr) { ROS_ASSERT(0); return NULL; }
};

class OctaveSessionSerializer : public OctaveMsgSerializer
{
public:
    OctaveSessionSerializer(const OctaveSessionSerializer& r) : OctaveMsgSerializer(r) {
        sessionid = r.sessionid;
        sessionoffset = r.sessionoffset;
    }
    OctaveSessionSerializer(const string& md5sum, const mxArray* pmxfunction, int serlen, const string& type)
        : OctaveMsgSerializer(md5sum, pmxfunction,serlen,type), sessionid(0), sessionoffset(0) {}

    virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
    {
        uint8_t* p = OctaveMsgSerializer::serialize(writePtr, seqid);
        *(int*)(writePtr+sessionoffset) = sessionid;
        return p;
    }

    int sessionid;
    int sessionoffset;
};

class OctaveMsgDeserializer : public Message
{
public:
    OctaveMsgDeserializer() {}
    OctaveMsgDeserializer(const OctaveMsgDeserializer& r) : Message()
    {
        _type = r._type;
        _md5sum = r._md5sum;
        _vdata = r._vdata;
        __connection_header = r.__connection_header;
    }
    OctaveMsgDeserializer(const string& md5sum, const string& type = string("*"))
    {
        _type = type;
        _md5sum = md5sum;
    }

    string _md5sum, _type;
    vector<uint8_t> _vdata;
    boost::shared_ptr<ros::M_string> __connection_header;

    virtual const string __getDataType() const { return _type; }
    virtual const string __getMD5Sum()   const { return _md5sum; }
    virtual const string __getServerMD5Sum() const { return _md5sum; }

    uint32_t serializationLength() const { return _vdata.size(); }
    virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
    {
        if( seqid != 0 )
            // if you get this message, define SERVICE_SERIALIZE_SEQID and change rosoct_service_call.m and rosoct_session_call.m
            ROS_DEBUG("ignoring service sequence id %d!", seqid);

        if( _vdata.size() > 0 )
            memcpy(writePtr, &_vdata[0], _vdata.size());
        return writePtr + _vdata.size();
    }
    virtual uint8_t *deserialize(uint8_t *readPtr)
    {
        _vdata.resize(__serialized_length);
        if( _vdata.size() > 0 )
            memcpy(&_vdata[0], readPtr, _vdata.size());
        return readPtr+__serialized_length;
    }
};

class OctaveSessionDeserializer : public OctaveMsgDeserializer
{
public:
    OctaveSessionDeserializer() : OctaveMsgDeserializer(), sessionid(0), sessionoffset(0) {}
    OctaveSessionDeserializer(const OctaveSessionDeserializer& r) : OctaveMsgDeserializer(r) {
        sessionid = r.sessionid;
        sessionoffset = r.sessionoffset;
    }
    OctaveSessionDeserializer(const string& md5sum, const string& type) : OctaveMsgDeserializer(md5sum,type) {}

    virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const
    {
        uint8_t* p = OctaveMsgDeserializer::serialize(writePtr,seqid);
        *(int*)(writePtr+sessionoffset) = sessionid;
        return p;
    }
    virtual uint8_t *deserialize(uint8_t *readPtr)
    {
        sessionid = *(int*)(readPtr+sessionoffset);
        return OctaveMsgDeserializer::deserialize(readPtr);
    }

    int sessionid;
    int sessionoffset;
};

class RoscppSubscription : public RoscppWorker
{
public:
    RoscppSubscription(Node* pnode, const string& topicname, const string& md5sum, const string& type, const mxArray* pfn, int maxqueue = 1)
    {
        _bDropWork = false;
        _msg._md5sum = md5sum;
        _msg._type = type;
        _topicname = topicname;
        assert( pnode != NULL && pfn != NULL );

        if(mxGetClassID(pfn) != mxFUNCTION_CLASS) {
            ROS_ERROR("not a function class");
            throw;
        }

        ROS_ASSERT(pfn->is_octave_value());
        ovfncallback = mxArray::as_octave_value((mxArray*)pfn);
        ROS_ASSERT(ovfncallback.user_function_value() != NULL);

        if( !pnode->subscribe(_topicname, _msg, &RoscppSubscription::cb, this, maxqueue) )
            throw;
    }
    virtual ~RoscppSubscription()
    {
        ros::Node* pnode = check_roscpp_nocreate();
        if( pnode != NULL ) {
            if( !pnode->unsubscribe(_topicname) )
                ROS_WARN("failed to unsubscribe from %s", _topicname.c_str());

            __rosoct_worker(0,NULL,0,NULL); // flush
        }
    }

    virtual void workerthread(void* userdata)
    {
        boost::shared_ptr<OctaveMsgDeserializer> pmsg((OctaveMsgDeserializer*)userdata);
        boost::mutex::scoped_lock lock(_mutex);
        dowork(pmsg.get());
    }

private:
    void cb()
    {
        if( _bDropWork )
            return;
        boost::mutex::scoped_lock lock(_mutex);
        AddWorker(this, new OctaveMsgDeserializer(_msg));
    }

    virtual void dowork(OctaveMsgDeserializer* pmsg)
    {
        ROS_ASSERT(ovfncallback.user_function_value() != NULL);
        octave_value_list args;
        args.resize(1);
        uint8NDArray odata;
        if( pmsg->_vdata.size() > 0 ) {
            odata.resize_no_fill(pmsg->_vdata.size());
            memcpy(odata.fortran_vec(),&pmsg->_vdata[0],pmsg->_vdata.size());
        }
        args(0) = odata;
        octave_value_list retval = ovfncallback.user_function_value()->do_multi_index_op(0, args);
    }

    string _topicname;
    OctaveMsgDeserializer _msg;
    octave_value ovfncallback;
    boost::mutex _mutex;
    bool _bDropWork;
};

class RoscppService : public RoscppWorker
{
public:
    RoscppService(Node* pnode, const string& servicename, const string& md5sum, const string& reqtype, const string& restype, const mxArray* pservicefn)
    {
        _bDropWork = false;
        preq = NULL;
        pres = NULL;
        _servicename = servicename;
        assert( pnode != NULL && pservicefn != NULL);

        if(mxGetClassID(pservicefn) != mxFUNCTION_CLASS ) {
            ROS_ERROR("not a function class");
            throw;
        }

        ROS_ASSERT(pservicefn->is_octave_value());
        ovfnservice = mxArray::as_octave_value((mxArray*)pservicefn);
        ROS_ASSERT(ovfnservice.user_function_value() != NULL);

        OctaveMsgDeserializer localreq(md5sum, reqtype);
        OctaveMsgDeserializer localres(md5sum, restype);
        if( !pnode->advertiseService(_servicename, &RoscppService::cb, this, localreq, localres, -1) )
            throw;

        ROS_INFO("advertised service %s", _servicename.c_str());
    }
    virtual ~RoscppService()
    {
        ros::Node* pnode = check_roscpp_nocreate();
        if( pnode != NULL ) {
            ROS_DEBUG("unadvertising %s", _servicename.c_str());
            pnode->unadvertiseService(_servicename);

            __rosoct_worker(0,NULL,0,NULL); // flush
        }
    }

    virtual void workerthread(void* userdata)
    {
        boost::mutex::scoped_lock lock(_mutex);
        dowork();
        _control.notify_all();
    }

private:

    // this setup allows only one service to be processed at a time
    bool cb(OctaveMsgDeserializer& req, OctaveMsgDeserializer& res)
    {
        if( _bDropWork )
            return false;

        boost::mutex::scoped_lock lockserv(_mutexService); // lock simultaneous service calls out
        boost::mutex::scoped_lock lock(_mutex);
        preq = &req;
        pres = &res;
        AddWorker(this, NULL);
        _control.wait(lock);

        // done so fill res
        preq = NULL;
        pres = NULL;
        res._md5sum = _md5sum;
        res._type = _restype;
        return _bSuccess;
    }

    virtual void dowork()
    {
        ROS_ASSERT(ovfnservice.user_function_value() != NULL);

        octave_value_list args;
        args.resize(3);
        uint8NDArray odata;
        if( preq->_vdata.size() > 0 ) {
            odata.resize_no_fill((int)preq->_vdata.size());
            ROS_ASSERT(odata.length()==(int)preq->_vdata.size());
            memcpy(odata.fortran_vec(),&preq->_vdata[0],preq->_vdata.size());
        }
        args(0) = odata;
        octave_value_list retval = ovfnservice.user_function_value()->do_multi_index_op(2, args);

        if( retval.length() < 2 ) {
            ROS_ERROR("service function failed");
            _bSuccess = false;
            return;
        }

        _bSuccess = retval(0).scalar_value()>0;

        if( _bSuccess ) {
            charNDArray array = retval(1).char_array_value();
            pres->_vdata.resize(array.byte_size());
            if( pres->_vdata.size() > 0 )
                memcpy(&pres->_vdata[0],array.fortran_vec(),pres->_vdata.size());
        }
    }

    bool _bSuccess; // success of the service call

    string _servicename;
    string _md5sum, _restype;
    OctaveMsgDeserializer* preq;
    OctaveMsgDeserializer* pres;
    octave_value ovfnservice;
    boost::condition _control;
    boost::mutex _mutex, _mutexService;
    bool _bDropWork;
};

typedef void (*FunctionPtr)(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[]);

class RosoctStaticData
{
public:
    RosoctStaticData() : nSessionHandleId(1) {}
    ~RosoctStaticData() {
        reset_all();
        mapFunctions.clear();
    }

    list<boost::shared_ptr<RoscppWorkExecutor> > listWorkerItems;
    map<string, pair<string, string> > mapAdvertised; ///< advertised topics

    map<string, boost::shared_ptr<RoscppSubscription> > subscriptions;
    map<string, boost::shared_ptr<RoscppService> > services;
    map<int, session::abstractSessionHandle> sessions;
    int nSessionHandleId; ///< counter of unique session ids to assign
    boost::mutex mutexWorker, mutexWorking;
    map<string,FunctionPtr> mapFunctions;
    vector<char*> argv;
    vector<string> vargv;
};

static RosoctStaticData s_staticdata;

#define s_listWorkerItems s_staticdata.listWorkerItems
#define s_mapAdvertised s_staticdata.mapAdvertised
#define s_subscriptions s_staticdata.subscriptions
#define s_services s_staticdata.services
#define s_sessions s_staticdata.sessions
#define s_nSessionHandleId s_staticdata.nSessionHandleId
#define s_mutexWorker s_staticdata.mutexWorker
#define s_mutexWorking s_staticdata.mutexWorking
#define s_mapFunctions s_staticdata.mapFunctions
#define s_vargv s_staticdata.vargv

void AddWorker(RoscppWorker* psub, void* userdata)
{
    boost::mutex::scoped_lock lock(s_mutexWorker);
    s_listWorkerItems.push_back(boost::shared_ptr<RoscppWorkExecutor>(new RoscppWorkExecutor(psub, userdata)));
}

void __rosoct_worker(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    list<boost::shared_ptr<RoscppWorkExecutor> > listworkers;

    int nItemsToProcess = -1;
    if( nrhs > 0 )
        nItemsToProcess = (int)mxGetScalar(prhs[0]);

    {
        boost::mutex::scoped_lock lock(s_mutexWorker);
        if( nItemsToProcess < 0 )
            listworkers.swap(s_listWorkerItems);
        else {
            nItemsToProcess = min(nItemsToProcess,(int)s_listWorkerItems.size());
            // only take nItemsToProcess from s_listWorkerItems
            list<boost::shared_ptr<RoscppWorkExecutor> >::iterator itlast = s_listWorkerItems.begin();
            advance(itlast,nItemsToProcess);
            listworkers.splice(listworkers.end(),s_listWorkerItems, s_listWorkerItems.begin(),itlast);
        }
    }

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)listworkers.size());

    boost::mutex::scoped_lock lock(s_mutexWorking);
    listworkers.clear(); // do all the work in the destructors
}

ros::Node* check_roscpp()
{
    // start roscpp
    ros::Node* pnode = ros::Node::instance();

    if( pnode && !pnode->checkMaster() ) {
        reset_all();

        delete pnode;
        return NULL;
    }

    if (!pnode) {
        char strname[256] = "nohost";
        gethostname(strname, sizeof(strname));
        strcat(strname,"_rosoct");

        int argc = (int)s_vargv.size();
        vector<string> vargv = s_vargv;
        vector<char*> argv(vargv.size());
        for(size_t i = 0; i < argv.size(); ++i)
            argv[i] = &vargv[i][0];
        ros::init(argc,argv.size() > 0 ? &argv[0] : NULL);

        pnode = new ros::Node(strname, ros::Node::DONT_HANDLE_SIGINT|ros::Node::ANONYMOUS_NAME|ros::Node::DONT_ADD_ROSOUT_APPENDER);

        bool bCheckMaster = pnode->checkMaster();

        delete pnode;

        if( !bCheckMaster ) {
            ROS_INFO("ros not present");
            return NULL;
        }

        argc = (int)s_vargv.size();
        vargv = s_vargv;
        for(size_t i = 0; i < argv.size(); ++i)
            argv[i] = &vargv[i][0];
        ros::init(argc,argv.size() > 0 ? &argv[0] : NULL);
        pnode = new ros::Node(strname, ros::Node::DONT_HANDLE_SIGINT|ros::Node::ANONYMOUS_NAME);
        ROS_INFO("new roscpp node started");
    }

    return pnode;
}

void __rosoct_service_call(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

#ifdef SERVICE_SERIALIZE_SEQID
    if( nrhs < 6 )
        mexErrMsgTxt("not enough args to __rosoct_service_call");

    OctaveMsgSerializer servreq(GetString(prhs[1]), prhs[4], (int)mxGetScalar(prhs[5]), GetString(prhs[2]));
#else
    if( nrhs < 5 )
        mexErrMsgTxt("not enough args to __rosoct_service_call");

    OctaveMsgDeserializer servreq(GetString(prhs[1]), GetString(prhs[2]));
    GetData(prhs[4],servreq._vdata);
#endif
    OctaveMsgDeserializer servres(GetString(prhs[1]), GetString(prhs[3]));
    bool bPersistent = nrhs > 5 && (int)mxGetScalar(prhs[5]);

    string servname = GetString(prhs[0]);
    bool bSuccess = ros::service::call(servname, servreq, servres);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
    if( nlhs > 1 ) {
        plhs[1] = mxCreateNumericMatrix(servres._vdata.size(),1,mxUINT8_CLASS,mxREAL);
        if( servres._vdata.size() > 0 )
            memcpy(mxGetPr(plhs[1]),&servres._vdata[0],servres._vdata.size());
    }
    if( nlhs > 2 && bPersistent ) {
        // get back handle to persistent service
    }
}

void __rosoct_msg_subscribe(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 5 )
        mexErrMsgTxt("not enough args to __rosoct_msg_subscribe");

    string topicname = GetString(prhs[0]);
    int success = 1;
    try {
        boost::shared_ptr<RoscppSubscription> subs(new RoscppSubscription(pnode, topicname, GetString(prhs[1]), GetString(prhs[2]), prhs[3], (int)mxGetScalar(prhs[4])));
        s_subscriptions[topicname] = subs;

        ROS_INFO("subscribed to %s", topicname.c_str());
    }
    catch(...) {
        // failed
        ROS_ERROR("failed to subscribe to %s", topicname.c_str());
    }

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)success);
}

void __rosoct_msg_unsubscribe(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_msg_unsubscribe");

    string topicname = GetString(prhs[0]);
    map<string, boost::shared_ptr<RoscppSubscription> >::iterator it = s_subscriptions.find(topicname);

    ROS_DEBUG("unsubscribe %s", topicname.c_str());
    bool bSuccess = s_subscriptions.erase(topicname)>0;

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_set_param(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 2 )
        mexErrMsgTxt("not enough args to __rosoct_set_param");

    string key = GetString(prhs[0]);
    if( mxIsDouble(prhs[1]) )
        pnode->setParam(key, *(double*)mxGetPr(prhs[1]));
    else if( mxIsInt8(prhs[1]) || mxIsInt16(prhs[1]) || mxIsInt32(prhs[1]) )
        pnode->setParam(key, *(int*)mxGetPr(prhs[1]));
    else if( mxIsChar(prhs[1]) )
        pnode->setParam(key, GetString(prhs[1]));
    else
        mexErrMsgTxt("unsupported key type!");

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar(1.0);
}

void __rosoct_get_param(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_get_param");

    string key = GetString(prhs[0]);
    string s;
    double d;
    int i;

    if( nlhs > 0 ) {
        if( pnode->getParam(key,s) )
            plhs[0] = mxCreateString(s.c_str());
        else if( pnode->getParam(key,d) )
            plhs[0] = mxCreateDoubleScalar(d);
        else if( pnode->getParam(key,i) ) {
            plhs[0] = mxCreateNumericMatrix(1,1,mxINT32_CLASS,mxREAL);
            *(int*)mxGetPr(plhs[0]) = i;
        }
        else
            // could not find
            plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
    }
}

void __rosoct_time_now(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    if( nlhs > 0 ) {
        ros::Time t = ros::Time::now();
        
        static const char* keys[] = {"sec","nsec"};
        mxArray* pmxtopics = mxCreateStructMatrix(1,1,2,keys);
        mxSetField(pmxtopics, 0, "sec", mxCreateDoubleScalar((double)t.sec));
        mxSetField(pmxtopics, 0, "nsec", mxCreateDoubleScalar((double)t.nsec));
        plhs[0] = pmxtopics;
    }
}

void __rosoct_advertise_service(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 5 )
        mexErrMsgTxt("not enough args to __rosoct_advertise");

    string servicename = GetString(prhs[0]);

    if( s_services.find(servicename) != s_services.end() )
        mexErrMsgTxt("topic already advertised");

    int success = 1;
    try {
        boost::shared_ptr<RoscppService> serv(new RoscppService(pnode, servicename, GetString(prhs[1]), GetString(prhs[2]), GetString(prhs[3]), prhs[4]));
        s_services[servicename] = serv;
    }
    catch(...) {
        // failed
        ROS_ERROR("failed to advertise service to %s", servicename.c_str());
    }

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)success);
}

void __rosoct_unadvertise_service(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_unadvertise_service");

    string servicename = GetString(prhs[0]);
    map<string, boost::shared_ptr<RoscppService> >::iterator it = s_services.find(servicename);

    bool bSuccess = pnode->unadvertiseService(servicename);

    if( bSuccess && s_services.find(servicename) == s_services.end() )
        ROS_WARN("have not advertised %s!", servicename.c_str());
    s_services.erase(servicename);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_advertise(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 4 )
        mexErrMsgTxt("not enough args to __rosoct_advertise");

    string topicname = GetString(prhs[0]);

    if( s_mapAdvertised.find(topicname) != s_mapAdvertised.end() )
        mexErrMsgTxt("topic already advertised");

    string md5sum = GetString(prhs[1]), type = GetString(prhs[2]);
    OctaveMsgSerializer msgcloner(md5sum, NULL, 0, type);
    bool bSuccess = pnode->advertise(topicname, msgcloner, (int)mxGetScalar(prhs[3]));

    if( bSuccess )
        s_mapAdvertised[topicname] = pair<string,string>(md5sum,type);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_unadvertise(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_unadvertise");

    string topicname = GetString(prhs[0]);

    s_mapAdvertised.erase(topicname);
    bool bSuccess = pnode->unadvertise(topicname);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_publish(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 3 )
        mexErrMsgTxt("not enough args to __rosoct_publish");

    bool bSuccess = false;
    string topicname = GetString(prhs[0]);
    map<string,pair<string,string> >::iterator it = s_mapAdvertised.find(topicname);
    if( it != s_mapAdvertised.end() ) {
        OctaveMsgSerializer msgcloner(it->second.first, prhs[1], (uint32_t)mxGetScalar(prhs[2]), it->second.second);
        pnode->publish(topicname, msgcloner);
        bSuccess = true;
    }

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_create_session(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 7 )
        mexErrMsgTxt("not enough args to __rosoct_create_session");

    string servname = GetString(prhs[0]);
    OctaveSessionSerializer servreq(GetString(prhs[1]), prhs[4], (uint32_t)mxGetScalar(prhs[5]), GetString(prhs[2]));
    OctaveSessionDeserializer servres(GetString(prhs[1]), GetString(prhs[3]));

    servreq.sessionoffset = (int)mxGetScalar(prhs[6]);
    servres.sessionoffset = servreq.sessionoffset;

    session::abstractSessionHandle handle = ros::session::create_session(servname, servreq, servres);

    int sessionid = 0;
    if( !!handle ) {
        sessionid = s_nSessionHandleId++;
        if( sessionid == 0 )
            s_nSessionHandleId++;

        ROS_ASSERT(s_sessions.find(sessionid) == s_sessions.end());
        s_sessions[sessionid] = handle;
        ROS_INFO("started session %s with sessionid %d, octave id %d", servname.c_str(), handle->GetSessionId(),sessionid);
    }

    if( nlhs > 0 ) {
        if( !handle )
            mxCreateDoubleMatrix(0,0,mxREAL);
        else
            plhs[0] = mxCreateDoubleScalar((double)sessionid);
    }
    if( nlhs > 1 ) {
        plhs[1] = mxCreateNumericMatrix(servres._vdata.size(),1,mxUINT8_CLASS,mxREAL);
        if( servres._vdata.size() > 0 )
            memcpy(mxGetPr(plhs[1]),&servres._vdata[0],servres._vdata.size());
    }
}

void __rosoct_terminate_session(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_terminate_session");

    int sessionid = (int)mxGetScalar(prhs[0]);

    bool bSuccess = false;
    map<int,session::abstractSessionHandle>::iterator it = s_sessions.find(sessionid);
    if( it != s_sessions.end() ) {
        bSuccess = it->second->terminate();
        s_sessions.erase(it);
    }
    else
        ROS_WARN("no session %d present", sessionid);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_session_call(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

#ifdef SERVICE_SERIALIZE_SEQID
    if( nrhs < 7 )
        mexErrMsgTxt("not enough args to __rosoct_session_call");
#else
    if( nrhs < 6 )
        mexErrMsgTxt("not enough args to __rosoct_session_call");
#endif


    int sessionid = (int)mxGetScalar(prhs[0]);
    bool bSuccess = false;
    map<int,session::abstractSessionHandle>::iterator it = s_sessions.find(sessionid);
    if( it != s_sessions.end() ) {
        string servname = GetString(prhs[1]);

#ifdef SERVICE_SERIALIZE_SEQID
        OctaveMsgSerializer servreq(GetString(prhs[2]), prhs[5], (int)mxGetScalar(prhs[6]), GetString(prhs[3]));
#else
        OctaveMsgDeserializer servreq(GetString(prhs[2]), GetString(prhs[3]));
        GetData(prhs[5],servreq._vdata);
#endif
        OctaveMsgDeserializer servres(GetString(prhs[2]), GetString(prhs[4]));

        bool bAsyncCall = false; // in order to support async, message has to be serialized at this point since it is a function
        bSuccess = it->second->call(servname, servreq, servres, bAsyncCall);

        if( nlhs > 1 ) {
            plhs[1] = mxCreateNumericMatrix(servres._vdata.size(),1,mxUINT8_CLASS,mxREAL);
            if( servres._vdata.size() > 0 )
                memcpy(mxGetPr(plhs[1]),&servres._vdata[0],servres._vdata.size());
        }
    }
    else {
        ROS_WARN("no session %d present, number of sessions present: %"PRIdS, sessionid, s_sessions.size());
        for(it = s_sessions.begin(); it != s_sessions.end(); ++it)
            ROS_WARN("session %s:%d, local id %d",it->second->GetSessionName().c_str(), it->second->GetSessionId(), it->first);
    }

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_wait_for_service(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar(0.0);
        return;
    }

    if( nrhs < 1 )
        mexErrMsgTxt("not enough args to __rosoct_wait_for_service");

    string service = GetString(prhs[0]);
    int32_t timeout = -1;

    if( nrhs > 1 )
        timeout = (int32_t)mxGetScalar(prhs[1]);
    bool bSuccess = service::waitForService(service, timeout);

    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)bSuccess);
}

void __rosoct_check_master(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp_nocreate();
    if( nlhs > 0 )
        plhs[0] = mxCreateDoubleScalar((double)(pnode!=NULL));
}

void __rosoct_get_topics(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    ros::Node* pnode = check_roscpp();
    if( !pnode ) {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleMatrix(0,0,mxREAL);
        if( nlhs > 1 )
            plhs[1] = mxCreateDoubleScalar(0.0);
        return;
    }

    vector<pair<string, string> > vtopicpairs;

    string cmd;
    for(int i = 0; i < nrhs; ++i) {
        cmd.resize(mxGetNumberOfElements(prhs[i])+1);
        mxGetString(prhs[i],&cmd[0],cmd.size()); cmd.resize(cmd.size()-1);
        if( cmd == "advertised" ) {
            vector<string> vtopics;
            pnode->getAdvertisedTopics(vtopics);

            vtopicpairs.clear();
            for(vector<string>::iterator it = vtopics.begin(); it != vtopics.end(); ++it)
                vtopicpairs.push_back(pair<string,string>(*it,string()));
        }
        else if( cmd == "published" )
            pnode->getPublishedTopics(&vtopicpairs);
    }

    if( nlhs > 0 ) {
        static const char* keys[] = {"topic","type"};
        mxArray* pmxtopics = mxCreateStructMatrix(vtopicpairs.size(),1,2,keys);
        int index = 0;
        for(vector<pair<string, string> >::iterator it = vtopicpairs.begin(); it != vtopicpairs.end(); ++it, ++index) {
            mxSetField(pmxtopics, index, "topic", mxCreateString(it->first.c_str()));
            mxSetField(pmxtopics, index, "type", mxCreateString(it->second.c_str()));
        }

        plhs[0] = pmxtopics;
    }

    if( nlhs > 1 )
        plhs[1] = mxCreateDoubleScalar(1.0);
}

void reset_all()
{
    __rosoct_worker(0,NULL,0,NULL); // flush all
    s_subscriptions.clear();
    s_services.clear();
    s_sessions.clear();

    ros::Node* pnode = check_roscpp_nocreate();
    if( pnode != NULL ) {
        for(map<string, pair<string,string> >::iterator it = s_mapAdvertised.begin(); it != s_mapAdvertised.end(); ++it)
            pnode->unadvertise(it->first);
    }
    s_mapAdvertised.clear();
}


static rl_event_hook_fcn_ptr octave_rl_event_hook = NULL;
static int rosoct_hook(void)
{
    if( octave_rl_event_hook != NULL )
        octave_rl_event_hook();

    //BEGIN_INTERRUPT_IMMEDIATELY_IN_FOREIGN_CODE;
    //BEGIN_INTERRUPT_WITH_EXCEPTIONS;
    __rosoct_worker(0,NULL,0,NULL); // flush all
    //END_INTERRUPT_WITH_EXCEPTIONS;
    //END_INTERRUPT_IMMEDIATELY_IN_FOREIGN_CODE;
    return 0;
}

void rosoct_exit()
{
    ROS_INFO("exiting rosoct");
    reset_all();
    if( ros::Node::instance() ) {

        delete ros::Node::instance();
    }
}

void __rosoct_exit(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    rosoct_exit();
}

bool install_rosoct(bool bRegisterHook)
{
    // the one thing stopping matlab is figuring out a way to install multiple mex functions
    s_mapFunctions.clear();
    s_mapFunctions["create_session"] = __rosoct_create_session;
    s_mapFunctions["session_call"] = __rosoct_session_call;
    s_mapFunctions["service_call"] = __rosoct_service_call;
    s_mapFunctions["msg_subscribe"] = __rosoct_msg_subscribe;
    s_mapFunctions["msg_unsubscribe"] = __rosoct_msg_unsubscribe;
    s_mapFunctions["set_param"] = __rosoct_set_param;
    s_mapFunctions["get_param"] = __rosoct_get_param;
    s_mapFunctions["advertise"] = __rosoct_advertise;
    s_mapFunctions["unadvertise"] = __rosoct_unadvertise;
    s_mapFunctions["advertise_service"] = __rosoct_advertise_service;
    s_mapFunctions["unadvertise_service"] = __rosoct_unadvertise_service;
    s_mapFunctions["terminate_session"] = __rosoct_terminate_session;
    s_mapFunctions["session_call"] = __rosoct_session_call;
    s_mapFunctions["publish"] = __rosoct_publish;
    s_mapFunctions["get_topics"] = __rosoct_get_topics;
    s_mapFunctions["wait_for_service"] = __rosoct_wait_for_service;
    s_mapFunctions["check_master"] = __rosoct_check_master;
    s_mapFunctions["worker"] = __rosoct_worker;
    s_mapFunctions["time_now"] = __rosoct_time_now;
    s_mapFunctions["exit"] = __rosoct_exit;

//    octave_shlib curlib;
//    install_mex_function((void*)__rosoct_create_session, false, "__rosoct_create_session", curlib, true, true);
//    install_mex_function((void*)__rosoct_session_call, false, "__rosoct_session_call", curlib, true, true);
//    install_mex_function((void*)__rosoct_service_call, false, "__rosoct_service_call", curlib, true, true);
//    install_mex_function((void*)__rosoct_msg_subscribe, false, "__rosoct_msg_subscribe", curlib, true, true);
//    install_mex_function((void*)__rosoct_msg_unsubscribe, false, "__rosoct_msg_unsubscribe", curlib, true, true);
//    install_mex_function((void*)__rosoct_set_param, false, "__rosoct_set_param", curlib, true, true);
//    install_mex_function((void*)__rosoct_get_param, false, "__rosoct_get_param", curlib, true, true);
//    install_mex_function((void*)__rosoct_advertise, false, "__rosoct_advertise", curlib, true, true);
//    install_mex_function((void*)__rosoct_unadvertise, false, "__rosoct_unadvertise", curlib, true, true);
//    install_mex_function((void*)__rosoct_advertise_service, false, "__rosoct_advertise_service", curlib, true, true);
//    install_mex_function((void*)__rosoct_unadvertise_service, false, "__rosoct_unadvertise_service", curlib, true, true);
//    install_mex_function((void*)__rosoct_terminate_session, false, "__rosoct_terminate_session", curlib, true, true);
//    install_mex_function((void*)__rosoct_session_call, false, "__rosoct_session_call", curlib, true, true);
//    install_mex_function((void*)__rosoct_publish, false, "__rosoct_publish", curlib, true, true);
//    install_mex_function((void*)__rosoct_get_topics, false, "__rosoct_get_topics", curlib, true, true);
//    install_mex_function((void*)__rosoct_wait_for_service, false, "__rosoct_wait_for_service", curlib, true, true);
//    install_mex_function((void*)__rosoct_check_master, false, "__rosoct_check_master", curlib, true, true);
//    install_mex_function((void*)__rosoct_worker, false, "__rosoct_worker", curlib, true, true);
//    install_mex_function((void*)__rosoct_exit, false, "__rosoct_exit", curlib, true, true);

#ifdef COMPILE_FOR_MATLAB
    mexAtExit(rosoct_exit); // register mex function (buggy for octave)
#else // octave
    if( bRegisterHook ) {
        ROS_INFO("registering rosoct hook to readline");
        octave_rl_event_hook = octave_rl_get_event_hook();
        octave_rl_set_event_hook(rosoct_hook);
    }

    // register octave exit function
    octave_value_list args; args.resize(1);
    args(0) = "rosoct_exit";
    feval("atexit", args);
#endif

    return true;
}

extern "C"
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    static bool s_bInstalled = false;
    bool bRegisterHook = true;
    bool bSuccess = true;
    map<string,FunctionPtr>::iterator itcallfn = s_mapFunctions.end();

    string cmd;
    for(int i = 0; i < nrhs; ++i) {
        cmd.resize(mxGetNumberOfElements(prhs[i])+1);
        mxGetString(prhs[i],&cmd[0],cmd.size()); cmd.resize(cmd.size()-1);
        if( cmd == "clear" ) { // clears all subscriptions and handles
            reset_all();
        }
        else if( cmd == "shutdown") {
            if( octave_rl_event_hook != NULL ) {
                ROS_INFO("unregistering rosoct hook");
                octave_rl_set_event_hook(octave_rl_event_hook);
                octave_rl_event_hook = NULL;
            }
            reset_all();
            if( ros::Node::instance() ) {

                delete ros::Node::instance();
            }
        }
        else if( cmd == "nohook") {
            bRegisterHook = false;
        }
        else if( cmd == "argv") {
            // setup the argv for roscpp (do not restart it)
            s_vargv.clear();
            if( i+1 < nrhs ) {
                string args, arg;
                args.resize(mxGetNumberOfElements(prhs[i+1])+1);
                mxGetString(prhs[i+1],&args[0],args.size()); args.resize(args.size()-1);
                stringstream ss(args);
                while(!ss.eof()) {
                    ss >> arg;
                    if( !ss || arg.size() == 0)
                        break;
                    s_vargv.push_back(arg);
                }
            }
        }
        else {
            if( !s_bInstalled ) {
                if( install_rosoct(bRegisterHook) )
                    s_bInstalled = true;
                else {
                    ROS_FATAL("rosoct failed to initialize");
                    bSuccess = false;
                }
            }

            itcallfn = s_mapFunctions.find(cmd);
            if( itcallfn != s_mapFunctions.end() )
                break;
        }
    }

    if( !s_bInstalled ) {
        if( install_rosoct(bRegisterHook) )
            s_bInstalled = true;
        else {
            ROS_FATAL("rosoct failed to initialize");
            bSuccess = false;
        }
    }

    if( itcallfn != s_mapFunctions.end() ) {
        assert(nrhs>=1);
        itcallfn->second(nlhs,plhs,nrhs-1,&prhs[1]);
    }
    else {
        if( nlhs > 0 )
            plhs[0] = mxCreateDoubleScalar((double)bSuccess);
    }
}
