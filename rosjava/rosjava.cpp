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

// author: Jason Wolfe

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>

#include <cstdio>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/tss.hpp>


#include <ros/init.h>
#include <ros/time.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <ros/node_handle.h>
#include <ros/service.h>

#include <signal.h>

#include "ros_roscpp_JNI.h"

// I want assertions always on.
#define MY_ROS_ASSERT(cond) \
  do { \
    if (!(cond)) { \
      ROS_FATAL("ASSERTION FAILED\n\tfile = %s\n\tline = %d\n\tcond = %s\n", __FILE__, __LINE__, #cond); \
      ROS_ISSUE_BREAK() \
    } \
  } while (0)



#ifdef MSVC
#define PRIdS "Id"
#else
#define PRIdS "zd"
#endif

using namespace ros;
using namespace std;





/******************************************************
 *   Misc Helpers
 ******************************************************/

string getString(JNIEnv * env, jstring s)
{
     if(s == NULL || env == NULL) return string("");
     const char* str = env->GetStringUTFChars(s, 0);
     string ret = str;
     env->ReleaseStringUTFChars(s, str);

     return ret;
}

jstring makeString(JNIEnv * env, const string &s) {
	return (jstring) env->NewStringUTF(s.c_str());
}


/******************************************************
 *   Thread-local JNIEnv objects
 ******************************************************/

static JavaVM *vm = NULL;

bool dieOnException(JNIEnv * env, const char * message = "") {
	if (env->ExceptionOccurred()) {
		env->ExceptionDescribe();
		ROS_FATAL(message);
		MY_ROS_ASSERT(false);
		return false;
	}
	return true;
}

class ThreadJNIEnv {
public:
	bool _detach;
	JNIEnv *env;
	ThreadJNIEnv() {
		cout << "Attaching " << boost::this_thread::get_id() << endl;
		vm->AttachCurrentThread((void **) &env, NULL);
		MY_ROS_ASSERT(env != NULL);
		_detach = true;
//		dieOnException(env, "Env already in exception state.");
	}

	ThreadJNIEnv(JNIEnv *e) {
		env = e;
		_detach = false;
		MY_ROS_ASSERT(env != NULL);
	}

	~ThreadJNIEnv() {
		if (_detach) {
			cout << "Detaching " << boost::this_thread::get_id() << endl;
			vm->DetachCurrentThread();
		}
	}
};

static boost::thread_specific_ptr<ThreadJNIEnv> tp;

JNIEnv *getJNIEnv(){
	JNIEnv *ret;
//	if (vm->GetEnv((void **)&ret, 0x00010001) == JNI_OK) {
//		cout << "GOT ENV";
//		return ret;
//	}

	ThreadJNIEnv *tenv = tp.get();
	if (tenv == NULL) {
		tenv = new ThreadJNIEnv();
		tp.reset(tenv);
	}

	ret = tenv->env;
	MY_ROS_ASSERT(ret != NULL);
	dieOnException(ret, "Env already in exception state.");
	return ret;
}




/************************************************************
 *   Node Handle creation, running, destruction, and features
 ************************************************************/


static jclass jObject;
//static jclass jRuntimeException;
static jclass jRosException;
static jclass jTime;
static jclass jString;
static jclass jMessage;
static jclass jSubscriberCallback;
static jclass jServiceCallback;
static jclass jByteBuffer;

static jmethodID jTimeCtor;

static jmethodID jMessageClone;
static jmethodID jMessageGetDataType;
static jmethodID jMessageGetMD5Sum;
static jmethodID jMessageGetServerMD5Sum;
static jmethodID jMessageGetMessageDefinition;
static jmethodID jMessageSerializationLength;
static jmethodID jMessageSerialize;
static jmethodID jMessageDeserialize;
static jmethodID jSubscriberCallbackCall;
static jmethodID jServiceCallbackCall;
static jmethodID jByteBufferOrder;

static jobject jByteOrderLittleEndian;


bool cacheClass(JNIEnv * env, jclass &ref, const char * name) {
	ref = env->FindClass(name);
	if (ref == NULL) return false;
	ref = (jclass) env->NewGlobalRef(ref);
	return true;
}

bool cacheMethod(JNIEnv * env, jmethodID &ref, jclass cls, const char * name, const char * args ) {
	ref = env->GetMethodID(cls, name, args);
	return (ref != NULL);
}

/*
 * Class:     ros_roscpp_CppNode
 * Method:    startNode
 * Signature: (Ljava/lang/Class;Ljava/lang/Class;Ljava/lang/Class;Ljava/lang/Class;Ljava/lang/Class;)V
 */

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_init
  (JNIEnv * env, jclass __jni, jstring name, jboolean noSigintHandler, jboolean anonymousName, jboolean noRosout, jobjectArray args)
{
	if (!env->GetJavaVM(&vm) < 0) return;
	tp.reset(new ThreadJNIEnv(env));

	if (!cacheClass(env, jObject, "java/lang/Object")) return; // Exception thrown
	if (!cacheClass(env, jString, "java/lang/String")) return; // Exception thrown
	//	if (!cacheClass(env, jRuntimeException, "java/lang/RuntimeException")) return; // Exception thrown
	if (!cacheClass(env, jRosException, "ros/RosException")) return; // Exception thrown
	if (!cacheClass(env, jTime, "ros/communication/Time")) return; // Exception thrown
	if (!cacheClass(env, jMessage, "ros/communication/Message")) return; // Exception thrown
	if (!cacheClass(env, jSubscriberCallback, "ros/Subscriber$Callback")) return; // Exception thrown
	if (!cacheClass(env, jServiceCallback, "ros/ServiceServer$Callback")) return; // Exception thrown
	if (!cacheClass(env, jByteBuffer, "java/nio/ByteBuffer")) return; // Exception thrown

	if (!cacheMethod(env, jTimeCtor, jTime, "<init>", "(II)V")) return; // Exception thrown

	if (!cacheMethod(env, jMessageClone, jMessage, "clone", "()Lros/communication/Message;")) return; // Exception thrown
	if (!cacheMethod(env, jMessageGetDataType, jMessage, "getDataType", "()Ljava/lang/String;")) return; // Exception thrown
	if (!cacheMethod(env, jMessageGetMD5Sum, jMessage, "getMD5Sum", "()Ljava/lang/String;")) return; // Exception thrown
	if (!cacheMethod(env, jMessageGetServerMD5Sum, jMessage, "getServerMD5Sum", "()Ljava/lang/String;")) return; // Exception thrown
	if (!cacheMethod(env, jMessageGetMessageDefinition, jMessage, "getMessageDefinition", "()Ljava/lang/String;")) return; // Exception thrown
	if (!cacheMethod(env, jMessageSerializationLength, jMessage, "serializationLength", "()I")) return; // Exception thrown
	if (!cacheMethod(env, jMessageSerialize, jMessage, "serialize", "(Ljava/nio/ByteBuffer;I)V")) return; // Exception thrown
	if (!cacheMethod(env, jMessageDeserialize, jMessage, "deserialize", "(Ljava/nio/ByteBuffer;)V")) return; // Exception thrown
	if (!cacheMethod(env, jSubscriberCallbackCall, jSubscriberCallback, "call", "(Lros/communication/Message;)V")) return; // Exception thrown
	if (!cacheMethod(env, jServiceCallbackCall, jServiceCallback, "call", "(Lros/communication/Message;)Lros/communication/Message;")) return; // Exception thrown
	if (!cacheMethod(env, jByteBufferOrder, jByteBuffer, "order", "(Ljava/nio/ByteOrder;)Ljava/nio/ByteBuffer;")) return; // Exception thrown

	jclass jByteOrder = env->FindClass("java/nio/ByteOrder");
	if (jByteOrder == NULL) return;
	jfieldID fid      = env->GetStaticFieldID(jByteOrder, "LITTLE_ENDIAN", "Ljava/nio/ByteOrder;");
	if (fid == NULL) return;
	jByteOrderLittleEndian = env->NewGlobalRef(env->GetStaticObjectField(jByteOrder, fid));
	if (jByteOrderLittleEndian == NULL) return;


	int len = env->GetArrayLength(args);

	vector<string> vargs;
	for(int i = 0; i < len; i++) vargs.push_back(getString(env, (jstring) env->GetObjectArrayElement(args,i)));

    vector<char*> argv(len);
    for(int i = 0; i < len; ++i)
        argv[i] = &vargs[i][0];

    uint32_t options = (noSigintHandler ? ros::init_options::NoSigintHandler : 0) |
                 	   (anonymousName   ? ros::init_options::AnonymousName : 0) |
	                   (noRosout        ? ros::init_options::NoRosout      : 0);
    ros::init(len, len > 0 ? &argv[0] : NULL, getString(env, name), options);
}

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createNodeHandle
  (JNIEnv * env, jclass __jni, jstring jns, jobjectArray jmappings)
{
	map<string, string> remappings;
	int len = env->GetArrayLength(jmappings);
	MY_ROS_ASSERT(len % 2 == 0);
	for(int i = 0; i < len; i+=2)
		remappings[getString(env, (jstring) env->GetObjectArrayElement(jmappings, i))] =
				   getString(env, (jstring) env->GetObjectArrayElement(jmappings, i+1));

	return (long) (new NodeHandle(getString(env, jns), remappings));
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdown
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
//    ROS_INFO("exiting rosjava");

/*    env->DeleteGlobalRef(jObject);
	env->DeleteGlobalRef(jString);
	env->DeleteGlobalRef(jRosException);
	env->DeleteGlobalRef(jTime);
	env->DeleteGlobalRef(jMessage);
	env->DeleteGlobalRef(jSubscriberCallback);
	env->DeleteGlobalRef(jServiceCallback);*/
    // Since init is one-time right now, just never release these ...

    NodeHandle *handle = (NodeHandle *) cppHandle;
	handle->shutdown();
	delete handle;
}




JNIEXPORT void JNICALL Java_ros_roscpp_JNI_spin
  (JNIEnv * env, jclass __jni)
{
	ros::spin();
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_spinOnce
  (JNIEnv * env, jclass __jni)
{
	ros::spinOnce();
}




JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_checkMaster
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
       return ros::master::check();
}

JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getMasterHost
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return makeString(env, handle->getMasterHost());
}

JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getMasterPort
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->getMasterPort();
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setMasterRetryTimeout
  (JNIEnv * env, jclass __jni, jlong cppHandle, jint ms)
{
//	NodeHandle *handle = (NodeHandle *) cppHandle;
	ros::master::setRetryTimeout(WallDuration((ms/1000),(ms % 1000) * 1000000));
}




JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getName
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
//	NodeHandle *handle = (NodeHandle *) cppHandle;
//	return makeString(env, handle->getName());
	return makeString(env, ros::this_node::getName());
}


JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_mapName
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jname)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return makeString(env, handle->resolveName(getString(env, jname)));
}


/************************************************************
 *   Logging
 ************************************************************/

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logDebug
  (JNIEnv * env, jclass __jni, jstring jmessage)
{
	ROS_DEBUG(getString(env,jmessage).c_str());
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logInfo
  (JNIEnv * env, jclass __jni, jstring jmessage)
{
	ROS_INFO(getString(env,jmessage).c_str());
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logWarn
  (JNIEnv * env, jclass __jni, jstring jmessage)
{
	ROS_WARN(getString(env,jmessage).c_str());
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logError
  (JNIEnv * env, jclass __jni, jstring jmessage)
{
	ROS_ERROR(getString(env,jmessage).c_str());
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logFatal
  (JNIEnv * env, jclass __jni, jstring jmessage)
{
	ROS_FATAL(getString(env,jmessage).c_str());
}




/************************************************************
 *   Getting miscellaneous info
 ************************************************************/

JNIEXPORT jobject JNICALL Java_ros_roscpp_JNI_now
  (JNIEnv * env, jclass __jni) {
    ros::Time t = ros::Time::now();
	jobject ret = env->NewObject(jTime, jTimeCtor, t.sec, t.nsec);
    return ret;
}

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getSubscribedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
	//NodeHandle *handle = (NodeHandle *) cppHandle;

    vector<string> topics;
    ros::this_node::getSubscribedTopics(topics);

    int sz = topics.size();
    jobjectArray oa = env->NewObjectArray(sz, jString, env->NewStringUTF(""));
    if (oa == NULL) return NULL;

    for(int i = 0; i < sz; i++) {
    	env->SetObjectArrayElement(oa, i, makeString(env, topics[i]));
    }

	return oa;
}

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getAdvertisedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
	//NodeHandle *handle = (NodeHandle *) cppHandle;

    vector<string> topics;
    ros::this_node::getAdvertisedTopics(topics);

    int sz = topics.size();
    jobjectArray oa = env->NewObjectArray(sz, jString, env->NewStringUTF(""));
    if (oa == NULL) return NULL;

    for(int i = 0; i < sz; i++) {
    	env->SetObjectArrayElement(oa, i, makeString(env, topics[i]));
    }

	return oa;
}

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getPublishedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle)
{
	//NodeHandle *handle = (NodeHandle *) cppHandle;

    vector<ros::master::TopicInfo> vtopics;
    ros::master::getTopics(vtopics);

    int sz = vtopics.size();
    jobjectArray oa = env->NewObjectArray(sz * 3, jString, env->NewStringUTF(""));
    if (oa == NULL) return NULL;

    for(int i = 0; i < sz; i++) {
    	env->SetObjectArrayElement(oa, i * 3, env->NewStringUTF(vtopics[i].name.c_str()));
    	env->SetObjectArrayElement(oa, i * 3 + 1, env->NewStringUTF(vtopics[i].datatype.c_str()));
    	env->SetObjectArrayElement(oa, i * 3 + 2, env->NewStringUTF(vtopics[i].md5sum.c_str()));
    }

	return oa;
}


/************************************************************
 *   Parameters
 ************************************************************/


JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_hasParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->hasParam(getString(env, jparam));
}

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_getBooleanParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	bool ret;
	if (!handle->getParam(getString(env, jparam), ret, cache)) {
    	env->ThrowNew(jRosException, "Param could not be fetched!");
    	return false;
	}
	return ret;
}

JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getIntParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	int ret;
	if (!handle->getParam(getString(env, jparam), ret, cache)) {
    	env->ThrowNew(jRosException, "Param could not be fetched!");
    	return 0;
	}
	return ret;
}

JNIEXPORT jdouble JNICALL Java_ros_roscpp_JNI_getDoubleParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	double ret;
	if (!handle->getParam(getString(env, jparam), ret, cache)) {
    	env->ThrowNew(jRosException, "Param could not be fetched!");
    	return 0;
	}
	return ret;
}

JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getStringParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	string ret;
	if (!handle->getParam(getString(env, jparam), ret, cache)) {
    	env->ThrowNew(jRosException, "Param could not be fetched!");
    	return 0;
	}
	return makeString(env, ret);
}



JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2Z
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean val)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->setParam(getString(env, jparam), val);
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2I
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jint val)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->setParam(getString(env, jparam), (int)val);
}


JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2D
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jdouble val)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->setParam(getString(env, jparam), val);
}


JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2Ljava_lang_String_2
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jstring val)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	return handle->setParam(getString(env, jparam), getString(env, val));
}





/************************************************************
 *   Message wrapper
 ************************************************************/


class JavaMessage : public ros::Message
{
public:
    jobject _message;

    JavaMessage(jobject message) : _message(getJNIEnv()->NewGlobalRef(message)) {}

    JavaMessage(const JavaMessage& r) : Message()
    {
//    	cout << "CLONE" << endl;
      	JNIEnv * env = getJNIEnv();
        _message = env->CallObjectMethod(r._message,jMessageClone);
       	MY_ROS_ASSERT(_message && dieOnException(env));
       	_message = env->NewGlobalRef(_message);
    }

    virtual ~JavaMessage() {
    	getJNIEnv()->DeleteGlobalRef(_message);
    }

    virtual void replaceContents (jobject newMessage) {
 //   	cout << "Replace!" << endl;
      	JNIEnv * env = getJNIEnv();
      	env->DeleteGlobalRef(_message);
      	_message = env->NewGlobalRef(newMessage);
       	MY_ROS_ASSERT(_message && dieOnException(env));
    }

    virtual const string __getDataType() const {
//		cout << "getDataType:";
    	JNIEnv * env = getJNIEnv();
    	jstring s = (jstring) env->CallObjectMethod(_message, jMessageGetDataType);
    	MY_ROS_ASSERT(s && dieOnException(env));
//    	cout << getString(env, s).c_str() << endl;
    	return getString(env, s);
    }

    virtual const string __getMD5Sum()   const {
//		cout << "getMD5:";
    	JNIEnv * env = getJNIEnv();
    	jstring s = (jstring) env->CallObjectMethod(_message, jMessageGetMD5Sum);
    	MY_ROS_ASSERT(s && dieOnException(env));
//    	cout << getString(env, s).c_str() << endl;
    	return getString(env, s);
    }

    virtual const string __getServerMD5Sum()   const {
//		cout << "getMD5:";
    	JNIEnv * env = getJNIEnv();
    	jstring s = (jstring) env->CallObjectMethod(_message, jMessageGetServerMD5Sum);
    	MY_ROS_ASSERT(s && dieOnException(env));
//    	cout << getString(env, s).c_str() << endl;
    	return getString(env, s);
    }

    virtual const string __getMessageDefinition()   const {
//		cout << "getMD: ";
    	JNIEnv * env = getJNIEnv();
    	jstring s = (jstring) env->CallObjectMethod(_message, jMessageGetMessageDefinition);
       	MY_ROS_ASSERT(s && dieOnException(env));
//    	cout << getString(env, s).c_str() << endl;
       	return getString(env, s);
    }

    uint32_t serializationLength() const {
//		cout << "serialLength:" << endl;
    	JNIEnv * env = getJNIEnv();
    	int len =  getJNIEnv()->CallIntMethod(_message, jMessageSerializationLength);
    	dieOnException(env);
 //   	cout << len << endl;
    	return len;
    }

    virtual uint8_t *serialize(uint8_t *writePtr, uint32_t seqid) const {
//		cout << "serialize" << endl;
    	JNIEnv * env = getJNIEnv();
    	uint32_t len = serializationLength();
    	jobject bb = env->NewDirectByteBuffer(writePtr, len);
       	MY_ROS_ASSERT(bb && dieOnException(env));
       	bb = env->CallObjectMethod(bb, jByteBufferOrder, jByteOrderLittleEndian);
       	MY_ROS_ASSERT(bb && dieOnException(env));
       	env->CallObjectMethod(_message, jMessageSerialize, bb, seqid);
    	dieOnException(env);
       	return writePtr + len;
    }

    virtual uint8_t *deserialize(uint8_t *readPtr) {
//		cout << "deserialize" << endl;
    	JNIEnv * env = getJNIEnv();
    	int sz = __serialized_length;
    	if (sz == 0) {
//    		ROS_WARN("empty message!");
    		return readPtr;
    	}
    	jobject bb = env->NewDirectByteBuffer(readPtr, sz);
       	MY_ROS_ASSERT(bb && dieOnException(env));
       	bb = env->CallObjectMethod(bb, jByteBufferOrder, jByteOrderLittleEndian);
       	MY_ROS_ASSERT(bb && dieOnException(env));
       	env->CallObjectMethod(_message, jMessageDeserialize, bb);
    	dieOnException(env);
       	return readPtr + sz;
    }
};


/************************************************************
 *   Subscriptions
 ************************************************************/

class JavaSubscriptionMessageHelper : public ros::SubscriptionMessageHelper {
public:
	jobject _scb;
	JavaMessage _msg;
	string md5, datatype;

	JavaSubscriptionMessageHelper(jobject scb, jobject tmpl) : _scb(getJNIEnv()->NewGlobalRef(scb)), _msg(tmpl) {
		md5 = _msg.__getMD5Sum();
		datatype = _msg.__getDataType();
	}
	~JavaSubscriptionMessageHelper() { getJNIEnv()->DeleteGlobalRef(_scb); }

	virtual MessagePtr create() { return boost::shared_ptr<Message>(new JavaMessage(_msg)); }

	virtual std::string getMD5Sum() { return md5; }
	virtual std::string getDataType() { return datatype; }

	virtual void call(const MessagePtr &msg) {
    	getJNIEnv()->CallVoidMethod(_scb, jSubscriberCallbackCall, ((JavaMessage *)msg.get())->_message);
	}
};

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createSubCallback
  (JNIEnv * env, jclass __jni, jobject jcallback, jobject messageTemplate)
{
	return (jlong) new boost::shared_ptr<JavaSubscriptionMessageHelper>(new JavaSubscriptionMessageHelper(jcallback, messageTemplate));
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_deleteSubCallback
  (JNIEnv * env, jclass __jni, jlong cppCallback)
{
	boost::shared_ptr<JavaSubscriptionMessageHelper> *callback = (boost::shared_ptr<JavaSubscriptionMessageHelper> *) cppCallback;
	delete callback;
}

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_subscribe
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jtopic, jlong cppCallback, jint queueSize)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	boost::shared_ptr<SubscriptionMessageHelper> *callback = (boost::shared_ptr<SubscriptionMessageHelper> *) cppCallback;
	SubscribeOptions so(getString(env, jtopic), queueSize, *callback);
	Subscriber subscriber = handle->subscribe(so);
	if (subscriber) return (jlong) new Subscriber(subscriber);
	return 0;
}

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isSubscriberValid
  (JNIEnv * env, jclass __jni, jlong cppSubscriber)
{
	Subscriber *subscriber = (Subscriber *) cppSubscriber;
	return (*subscriber) && ros::ok() ? true : false;
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownSubscriber
  (JNIEnv * env, jclass __jni, jlong cppSubscriber)
{
	Subscriber *subscriber = (Subscriber *) cppSubscriber;
	delete subscriber;
}



/************************************************************
 *   Publications
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_advertise
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jtopic, jobject jmessage, jint queueSize, jboolean latch)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	JavaMessage msg(jmessage);
	AdvertiseOptions ao(getString(env, jtopic), queueSize, msg.__getMD5Sum(), msg.__getDataType(), msg.__getMessageDefinition());
	ao.latch = latch;
	Publisher publisher = handle->advertise(ao);
	if (publisher) return (jlong) new Publisher(publisher);
	return 0;
}

JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getNumSubscribers
  (JNIEnv * env, jclass __jni, jlong cppPublisher) {
	Publisher *publisher = (Publisher *) cppPublisher;
	return publisher->getNumSubscribers();
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_publish
  (JNIEnv * env, jclass __jni, jlong cppPublisher, jobject jmessage) {
	Publisher *publisher = (Publisher *) cppPublisher;
	JavaMessage message(jmessage);
	publisher->publish(message);
}

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isPublisherValid
  (JNIEnv * env, jclass __jni, jlong cppPublisher)
{
	Publisher *publisher = (Publisher *) cppPublisher;
	return (*publisher)  && ros::ok() ? true : false;
}


JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownPublisher
  (JNIEnv * env, jclass __jni, jlong cppPublisher)
{
	Publisher *publisher = (Publisher *) cppPublisher;
	delete publisher;
}



/************************************************************
 *   Service Clients
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_serviceClient
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring name, jstring md5, jboolean persist, jobjectArray jheaderValues)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	map<string, string> headers;
	int len = env->GetArrayLength(jheaderValues);
	for(int i = 0; i < len; i+=2) headers[getString(env, (jstring) env->GetObjectArrayElement(jheaderValues,i))] = getString(env, (jstring) env->GetObjectArrayElement(jheaderValues,i+1));

	ServiceClientOptions sco(getString(env, name), getString(env, md5), persist, headers);
	ServiceClient client = handle->serviceClient(sco);
	if (client) return (jlong) new ServiceClient(client);
	return 0;
}

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_callService
  (JNIEnv * env, jclass __jni, jlong cppServiceClient, jobject jrequest, jobject jresponse, jstring md5)
{
	ServiceClient *client = (ServiceClient *) cppServiceClient;
	JavaMessage request(jrequest), response(jresponse);
//	cout << "about to call!" << endl;
	return client->call(request, response, getString(env, md5));
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownServiceClient
  (JNIEnv * env, jclass __jni, jlong cppServiceClient)
{
	ServiceClient *client = (ServiceClient *) cppServiceClient;
	delete client;
}


/************************************************************
 *   Service Servers
 ************************************************************/

class JavaServiceMessageHelper : public ros::ServiceMessageHelper {
public:
	jobject _scb;
	JavaMessage _req, _res;
	string md5, datatype, requestDataType, responseDataType;

	JavaServiceMessageHelper(jobject scb, string smd5, string sdatatype, jobject req, jobject res)
		: _scb(getJNIEnv()->NewGlobalRef(scb)), _req(req), _res(res), md5(smd5), datatype(sdatatype) {
		requestDataType = _req.__getDataType();
		responseDataType = _res.__getDataType();
	}
	~JavaServiceMessageHelper() { getJNIEnv()->DeleteGlobalRef(_scb); }

	virtual MessagePtr createRequest() { return boost::shared_ptr<Message>(new JavaMessage(_req)); }
	virtual MessagePtr createResponse() { return boost::shared_ptr<Message>(new JavaMessage(_res)); }

	virtual std::string getMD5Sum() { return md5; }
	virtual std::string getDataType() { return datatype; }
	virtual std::string getRequestDataType() { return requestDataType; }
	virtual std::string getResponseDataType() { return responseDataType; }

	virtual bool call(const MessagePtr &req, const MessagePtr &res) {
      	JNIEnv * env = getJNIEnv();
    	jobject jresponse = env->CallObjectMethod(_scb, jServiceCallbackCall, ((JavaMessage *)req.get())->_message);
    	MY_ROS_ASSERT(jresponse != 0 && dieOnException(env));
    	((JavaMessage *)res.get())->replaceContents(jresponse);
    	return true;
	}
};


JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createSrvCallback
  (JNIEnv * env, jclass __jni, jobject jcallback, jstring serviceMD5, jstring serviceDataType, jobject jreq, jobject jres)
{
	JavaServiceMessageHelper *helper = new JavaServiceMessageHelper(jcallback, getString(env, serviceMD5), getString(env, serviceDataType), jreq, jres);
	return (jlong) new boost::shared_ptr<JavaServiceMessageHelper>(helper);
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_deleteSrvCallback
  (JNIEnv * env, jclass __jni, jlong cppCallback)
{
	boost::shared_ptr<JavaServiceMessageHelper> *callback = (boost::shared_ptr<JavaServiceMessageHelper> *) cppCallback;
	delete callback;
}

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_advertiseService
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring name, jlong cppCallback)
{
	NodeHandle *handle = (NodeHandle *) cppHandle;
	boost::shared_ptr<ServiceMessageHelper> *callback = (boost::shared_ptr<ServiceMessageHelper> *) cppCallback;
	AdvertiseServiceOptions aso(getString(env, name), *callback);
	ServiceServer server = handle->advertiseService(aso);
	if (server) return (jlong) new ServiceServer(server);
	return 0;
}

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isServiceServerValid
  (JNIEnv * env, jclass __jni, jlong cppServiceServer)
{
	ServiceServer *server = (ServiceServer *) cppServiceServer;
	return (*server)  && ros::ok() ? true : false;
}

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownServiceServer
  (JNIEnv * env, jclass __jni, jlong cppServiceServer)
{
	ServiceServer *server = (ServiceServer *) cppServiceServer;
	delete server;
}



