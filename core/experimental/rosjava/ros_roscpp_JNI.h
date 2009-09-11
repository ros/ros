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




#include <jni.h>
/* Header for class ros_roscpp_JNI */

#ifndef _Included_ros_roscpp_JNI
#define _Included_ros_roscpp_JNI
#ifdef __cplusplus
extern "C" {
#endif


	/************************************************************
	 *   Node Handle creation, running, destruction, and features
	 ************************************************************/

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_init
  (JNIEnv * env, jclass __jni, jstring name, jboolean noSigintHandler, jboolean anonymousName, jboolean noRosout, jobjectArray jargs);

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createNodeHandle
  (JNIEnv * env, jclass __jni, jstring jns, jobjectArray jmappings);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdown
  (JNIEnv * env, jclass __jni, jlong cppHandle);


JNIEXPORT void JNICALL Java_ros_roscpp_JNI_spin
  (JNIEnv * env, jclass __jni);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_spinOnce
  (JNIEnv * env, jclass __jni);


JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_checkMaster
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getMasterHost
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getMasterPort
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setMasterRetryTimeout
  (JNIEnv * env, jclass __jni, jlong cppHandle, jint ms);


JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getName
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_mapName
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jname);


/************************************************************
 *   Logging
 ************************************************************/

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logDebug
  (JNIEnv * env, jclass __jni, jstring jmessage);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logInfo
  (JNIEnv * env, jclass __jni, jstring jmessage);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logWarn
  (JNIEnv * env, jclass __jni, jstring jmessage);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logError
  (JNIEnv * env, jclass __jni, jstring jmessage);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_logFatal
  (JNIEnv * env, jclass __jni, jstring jmessage);


/************************************************************
 *   Getting miscellaneous info
 ************************************************************/

JNIEXPORT jobject JNICALL Java_ros_roscpp_JNI_now
  (JNIEnv * env, jclass __jni);

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getSubscribedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getAdvertisedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle);

JNIEXPORT jobjectArray JNICALL Java_ros_roscpp_JNI_getPublishedTopics
  (JNIEnv * env, jclass __jni, jlong cppHandle);



/************************************************************
 *   Parameters
 ************************************************************/


JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_hasParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam);

// Throws RosException on failure.
JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_getBooleanParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache);

// Throws RosException on failure.
JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getIntParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache);

// Throws RosException on failure.
JNIEXPORT jdouble JNICALL Java_ros_roscpp_JNI_getDoubleParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache);

// Throws RosException on failure.
JNIEXPORT jstring JNICALL Java_ros_roscpp_JNI_getStringParam
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean cache);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2Z
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jboolean val);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2I
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jint val);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2D
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jdouble val);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_setParam__JLjava_lang_String_2Ljava_lang_String_2
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jparam, jstring val);



/************************************************************
 *   Subscriptions
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createSubCallback
  (JNIEnv * env, jclass __jni, jobject jcallback, jobject messageTemplate);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_deleteSubCallback
  (JNIEnv * env, jclass __jni, jlong cppCallback);


JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_subscribe
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jtopic, jlong cppCallback, jint queueSize);

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isSubscriberValid
  (JNIEnv * env, jclass __jni, jlong cppSubscriber);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownSubscriber
  (JNIEnv * env, jclass __jni, jlong cppSubscriber);



/************************************************************
 *   Publications
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_advertise
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring jtopic, jobject jmessage, jint queueSize, jboolean latch);

JNIEXPORT jint JNICALL Java_ros_roscpp_JNI_getNumSubscribers
  (JNIEnv * env, jclass __jni, jlong cppPublisher);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_publish
  (JNIEnv * env, jclass __jni, jlong cppPublisher, jobject jmessage);

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isPublisherValid
  (JNIEnv * env, jclass __jni, jlong cppPublisher);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownPublisher
  (JNIEnv * env, jclass __jni, jlong cppPublisher);


/************************************************************
 *   Service Clients
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_serviceClient
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring name, jstring md5, jboolean persist, jobjectArray jheaderValues);

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_callService
  (JNIEnv * env, jclass __jni, jlong cppServiceClient, jobject jrequest, jobject jresponse, jstring md5);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownServiceClient
  (JNIEnv * env, jclass __jni, jlong cppServiceClient);


/************************************************************
 *   Service Servers
 ************************************************************/

JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_createSrvCallback
  (JNIEnv * env, jclass __jni, jobject jcallback, jstring serviceMD5, jstring serviceDataType, jobject jreq, jobject jres);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_deleteSrvCallback
  (JNIEnv * env, jclass __jni, jlong cppCallback);


JNIEXPORT jlong JNICALL Java_ros_roscpp_JNI_advertiseService
  (JNIEnv * env, jclass __jni, jlong cppHandle, jstring name, jlong cppCallback);

JNIEXPORT jboolean JNICALL Java_ros_roscpp_JNI_isServiceServerValid
  (JNIEnv * env, jclass __jni, jlong cppServiceServer);

JNIEXPORT void JNICALL Java_ros_roscpp_JNI_shutdownServiceServer
  (JNIEnv * env, jclass __jni, jlong cppServiceServer);


#ifdef __cplusplus
}
#endif
#endif
