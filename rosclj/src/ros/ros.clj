;///////////////////////////////////////////////////////////////////////////////
;// The rosclj package provides a thin Clojure wrapper for rosjava
;//
;// Copyright (C) 2009, Jason Wolfe
;//
;// Redistribution and use in source and binary forms, with or without
;// modification, are permitted provided that the following conditions are met:
;//   * Redistributions of source code must retain the above copyright notice,
;//     this list of conditions and the following disclaimer.
;//   * Redistributions in binary form must reproduce the above copyright
;//     notice, this list of conditions and the following disclaimer in the
;//     documentation and/or other materials provided with the distribution.
;//   * Neither the name of Stanford University nor the names of its
;//     contributors may be used to endorse or promote products derived from
;//     this software without specific prior written permission.
;//
;// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;// POSSIBILITY OF SUCH DAMAGE.
;//////////////////////////////////////////////////////////////////////////////


;;; Main Clojure wrapper for ROS core and messages/services

;; Since you can always fall back to calling Java, the focus
;; is on wrapping things that are especially verbose or would
;; lead to un-ideomatic code.

; Main things you want to call or access here:
;
; (import-ros) to import ROS Java classes into your namespace
;
; Call Java methods directly to get the global Ros instance and initialize it. 
;
; Call Java methods again to create NodeHandles, or use 
;   with-node-handle to create a node handle that is guaranteed 
;   to be destroyed when the scope is exited (exceptionally or normally).

; defmsgs and defsrvs declare the set of messages and services 
;   that you want Clojure support for (above and beyond what you
;   already get from Java).  In particular, automated conversion
;   between Clojure maps and Messages is supported for declared types,
;   using map-msg and msg-map.   
;   Only top-level messages and services need be declared; others will
;   be pulled in as-needed.
;
; import-all-msgs-and-srvs can be used to import all the requisite java
;   classes (e.g., at the repl) for ease of use.
;
; sub-cb and srv-cb are convenient wrappers for Subscription and Service
;   callbacks.  make-subscriber-callback and make-service-callback are 
;   lower-level, if you want to avoid conversion to Clojure maps.
;
; wait-for-subscribers, get-message, put-message, and 
;   call-service do more or less what they say.  The latter three also come
;   with "cached" versions that transparently keep the corresponding 
;   Subscribers, Publishers, and ServiceClients alive until the 
;   NodeHandle is shutdown for comparable speed to what would be achieved
;   by manually managing these connections.  


(ns ros.ros
  )

(set! *warn-on-reflection* true)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                          Misc helper functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defn- symbol-cat [& args]
  "Produce a symbol named by the concatenation of the 
   string representations of args."
  (symbol (apply str args)))

(defmacro lazy-get* 
  "Like get but lazy about default"
  [m k d]
  `(if-let [pair# (find ~m ~k)] 
       (val pair#)
     ~d))

(defn safe-get* 
  "Like get but throw an exception if not found"
  [m k] 
  (lazy-get* m k (throw (IllegalArgumentException. (format "Key %s not found in %s" k m)))))

(defmacro assert-is*
  "Like assert, but prints some more info about the offending 
   form (may multiple eval on error)"
  ([form] `(assert-is* ~form ""))
  ([form format-str & args]
     `(when-not ~form
	(throw (Exception. (str (format ~format-str ~@args) 
				": Got " '~form " as " 
				(cons '~(first form) (list ~@(next form)))))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                              Setting up ROS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmacro import-ros 
  "Import the core rosjava classes into the current namespace."
  []
  `(do (import ~'[ros Ros NodeHandle RosException 
		  Publisher Subscriber Subscriber$Callback Subscriber$QueueingCallback 
		  ServiceClient ServiceServer ServiceServer$Callback]
	       ~'[ros.communication Message Service Time Duration])
       nil))

(import-ros)

(defmacro with-node-handle 
  "Create a node handle with arguments provided by args, bind it 
   to nh-var in the execution of body, and destroy it upon exiting
   this form (exceptionally or normally."
  [[nh-var & args] & body]
  `(let [~nh-var (.createNodeHandle  ~@args)]
     (Thread/sleep 100) ; Allow time to get time, ect ???
     (try (do ~@body) (finally (.shutdown ~nh-var)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                   Core handling of Messages and Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defn- array-type [#^String t]
  (when (.endsWith t "[]")
    (.substring t 0 (- (count t) 2))))

(defn- submsg-type [#^String t]
  (when (not (#{"byte" "int" "long" "short" "float" "double" "string"
		"ros.communication.duration" "ros.communication.time" "java.lang.string"}
	      (.toLowerCase t)))
    (resolve (symbol t))))


(defn message-field-types [class-symbol]
  (.invoke (.getMethod (Class/forName (str class-symbol)) 
				   "fieldTypes" (make-array Class 0)) 
		       nil (make-array Object 0)))

(defn message-submessage-types [class-symbol]
 (map symbol
  (.invoke (.getMethod (Class/forName (str class-symbol)) 
				   "submessageTypes" (make-array Class 0)) 
		       nil (make-array Object 0))))

(defonce *primitive-coerce-map*
  {"double" double "float" float "long" long "int" int "short" short "byte" byte})

(defonce *primitive-type-map*
  {"double" 'Double/TYPE "float" 'Float/TYPE "long" 'Long/TYPE
   "int" 'Integer/TYPE "short" 'Short/TYPE "byte" 'Byte/TYPE})



(def *msgs* (atom #{}))
(defn msg-list [] @*msgs*)


(defmulti msg-map 
  "Convert an instance of ros.communication.Message 
   into a Clojure map (recursively).  If the message is
   already a map, just returns it."
  class)

(defmethod msg-map clojure.lang.IPersistentMap [m] m)


(defmulti map-msg* (fn [t m] t))

(defn map-msg 
  "Convert a Clojure map to a Message (recursively).  The type should
   either be in the :class field, or passed directly.
   If m already a Message, just returns it."
  ([m] (map-msg* (if (map? m) (:class m) (class m)) m))
  ([t m] (map-msg* t m)))


(defmacro defmsg 
  "Inform Clojure that you intend to use a particular message type.
   cls should name the Class object for the desired message type. 
   Calling this is a prerequisite for using any of the above convenience methods.
   You probably want to call defmsgs instead."
  [cls]
  (when-not (@*msgs* cls)
    (println "Declaring message type" cls)
    (swap! *msgs* conj cls)
    (let [g (with-meta (gensym) {:tag cls})  m (gensym)
	types    (message-field-types cls)
	fields   (map symbol (keys types))]
    `(do 

       ~@(for [subtype (message-submessage-types cls)]
	   `(defmsg ~subtype))

       (defmethod map-msg*  ~cls [_# ~m]
	 (if (= (class ~m) ~cls) ~m
  	   (let [~g (new ~cls)]
	     ~@(for [[elt-str type-str] types
		     :let [val (gensym "val")]]
		 `(let [~val (safe-get* ~m ~(keyword elt-str))]
		    (set! (. ~g ~(symbol elt-str))
		      ~(if-let [at (array-type type-str)]
			   `(if (and ~val (.isArray (class ~val))) ~val
			      (let [c# (int (count ~val))
				    a# (make-array ~(or (*primitive-type-map* at)
							(resolve (symbol at)))
						   c#)]
				(loop [i# (int 0) ~val ~val]
				  (if (= i# c#) a#
				    (do (aset a# i# 
					 ~(if-let [st (submsg-type at)]
					      `(map-msg* ~st (first ~val))
					    (if-let [pc (*primitive-coerce-map* at)]
					        `(~pc (first ~val))
					      `(first ~val)))) 
					(recur (inc i#) (next ~val)))))))
			  (if-let [st (submsg-type type-str)]
			      `(map-msg* ~st ~val)
			    (if-let [pc (*primitive-coerce-map* type-str)]
			       `(~pc ~val)
			      val))))))
	     ~g)))

       (defmethod msg-map ~cls [~(with-meta m {:tag cls} )]
	 (hash-map :class ~cls
	   ~@(apply concat 
	      (for [[elt-str type-str] types]
		[(keyword elt-str) 
		 (if-let [at (array-type type-str)]
		     (if-let [st (submsg-type at)]
		         `(map msg-map (. ~m ~(symbol elt-str)))  
		       `(. ~m ~(symbol elt-str)))
		   (if-let [st (submsg-type type-str)]
		       `(msg-map (. ~m ~(symbol elt-str)))
		     `(. ~m ~(symbol elt-str))))]))))
      
       nil))))

; Special case header, to make seq and stamp optional
(defmsg ros.pkg.roslib.msg.Header)
(let [zt (ros.communication.Time. 0 0)]
  (defmethod map-msg* ros.pkg.roslib.msg.Header [_ m]
    (if (instance? ros.pkg.roslib.msg.Header m) m
      (let [ret (ros.pkg.roslib.msg.Header.)]
        (set! (.seq ret)      (get m :seq 0))
        (set! (.stamp ret)    (get m :stamp zt))
	(set! (.frame_id ret) (safe-get* m :frame_id))
	ret))))



(defmacro try-import [cls]
  `(if-let [x# ((ns-map *ns*) '~(symbol (last (.split (name cls) "\\."))))]
     (when-not (= x# ~cls) 
       (println "Cannot import" ~cls "due to name conflict."))
     (import ~cls)))

(defmacro import-msg [cls]
  `(do (try-import ~cls)
       ~@(for [sub (message-submessage-types cls)]
	   `(import-msg ~sub))))

(defmacro defmsgs 
  "Compress a set of calls to defmsg.  Call like 
   (defmsg [pkg msg1 msg2] [pkg msg3] ...).  Also
   imports all Message classes and descendents."
  [& specs]
  `(do ~@(apply concat
	  (for [[pkg & msgs] specs
		msg msgs]
	   (let [cls (symbol (str "ros.pkg." pkg ".msg." msg))]
	     [`(defmsg ~cls)
	      `(import-msg ~cls)]
	      )))))

(defmacro import-all-msgs []
  `(do ~@(for [msg (msg-list)] `(try-import ~msg))))






(def *srvs* (atom #{}))
(defn srv-list [] @*srvs*)

(defmulti srv-req 
  "Take a service Class and return its Request Class."
  identity)

(defmulti srv-res 
  "Take a service Class and return its Response Class."
  identity)

(defmulti req-srv 
  "Take a request Message and return its Service Class."
  class)

(defmacro defsrv 
  "Inform Clojure that you intend to use a particular service type.
   cls should name the Class object for the desired service type. 
   Calling this is a prerequisite for using any of the above convenience methods.
   You probably want to call defsrvs instead."
  [cls]
  (let [req (symbol-cat cls '$Request)
	res (symbol-cat cls '$Response)]
    (when-not (@*srvs* cls)
      (println "Declaring service type" cls)
      (swap! *srvs* conj cls)
    `(do 
       (defmethod srv-req   ~cls [_#] ~req)
       (defmethod req-srv   ~req [_#] (new ~cls))
       (defmethod srv-res   ~cls [_#] ~res)
       (defmsg ~req)
       (defmsg ~res)
       nil))))

(defmacro import-srv [cls]
  `(do (import ~cls)
       (import-msg ~(symbol-cat cls '$Request))
       (import-msg ~(symbol-cat cls '$Response))))

(defmacro defsrvs
  "Compress a set of calls to defsrv.  Call like 
   (defmsg [pkg srv1 srv2] [pkg srv3] ...).  Also
   imports all Request and Response Messages and 
   descendents."
  [& specs]
  `(do ~@(apply concat
	  (for [[pkg & srvs] specs
		srv srvs]
	    (let [cls (symbol (str "ros.pkg." pkg ".srv." srv))]
	      [`(defsrv ~cls)
	       `(import-srv ~cls)])))))

(defmacro import-all-srvs []
  `(do ~@(for [msg (srv-list)] `(import ~msg))))

(defmacro import-all-msgs-and-srvs []
  `(do (import-all-msgs) 
       (import-all-srvs)
       (println "Imported" (count (msg-list)) "message types and" 
	  	           (count (srv-list)) "service types.")))


	
    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                   Convenience wrappers for callbacks
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defn make-subscriber-callback 
  "Wrap a Clojure function into a Subscriber callback"
  [f]
  (proxy [Subscriber$Callback] [] (call [q] (f q))))

(defn make-service-callback 
  "Wrap a Clojure function into a Service callback"
  [f] 
  (proxy [ServiceServer$Callback] [] (call [q] (f q))))

(defmacro sub-cb 
  "Friendly syntax for creating a Subscriber callback:
    (sub-cb msg-class? [arg] & body)
   msg-class is the expected class of the received messages;
   it is optional, and is just used to check that the provided
   messages have the correct type.  In the body of the callback,
   arg will be bound to a Clojure map containing the contents of
   the received Message (translated by msg-map)"     
  [& args]
  (let [m (gensym)
	[cls args] (if (not (coll? (first args))) [(first args) (next args)] [nil args])]
    (assert-is* (= 1 (count (first args))))
    `(make-subscriber-callback
      (fn [~m]
	~@(when cls [`(assert-is* (= (class ~m) ~cls))])
	(let [~(ffirst args) (msg-map ~m)]
	  ~@(next args))))))

(defmacro srv-cb [srv-type binding-form & body]
  "Friendly syntax for creating a Service callback:
    (srv-cb srv-class [arg] & body)
   srv-class is the class of the service for the callback (required). 
   In the body of the callback, arg will be bound to a Clojure map 
   containing the contents of the received Request (translated by msg-map).
   The return value of body will be automatically tranlsated back to a 
   Java Message using map-msg."
  (assert-is* (= 1 (count binding-form)))
  `(let [req# (srv-req ~srv-type)
	 res# (srv-res ~srv-type)]
     (make-service-callback
      (fn [m#] 
	(assert-is* (= (class m#) req#))
	(map-msg
	 (assoc (let [~(first binding-form) (msg-map m#)]	~@body)
	   :class res#))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;            Convenience methods for publications, services, etc.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defn wait-for-subscribers 
  "Wait at most secs seconds for at least subs subscribers on 
   publication pub.  Return true if subscribers are achieved, else
   false."
  [#^NodeHandle nh #^Publisher pub subs secs]
  (let [start-time (.now nh)
	dur        (Duration. (double secs))]
    (loop []
      (Thread/sleep 1)
      (or (>= (.getNumSubscribers pub) subs)
	  (when-not (.hasElapsed start-time dur)
	    (recur))))))

(defn get-message 
  "Pass a topic string and Message subclass/instance of the expected type, 
   and get back a translated (Clojure map) message corresponding
   to the first recieved message on this topic."
  ([#^NodeHandle nh topic message-type]
;     (print "Get single message " topic " took ")
;     (time
     (let [#^Message message-template (if (instance? Class message-type)
					  (.newInstance #^Class message-type)
					message-type)
	   result (atom nil)
	   sub (.subscribe nh topic message-template (sub-cb [m] (reset! result m)) 1)]
       (while (not @result) (.spinOnce nh))
       (.shutdown sub)
       @result)))

(def  #^{:doc "Like get-message, but keeps the subscriber open
               until NodeHandle nh is shutdown, for significant speedups.
               Uses a queue size of 1."
	 :arglists '([nh topic message-type])}
  get-message-cached
  (let [mem (atom {})]
    (fn [#^NodeHandle nh topic message-type]
     (let [#^Message message-template (if (instance? Class message-type)
					  (.newInstance #^Class message-type)
					message-type)
	   result 
	     (or (@mem [nh topic])
		 (let [a (atom nil)
		       sub (.subscribe nh topic message-template 
				       (make-subscriber-callback #(reset! a %))
				       1)]
		   (swap! mem assoc [nh topic] a)
		   a))]
       (while (not @result) (.spinOnce nh))
       (let [ret @result]
	 (reset! result nil)
	 (msg-map ret))))))


(defn put-message 
  "Pass a topic string and Message object/map, which will be published on 
   the desired topic.  Waits up to 5 seconds for at least n-subs subscribers
   before publishing. Returns true if message published, or false if the 
   desired number of subscribers were not obtained within the time limit."
  ([#^NodeHandle nh topic message n-subs]
      (let [#^Message message (map-msg message)
	    pub (.advertise nh topic message 1)]
       (if (or (not n-subs) (zero? n-subs) (wait-for-subscribers nh pub n-subs 5.0))
	   (do (.publish pub message) (.shutdown pub) true)
	 (do (println "No subscribers on" topic) (.shutdown pub) false)))))

(def #^{:doc "Like put-message, but keeps the publisher open
              until NodeHandle nh is shutdown, for significant speedups.
              Waits for one subscriber on first publication, throwing an
              exception if not received, and does not wait at all thereafter."
	 :arglists '([nh topic message])} 
  put-message-cached  
  (let [mem (atom {})]
    (fn [#^NodeHandle nh topic message]
      (let [#^Message message (map-msg message)
	    #^Publisher pub 
	      (or (@mem [nh topic])
		  (let [ret (.advertise nh topic message 10)]
		    (assert (wait-for-subscribers nh ret 1 2.0))
		    (swap! mem assoc [nh topic] ret)
		    ret))]
	(.publish pub message)))))
       
  

(defn call-service 
  "Call service 'name' with the given Request map, and return a Clojure
   map corresponding to the response."
  ([#^NodeHandle nh #^String name request]
     (let [#^Message request (map-msg request)
	   srv (.serviceClient nh name #^Service (req-srv request) false)
	   result (msg-map (.call srv (map-msg request)))]
       (.shutdown srv)
       result)))


(def  #^{:doc "Like call-service, but creates a persistent service that is
              kept alive until NodeHandle nh is shutdown, for significant speedups."
	 :arglists '([nh service-name request])} 
  call-service-cached   
  (let [mem (atom {})]
    (fn [#^NodeHandle nh #^String name request]
      (let [#^Message request (map-msg request)
	    #^ServiceClient srv 
	      (or (@mem [nh name])
		  (let [ret (.serviceClient nh name #^Service 
					    (req-srv request) true)]
		    (swap! mem assoc [nh name] ret)
		    ret))]
	(msg-map (.call srv (map-msg request)))))))






(set! *warn-on-reflection* false)





