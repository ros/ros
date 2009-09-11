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
;; lead to un-ideomatic code, i.e., proxying callbacks and 
;; mutable Message types.

; Main things you want to call or access here:
;
; (import-ros) to import ROS Java classes into your namespace
; *ros* is the global Ros instance
; with-node-handle creates a node handle that is guaranteed 
;   to be destroyed when the scope is exited (exceptionally or normally).

; defmsgs and defsrvs declare the set of messages and services 
;   that you want Clojure support for (above and beyond what you
;   already get from Java).  In particular, automated conversion
;   between Clojure maps and Messages is supported for declared types. 

; sub-cb and srv-cb are convenient wrappers for Subscription and Service
;   callbacks.  make-subscriber-callback and make-service-callback are 
;   lower-level, if you want to avoid conversion to Clojure maps.

; wait-for-subscribers, get-single-message, put-single-message, and 
; call-srv do more or less what they say.  

; Finally, test-ros tests and illustrates most of this core functionality

; (add-classpath "file:///Users/jawolfe/Projects/ros/ros/core/experimental/rosjava/src")
; (add-classpath "file:///Users/jawolfe/Projects/ros/ros/core/experimental/rosclj/src")

(ns ros.ros
  )

(set! *warn-on-reflection* true)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                          Misc helper functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defn import-as [sym class]
  "Import class under psudonym named by symbol sym"
  (let [#^clojure.lang.Namespace ns *ns*]
    (.importClass ns sym class)))

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
				": Got " '~form " as " (cons '~(first form) (list ~@(next form)))))))))

(defn read-reader
  "Read the complete contents of this reader into a String."
  [#^java.io.Reader r]
  (let [sb (StringBuilder.)]
    (loop []
      (let [c (.read r)]
	(if (neg? c)
	    (str sb)
	  (do (.append sb (char c))
	      (recur)))))))

(defn shell 
  "Call shell command with args, and return a string representation
   of its standard output."
  [& args]
  (read-reader
   (java.io.BufferedReader.
    (java.io.InputStreamReader.
     (.getInputStream
      (doto (.exec (Runtime/getRuntime) #^"[Ljava.lang.String;" (into-array String (map str args)))
	(.waitFor)))))))



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

(defonce #^Ros *ros* (doto (Ros/getInstance) (.init "rosclj")))

(defmacro with-node-handle 
  "Create a node handle with arguments provided by args, bind it 
   to nh-var in the execution of body, and destroy it upon exiting
   this form (exceptionally or normally."
  [[nh-var & args] & body]
  `(let [~nh-var (.createNodeHandle *ros* ~@args)]
     (Thread/sleep 100) ; Allow time to get time, ect ???
     (try (do ~@body) (finally (.shutdown ~nh-var)))))
;  nil)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                   Core handling of Messages and Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defn- ros-ensure-classpath
  "Ensure that a given message/service named by class-symbol is 
   available on the classpath.  Assumes that this has already 
   been called on all submessages."
  [class-symbol]
;  (println class-symbol)
  (try (Class/forName (str class-symbol))
       (catch java.lang.ClassNotFoundException e 
	  (let [sections (.split #^String (name class-symbol) "\\.")]
	    (assert (= (take 2 sections) ["ros" "pkg"]))
	    (assert (= (count sections) 5))
	    (assert (#{"srv" "msg"} (nth sections 3)))
	    (let [pkg-name (nth sections 2)
		  type     (nth sections 3)
		  pkg-dir  (shell "rospack" "find" pkg-name)
		  cp       (str "file://" (apply str (butlast pkg-dir)) "/" type "/java/")]
	      (assert-is* (= (first pkg-dir) \/))
	      (println "Adding classpath" cp)
	      (add-classpath cp)
	      (Class/forName (str class-symbol)))))))



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
  "Convert a Clojure map to a message.  The type should
   either be in the :class field, or passed directly.
   If already a message, no work is done."
  ([m] (map-msg* (if (map? m) (:class m) (class m)) m))
  ([t m] (map-msg* t m)))


(defmacro defmsg 
  "Inform Clojure that you intend to use a particular message type.
   cls should name the Class object for the desired message type. 
   Calling this is a prerequisite for using any of the above convenience methods."
  [cls]
  (ros-ensure-classpath cls)
  (when-not (@*msgs* cls)
    (println "Defining message " cls)
    (swap! *msgs* conj cls)
    (let [g (with-meta (gensym) {:tag cls})  m (gensym)
	types    (message-field-types cls)
	fields   (map symbol (keys types))]
      (doseq [subtype (message-submessage-types cls)]
	(ros-ensure-classpath subtype))
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
			   `(if (.isArray (class ~val)) ~val
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


(defmacro import-msg [cls]
  `(do (import ~cls)
       ~@(for [sub (message-submessage-types cls)]
	   `(import-msg ~sub))))

(defmacro defmsgs 
  "Compress a set of calls to defmsg.  Call like 
   (defmsg [pkg msg1 msg2] [pkg msg3] ...).  Also
   imports all message classes and descendents."
  [& specs]
  `(do ~@(apply concat
	  (for [[pkg & msgs] specs
		msg msgs]
	   (let [cls (symbol (str "ros.pkg." pkg ".msg." msg))]
	     [`(defmsg ~cls)
	      `(import-msg ~cls)]
	      )))))

(defmacro import-all-msgs []
  `(do ~@(for [msg (msg-list)] `(import ~msg))))






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
   Calling this is a prerequisite for using any of the above convenience methods."
  [cls]
  (ros-ensure-classpath cls)
  (let [req (symbol-cat cls '$Request)
	res (symbol-cat cls '$Response)]
    (when-not (@*srvs* cls)
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
   (defmsg [pkg srv1 srv2] [pkg srv3] ...)"
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
  `(do (import-all-msgs) (import-all-srvs)))


	
    



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

(defn get-single-message 
  "Pass a topic string and Message object of the expected type, 
   and get back a translated (Clojure map) message corresponding
   to the first recieved message on this topic.  Call with an 
   existing node handle for efficiency, or without one for convenience."
  ([topic message-template]
     (with-node-handle [nh] 
       (get-single-message nh topic message-template)))
  ([#^NodeHandle nh topic message-template]
;     (print "Get single message " topic " took ")
;     (time
     (let [result (atom nil)
	   sub (.subscribe nh topic message-template (sub-cb [m] (reset! result m)) 1)]
       (while (not @result) (.spinOnce nh))
       (.shutdown sub)
       @result)));)


(defn put-single-message 
  "Pass a topic string and Message object, which will be published on 
   the desired topic. By default, waits up to 2 seconds for at least 1 subscriber 
   before publishing; pass n-subs (nil if desired) to override this behavior."
  ([topic message] (put-single-message topic message 1))
  ([topic message n-subs]
     (with-node-handle [nh] 
       (put-single-message nh topic message n-subs)))
  ([#^NodeHandle nh topic message n-subs]
;     (print "Put single message " topic " took ")
;     (time 
      (let [pub (.advertise nh topic message 1)]
       (if (or (not n-subs) (zero? n-subs) (time (wait-for-subscribers nh pub n-subs 2.0)))
	   (do (.publish pub message) (.shutdown pub) true)
	 (do (println "No subscribers on" topic) (.shutdown pub) false)))));)

(def put-single-message-cached
;  "Like put-single-message, but caches publishers for big speedups.
;   Assumes single subscriber, only waits on first pub."
  (let [mem (atom {})]
    (fn [#^NodeHandle nh topic #^Message message]
;      (print "Putting message on cached topic " topic " took ")
;      (time
      (let [#^Publisher pub 
	      (or (@mem [nh topic])
		  (let [ret (.advertise nh topic message 10)]
		    (assert (wait-for-subscribers nh ret 1 2.0))
		    (swap! mem assoc [nh topic] ret)
		    ret))]
	(.publish pub message)))));)
       
  

(defn call-srv 
  "Call service 'name' with the given Request object, and return a Clojure
   map corresponding to the response.  (TODO: make uniform?)"
  ([name request]
     (with-node-handle [nh]
       (call-srv nh name request)))
  ([#^NodeHandle nh #^String name #^Message request]
;     (print "Calling service " name " took ") 
;     (time
     (let [srv (.serviceClient nh name #^Service (req-srv request) false)
	   result (msg-map (.call srv request))]
       (.shutdown srv)
       result)));)


(def call-srv-cached
;  "Like put-single-message, but caches publishers for big speedups.
;   Assumes single subscriber, only waits on first pub."
  (let [mem (atom {})]
    (fn [#^NodeHandle nh #^String topic #^Message request]
;      (print "calling cached service " topic " took ")
;      (time
      (let [#^ServiceClient srv 
	      (or (@mem [nh topic])
		  (let [ret (.serviceClient nh topic #^Service 
					    (req-srv request) true)]
		    (swap! mem assoc [nh topic] ret)
		    ret))]
	(msg-map (.call srv request))))));)







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                  Test/illustrate some of the above features.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(comment

(defmsgs [ros.pkg.std_msgs.msg [String :as mString]])
(defsrvs [ros.pkg.rosjava_test.srv TwoInts])


(defn test-ros []
  (with-node-handle [nh] 
    (doto nh 
      (.setParam "testi" 2)
      (.setParam "testd" 2.1)
      (.setParam "tests" "2.5"))
    (println (.getIntParam nh "testi")
	     (.getDoubleParam nh "testd")
	     (.getStringParam nh "tests"))
    (println (.now nh))
    (let [sc (.serviceClient nh "add_two_ints" (TwoInts.) false)
	  req (map-msg {:class TwoInts$Request :a 5 :b 7})]
      (print "expect yes: ") (try (.call sc req) (catch Exception e (println "yes" e)))
      (let [ss (.advertiseService nh "add_two_ints" (TwoInts.) 
				  (srv-cb TwoInts [{:keys [a b]}] {:sum (+ a b)}))]
	(println "5+7=" (:sum (msg-map (.callLocal sc req))))))
    (let [pub (.advertise nh "/chatter" (mString.) 100)
	  q (Subscriber$QueueingCallback.)
	  sub (.subscribe nh "/chatter" (mString.) q 10)
	  q2  (sub-cb mString [m] (println "direct" (:data m)))
	  sub2 (.subscribe nh "/chatter" (mString.) q2 10)]
      (println (.getTopics nh) 
	       (.getAdvertisedTopics nh) 
	       (.getSubscribedTopics nh))
      (dotimes [i 20]
	(.publish pub (map-msg {:class mString :data (str 'hola i)}))
	(when (= i 17) (.shutdown sub))
	(while (not (.isEmpty q)) (println "queued" (:data (msg-map (.pop q)))))
	(Thread/sleep 100)
	(.spinOnce nh)))))
	    
)




(set! *warn-on-reflection* false)





