;///////////////////////////////////////////////////////////////////////////////
;// The roscpp package provides a thin Clojure wrapper for rosjava
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



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                   Core handling of Messages and Services
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; TODO: extract element types from messages, auto-define depended-on messages,
 ; stuff types in map-msg. 

(def *msgs* (atom #{}))
(defn msg-list [] @*msgs*)

(defmulti msg-map 
  "Convert an instance of ros.communication.Message 
   into a Clojure map (recursively)"
  class)

(defmulti map-msg 
  "Convert a Clojure map into an instance of ros.communication.Message
   (recursively), using the :class value to determine the desired type."
  :class)


(def *srvs* (atom #{}))
(defn srv-list [] @*srvs*)

(defmulti srv-req 
  "Take a service Class and return its Request Class."
  identity)

(defmulti srv-res 
  "Take a service Class and return its Response Class."
  identity
  )

(defmulti req-srv 
  "Take a request Message and return its Service Class."
  class)


(defn msg-map-helper 
  "Helper for recursively converting Messages into maps.  Do not call directly."
  [val]
  (cond (instance? ros.communication.Message val)
	  (msg-map val)
	(instance? java.util.Collection val)
	  (cond (empty? val) nil
		(instance? ros.communication.Message (first val)) (map msg-map-helper val)
		:else val)
	:else 
          val))

(defn map-msg-helper 
  "Helper for recursively converting maps into Messages.  Do not call directly."
  [val] 
  (cond (map? val)
	  (map-msg val)
	(instance? java.util.Collection val)
	  (map map-msg-helper val)
	:else 
	  val))



(defn list-message-fields [class-symbol]
  "List the field names for a Message subclass."
  (map symbol (keys (.invoke (.getMethod (Class/forName (str class-symbol)) 
				   "fieldTypes" (make-array Class 0)) 
		       nil (make-array Object 0)))))


(defn- ros-ensure-classpath
  "Ensure that a given message/service named by class-symbol is 
   available on the classpath.  Assumes that this has already 
   been called on all submessages."
  [class-symbol]
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
	      (assert (= (first pkg-dir) \/))
	      (println "Adding classpath" cp)
	      (add-classpath cp)
	      (Class/forName (str class-symbol)))))))


(defmacro defmsg 
  "Inform Clojure that you intend to use a particular message type.
   cls should name the Class object for the desired message type. 
   Calling this is a prerequisite for using any of the above convenience methods."
  [cls]
  (ros-ensure-classpath cls)
  (let [g (with-meta (gensym) {:tag cls})  m (gensym)
	elements (list-message-fields cls)]
    (swap! *msgs* conj cls)
    `(do 
       (defmethod map-msg  ~cls [~m]
	 (let [~g (new ~cls)]
	   ~@(map (fn [elt] 
		    (if (or (= cls 'ros.pkg.roslib.msg.Header) (= elt 'header))
		        `(let [x# (find ~m ~(keyword (str elt)))]
			   (when x# (set! (. ~g ~elt) (map-msg-helper (val x#)))))
		      `(set! (. ~g ~elt) (map-msg-helper (safe-get* ~m ~(keyword (str elt)))))))
		  elements)
	   ~g))
       (defmethod msg-map ~cls [ ~g]
	 (into {}
	       [~@(cons [:class cls]
		      (map (fn [elt]
			     [(keyword (str elt)) `(msg-map-helper (. ~g ~elt))])
			   elements))]))
       nil)))

(defmacro defsrv 
  "Inform Clojure that you intend to use a particular service type.
   cls should name the Class object for the desired service type. 
   Calling this is a prerequisite for using any of the above convenience methods."
  [cls]
  (ros-ensure-classpath cls)
  (swap! *srvs* conj cls)
  (let [req (symbol-cat cls '$Request)
	res (symbol-cat cls '$Response)]
    `(do 
       (defmethod srv-req   ~cls [_#] ~req)
       (defmethod req-srv   ~req [_#] (new ~cls))
       (defmethod srv-res   ~cls [_#] ~res)
       (defmsg ~req)
       (defmsg ~res)
       nil)))




(defn defmsgs* 
  "Helper function for defining a set of messages. 
   You should not need to call this directly."
  [prefix-str spec] 
  (cond (not (coll? spec))
	  [`(defmsg ~(symbol-cat prefix-str "." spec))
	   `(import '[~(symbol prefix-str) ~spec])] 
	(keyword? (second spec))
	  (let [cls (symbol-cat prefix-str "." (first spec))
		alias (nth spec 2)]
	    (assert-is* (and (= (second spec) :as) (= (count spec) 3)))
	    (concat [`(defmsg ~cls)]
		    (when alias [`(import-as '~alias ~cls)])))
	:else  
	(let [new-prefix (str prefix-str (when (> (count prefix-str) 0) ".") (first spec))]
	  (assert-is* (> (count spec) 1))
	  (assert-is* (symbol? (first spec)))
	  (mapcat #(defmsgs* new-prefix %) (rest spec)))))

(defmacro defmsgs 
  "Take a libspec-like definition of message-files, with optional aliases,
   and emit the appropriate defmsg statements.  E.g., 
   (defmsgs [my-pkg.msg Test [String :as ::mString] [submsg Blue Green]]).
   By default, classes are mapped to their short names like import, unless
   renamed by :as; an argument of nil to :as means don't import."
  [& specs]
  `(do ~@(mapcat #(defmsgs* "" % ) specs)))



(defn defsrvs* [prefix-str spec] 
  "Helper function for defining a set of services. 
   You should not need to call this directly."
  (cond (not (coll? spec))
	  [`(defsrv ~(symbol-cat prefix-str "." spec))
	   `(import '[~(symbol prefix-str) ~spec ~(symbol-cat spec '$Request) 
		                                 ~(symbol-cat spec '$Response)])
	   ] 
	(keyword? (second spec))
	  (let [cls   (symbol-cat prefix-str "." (first spec))
		alias (nth spec 2)]
	    (assert-is* (and (= (second spec) :as) (= (count spec) 3)))
	    (concat [`(defsrv ~cls)] 
		    (when alias 
		      [`(import-as '~alias ~cls)
		       `(import-as '~(symbol-cat alias '$Request) ~(symbol-cat cls '$Request))
		       `(import-as '~(symbol-cat alias '$Response) ~(symbol-cat cls '$Response))])
		    ))
	:else  
	(let [new-prefix (str prefix-str (when (> (count prefix-str) 0) ".") (first spec))]
	  (assert-is* (> (count spec) 1))
	  (assert-is* (symbol? (first spec)))
	  (mapcat #(defsrvs* new-prefix %) (rest spec)))))


(defmacro defsrvs 
  "Like defmsgs, but for services."
  [& specs]
  `(do ~@(mapcat #(defsrvs* "" % ) specs)))




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
     (let [result (atom nil)]
       (.subscribe nh topic message-template (sub-cb [m] (reset! result m)) 1)
       (while (not @result) (.spinOnce nh))
       @result)))


(defn put-single-message 
  "Pass a topic string and Message object, which will be published on 
   the desired topic. By default, waits up to 2 seconds for at least 1 subscriber 
   before publishing; pass n-subs (nil if desired) to override this behavior."
  ([topic message] (put-single-message topic message 1))
  ([topic message n-subs]
     (with-node-handle [nh] 
       (put-single-message nh topic message n-subs)))
  ([#^NodeHandle nh topic message n-subs]
     (let [pub (.advertise nh topic message 1)]
       (if (or (not n-subs) (zero? n-subs) (wait-for-subscribers nh pub n-subs 2.0))
	   (do (.publish pub message) true)
	 (do (println "No subscribers on" topic) false)))))


(defn call-srv 
  "Call service 'name' with the given Request object, and return a Clojure
   map corresponding to the response.  (TODO: make uniform?)"
  ([name request]
     (with-node-handle [nh]
       (call-srv nh name request)))
  ([#^NodeHandle nh #^String name #^Message request]
     (let [srv (.serviceClient nh name #^Service (req-srv request) false)
	   result (msg-map (.call srv request))]
       (.shutdown srv)
       result)))







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;                  Test/illustrate some of the above features.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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
      (println (.getPublishedTopics nh) 
	       (.getAdvertisedTopics nh) 
	       (.getSubscribedTopics nh))
      (dotimes [i 20]
	(.publish pub (map-msg {:class mString :data (str 'hola i)}))
	(when (= i 17) (.shutdown sub))
	(while (not (.isEmpty q)) (println "queued" (:data (msg-map (.pop q)))))
	(Thread/sleep 100)
	(.spinOnce nh)))))
	    





(set! *warn-on-reflection* false)