;; Auto-generated. Do not edit!


(when (boundp 'sp_common::ActuatorState)
  (if (not (find-package "SP_COMMON"))
    (make-package "SP_COMMON"))
  (shadow 'ActuatorState (find-package "SP_COMMON")))
(unless (find-package "SP_COMMON::ACTUATORSTATE")
  (make-package "SP_COMMON::ACTUATORSTATE"))

(in-package "ROS")
;;//! \htmlinclude ActuatorState.msg.html


(defclass sp_common::ActuatorState
  :super ros::object
  :slots (_stamp _name _type _bus _id _halted _position_raw _velocity_raw _circle _last_position_raw _position _velocity _effort _cmd_effort _exe_effort ))

(defmethod sp_common::ActuatorState
  (:init
   (&key
    ((:stamp __stamp) (let (r) (dotimes (i 0) (push (instance ros::time :init) r)) r))
    ((:name __name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:type __type) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:bus __bus) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:id __id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:halted __halted) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:position_raw __position_raw) (make-array 0 :initial-element 0 :element-type :integer))
    ((:velocity_raw __velocity_raw) (make-array 0 :initial-element 0 :element-type :integer))
    ((:circle __circle) (make-array 0 :initial-element 0 :element-type :integer))
    ((:last_position_raw __last_position_raw) (make-array 0 :initial-element 0 :element-type :integer))
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:velocity __velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:effort __effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cmd_effort __cmd_effort) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:exe_effort __exe_effort) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _stamp __stamp)
   (setq _name __name)
   (setq _type __type)
   (setq _bus __bus)
   (setq _id __id)
   (setq _halted __halted)
   (setq _position_raw __position_raw)
   (setq _velocity_raw __velocity_raw)
   (setq _circle __circle)
   (setq _last_position_raw __last_position_raw)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _effort __effort)
   (setq _cmd_effort __cmd_effort)
   (setq _exe_effort __exe_effort)
   self)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:bus
   (&optional __bus)
   (if __bus (setq _bus __bus)) _bus)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:halted
   (&optional __halted)
   (if __halted (setq _halted __halted)) _halted)
  (:position_raw
   (&optional __position_raw)
   (if __position_raw (setq _position_raw __position_raw)) _position_raw)
  (:velocity_raw
   (&optional __velocity_raw)
   (if __velocity_raw (setq _velocity_raw __velocity_raw)) _velocity_raw)
  (:circle
   (&optional __circle)
   (if __circle (setq _circle __circle)) _circle)
  (:last_position_raw
   (&optional __last_position_raw)
   (if __last_position_raw (setq _last_position_raw __last_position_raw)) _last_position_raw)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:effort
   (&optional __effort)
   (if __effort (setq _effort __effort)) _effort)
  (:cmd_effort
   (&optional __cmd_effort)
   (if __cmd_effort (setq _cmd_effort __cmd_effort)) _cmd_effort)
  (:exe_effort
   (&optional __exe_effort)
   (if __exe_effort (setq _exe_effort __exe_effort)) _exe_effort)
  (:serialization-length
   ()
   (+
    ;; time[] _stamp
    (* 8    (length _stamp)) 4
    ;; string[] _name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _name)) 4
    ;; string[] _type
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _type)) 4
    ;; string[] _bus
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _bus)) 4
    ;; int32[] _id
    (* 4    (length _id)) 4
    ;; bool[] _halted
    (* 1    (length _halted)) 4
    ;; uint16[] _position_raw
    (* 2    (length _position_raw)) 4
    ;; int16[] _velocity_raw
    (* 2    (length _velocity_raw)) 4
    ;; int64[] _circle
    (* 8    (length _circle)) 4
    ;; uint16[] _last_position_raw
    (* 2    (length _last_position_raw)) 4
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; float64[] _velocity
    (* 8    (length _velocity)) 4
    ;; float64[] _effort
    (* 8    (length _effort)) 4
    ;; float64[] _cmd_effort
    (* 8    (length _cmd_effort)) 4
    ;; float64[] _exe_effort
    (* 8    (length _exe_effort)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; time[] _stamp
     (write-long (length _stamp) s)
     (dotimes (i (length _stamp))
       (write-long (send (elt _stamp i) :sec) s) (write-long (send (elt _stamp i) :nsec) s)
       )
     ;; string[] _name
     (write-long (length _name) s)
     (dolist (elem _name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _type
     (write-long (length _type) s)
     (dolist (elem _type)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _bus
     (write-long (length _bus) s)
     (dolist (elem _bus)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int32[] _id
     (write-long (length _id) s)
     (dotimes (i (length _id))
       (write-long (elt _id i) s)
       )
     ;; bool[] _halted
     (write-long (length _halted) s)
     (dotimes (i (length _halted))
       (if (elt _halted i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; uint16[] _position_raw
     (write-long (length _position_raw) s)
     (dotimes (i (length _position_raw))
       (write-word (elt _position_raw i) s)
       )
     ;; int16[] _velocity_raw
     (write-long (length _velocity_raw) s)
     (dotimes (i (length _velocity_raw))
       (write-word (elt _velocity_raw i) s)
       )
     ;; int64[] _circle
     (write-long (length _circle) s)
     (dotimes (i (length _circle))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _circle i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _circle i)) (= (length ((elt _circle i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _circle i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _circle i) . bv) 1) -1) s))
             ((and (class (elt _circle i)) (= (length ((elt _circle i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _circle i) . bv) 0) s)
              (write-long (if (>= (elt _circle i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _circle i) s)(write-long (if (>= (elt _circle i) 0) 0 #xffffffff) s)))
       )
     ;; uint16[] _last_position_raw
     (write-long (length _last_position_raw) s)
     (dotimes (i (length _last_position_raw))
       (write-word (elt _last_position_raw i) s)
       )
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _velocity
     (write-long (length _velocity) s)
     (dotimes (i (length _velocity))
       (sys::poke (elt _velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _effort
     (write-long (length _effort) s)
     (dotimes (i (length _effort))
       (sys::poke (elt _effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cmd_effort
     (write-long (length _cmd_effort) s)
     (dotimes (i (length _cmd_effort))
       (sys::poke (elt _cmd_effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _exe_effort
     (write-long (length _exe_effort) s)
     (dotimes (i (length _exe_effort))
       (sys::poke (elt _exe_effort i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; time[] _stamp
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stamp (let (r) (dotimes (i n) (push (instance ros::time :init) r)) r))
     (dolist (elem- _stamp)
     (send elem- :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send elem- :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; string[] _name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _type (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _type i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _bus
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bus (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _bus i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int32[] _id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; bool[] _halted
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _halted (make-list n))
     (dotimes (i n)
     (setf (elt _halted i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; uint16[] _position_raw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position_raw (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _position_raw i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int16[] _velocity_raw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity_raw (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _velocity_raw i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; int64[] _circle
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _circle (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _circle i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _circle i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; uint16[] _last_position_raw
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _last_position_raw (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _last_position_raw i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     ))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cmd_effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cmd_effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cmd_effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _exe_effort
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _exe_effort (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _exe_effort i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get sp_common::ActuatorState :md5sum-) "40a5a2d346a26f8d3a3f5c992d8174b2")
(setf (get sp_common::ActuatorState :datatype-) "sp_common/ActuatorState")
(setf (get sp_common::ActuatorState :definition-)
      "# This message is for visualizing the ActuatorState
# The data from real Actuator will be parsed by sp_hw
# notice that this is a vector of ActuatorState

time[] stamp
string[] name
string[] type
string[] bus
int32[] id
bool[] halted

# the direct feedback from rotor's encoder
uint16[] position_raw
int16[] velocity_raw
int64[] circle
uint16[] last_position_raw

float64[] position
float64[] velocity
float64[] effort


float64[] cmd_effort 
float64[] exe_effort

")



(provide :sp_common/ActuatorState "40a5a2d346a26f8d3a3f5c992d8174b2")


