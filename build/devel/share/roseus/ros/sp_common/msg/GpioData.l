;; Auto-generated. Do not edit!


(when (boundp 'sp_common::GpioData)
  (if (not (find-package "SP_COMMON"))
    (make-package "SP_COMMON"))
  (shadow 'GpioData (find-package "SP_COMMON")))
(unless (find-package "SP_COMMON::GPIODATA")
  (make-package "SP_COMMON::GPIODATA"))

(in-package "ROS")
;;//! \htmlinclude GpioData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass sp_common::GpioData
  :super ros::object
  :slots (_bus _gpio_name _gpio_state _gpio_type _header ))

(defmethod sp_common::GpioData
  (:init
   (&key
    ((:bus __bus) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:gpio_name __gpio_name) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:gpio_state __gpio_state) (let (r) (dotimes (i 0) (push nil r)) r))
    ((:gpio_type __gpio_type) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:header __header) (instance std_msgs::Header :init))
    )
   (send-super :init)
   (setq _bus __bus)
   (setq _gpio_name __gpio_name)
   (setq _gpio_state __gpio_state)
   (setq _gpio_type __gpio_type)
   (setq _header __header)
   self)
  (:bus
   (&optional __bus)
   (if __bus (setq _bus __bus)) _bus)
  (:gpio_name
   (&optional __gpio_name)
   (if __gpio_name (setq _gpio_name __gpio_name)) _gpio_name)
  (:gpio_state
   (&optional __gpio_state)
   (if __gpio_state (setq _gpio_state __gpio_state)) _gpio_state)
  (:gpio_type
   (&optional __gpio_type)
   (if __gpio_type (setq _gpio_type __gpio_type)) _gpio_type)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serialization-length
   ()
   (+
    ;; string[] _bus
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _bus)) 4
    ;; string[] _gpio_name
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _gpio_name)) 4
    ;; bool[] _gpio_state
    (* 1    (length _gpio_state)) 4
    ;; string[] _gpio_type
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _gpio_type)) 4
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _bus
     (write-long (length _bus) s)
     (dolist (elem _bus)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _gpio_name
     (write-long (length _gpio_name) s)
     (dolist (elem _gpio_name)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool[] _gpio_state
     (write-long (length _gpio_state) s)
     (dotimes (i (length _gpio_state))
       (if (elt _gpio_state i) (write-byte -1 s) (write-byte 0 s))
       )
     ;; string[] _gpio_type
     (write-long (length _gpio_type) s)
     (dolist (elem _gpio_type)
       (write-long (length elem) s) (princ elem s)
       )
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _bus
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bus (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _bus i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _gpio_name
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gpio_name (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _gpio_name i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool[] _gpio_state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gpio_state (make-list n))
     (dotimes (i n)
     (setf (elt _gpio_state i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;; string[] _gpio_type
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _gpio_type (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _gpio_type i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;;
   self)
  )

(setf (get sp_common::GpioData :md5sum-) "297858866c40f438698ae2b119d8be46")
(setf (get sp_common::GpioData :datatype-) "sp_common/GpioData")
(setf (get sp_common::GpioData :definition-)
      "string[] bus
string[] gpio_name
bool[] gpio_state
string[] gpio_type
std_msgs/Header header
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :sp_common/GpioData "297858866c40f438698ae2b119d8be46")


