;; Auto-generated. Do not edit!


(when (boundp 'sp_common::SingleJointWrite)
  (if (not (find-package "SP_COMMON"))
    (make-package "SP_COMMON"))
  (shadow 'SingleJointWrite (find-package "SP_COMMON")))
(unless (find-package "SP_COMMON::SINGLEJOINTWRITE")
  (make-package "SP_COMMON::SINGLEJOINTWRITE"))

(in-package "ROS")
;;//! \htmlinclude SingleJointWrite.msg.html


(defclass sp_common::SingleJointWrite
  :super ros::object
  :slots (_state _num ))

(defmethod sp_common::SingleJointWrite
  (:init
   (&key
    ((:state __state) 0.0)
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _state (float __state))
   (setq _num (round __num))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; float64 _state
    8
    ;; uint16 _num
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _state
       (sys::poke _state (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint16 _num
       (write-word _num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _state
     (setq _state (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint16 _num
     (setq _num (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get sp_common::SingleJointWrite :md5sum-) "a8df87d7f5c9d7fe4d040df6e38c801b")
(setf (get sp_common::SingleJointWrite :datatype-) "sp_common/SingleJointWrite")
(setf (get sp_common::SingleJointWrite :definition-)
      "# This message is for....

float64 state
uint16 num

")



(provide :sp_common/SingleJointWrite "a8df87d7f5c9d7fe4d040df6e38c801b")


