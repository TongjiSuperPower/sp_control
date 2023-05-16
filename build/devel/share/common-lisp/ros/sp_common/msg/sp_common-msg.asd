
(cl:in-package :asdf)

(defsystem "sp_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ActuatorState" :depends-on ("_package_ActuatorState"))
    (:file "_package_ActuatorState" :depends-on ("_package"))
    (:file "GpioData" :depends-on ("_package_GpioData"))
    (:file "_package_GpioData" :depends-on ("_package"))
    (:file "SingleJointWrite" :depends-on ("_package_SingleJointWrite"))
    (:file "_package_SingleJointWrite" :depends-on ("_package"))
  ))