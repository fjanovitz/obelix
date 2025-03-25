
(cl:in-package :asdf)

(defsystem "drivers-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :trajectory_msgs-msg
)
  :components ((:file "_package")
    (:file "Camera" :depends-on ("_package_Camera"))
    (:file "_package_Camera" :depends-on ("_package"))
    (:file "Servo" :depends-on ("_package_Servo"))
    (:file "_package_Servo" :depends-on ("_package"))
  ))