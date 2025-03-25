;; Auto-generated. Do not edit!


(when (boundp 'drivers::Servo)
  (if (not (find-package "DRIVERS"))
    (make-package "DRIVERS"))
  (shadow 'Servo (find-package "DRIVERS")))
(unless (find-package "DRIVERS::SERVO")
  (make-package "DRIVERS::SERVO"))
(unless (find-package "DRIVERS::SERVOREQUEST")
  (make-package "DRIVERS::SERVOREQUEST"))
(unless (find-package "DRIVERS::SERVORESPONSE")
  (make-package "DRIVERS::SERVORESPONSE"))

(in-package "ROS")

(if (not (find-package "TRAJECTORY_MSGS"))
  (ros::roseus-add-msgs "trajectory_msgs"))




(defclass drivers::ServoRequest
  :super ros::object
  :slots (_trajectory ))

(defmethod drivers::ServoRequest
  (:init
   (&key
    ((:trajectory __trajectory) (instance trajectory_msgs::JointTrajectory :init))
    )
   (send-super :init)
   (setq _trajectory __trajectory)
   self)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; trajectory_msgs/JointTrajectory _trajectory
    (send _trajectory :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; trajectory_msgs/JointTrajectory _trajectory
       (send _trajectory :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; trajectory_msgs/JointTrajectory _trajectory
     (send _trajectory :deserialize buf ptr-) (incf ptr- (send _trajectory :serialization-length))
   ;;
   self)
  )

(defclass drivers::ServoResponse
  :super ros::object
  :slots ())

(defmethod drivers::ServoResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass drivers::Servo
  :super ros::object
  :slots ())

(setf (get drivers::Servo :md5sum-) "2a0eff76c870e8595636c2a562ca298e")
(setf (get drivers::Servo :datatype-) "drivers/Servo")
(setf (get drivers::Servo :request) drivers::ServoRequest)
(setf (get drivers::Servo :response) drivers::ServoResponse)

(defmethod drivers::ServoRequest
  (:response () (instance drivers::ServoResponse :init)))

(setf (get drivers::ServoRequest :md5sum-) "2a0eff76c870e8595636c2a562ca298e")
(setf (get drivers::ServoRequest :datatype-) "drivers/ServoRequest")
(setf (get drivers::ServoRequest :definition-)
      "trajectory_msgs/JointTrajectory trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

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

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
")

(setf (get drivers::ServoResponse :md5sum-) "2a0eff76c870e8595636c2a562ca298e")
(setf (get drivers::ServoResponse :datatype-) "drivers/ServoResponse")
(setf (get drivers::ServoResponse :definition-)
      "trajectory_msgs/JointTrajectory trajectory

================================================================================
MSG: trajectory_msgs/JointTrajectory
Header header
string[] joint_names
JointTrajectoryPoint[] points

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

================================================================================
MSG: trajectory_msgs/JointTrajectoryPoint
# Each trajectory point specifies either positions[, velocities[, accelerations]]
# or positions[, effort] for the trajectory to be executed.
# All specified values are in the same order as the joint names in JointTrajectory.msg

float64[] positions
float64[] velocities
float64[] accelerations
float64[] effort
duration time_from_start
---
")



(provide :drivers/Servo "2a0eff76c870e8595636c2a562ca298e")


