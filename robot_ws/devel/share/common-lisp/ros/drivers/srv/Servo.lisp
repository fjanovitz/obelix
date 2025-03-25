; Auto-generated. Do not edit!


(cl:in-package drivers-srv)


;//! \htmlinclude Servo-request.msg.html

(cl:defclass <Servo-request> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass Servo-request (<Servo-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Servo-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Servo-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<Servo-request> is deprecated: use drivers-srv:Servo-request instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <Servo-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:trajectory-val is deprecated.  Use drivers-srv:trajectory instead.")
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Servo-request>) ostream)
  "Serializes a message object of type '<Servo-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Servo-request>) istream)
  "Deserializes a message object of type '<Servo-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Servo-request>)))
  "Returns string type for a service object of type '<Servo-request>"
  "drivers/ServoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servo-request)))
  "Returns string type for a service object of type 'Servo-request"
  "drivers/ServoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Servo-request>)))
  "Returns md5sum for a message object of type '<Servo-request>"
  "2a0eff76c870e8595636c2a562ca298e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servo-request)))
  "Returns md5sum for a message object of type 'Servo-request"
  "2a0eff76c870e8595636c2a562ca298e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Servo-request>)))
  "Returns full string definition for message of type '<Servo-request>"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Servo-request)))
  "Returns full string definition for message of type 'Servo-request"
  (cl:format cl:nil "trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%# Each trajectory point specifies either positions[, velocities[, accelerations]]~%# or positions[, effort] for the trajectory to be executed.~%# All specified values are in the same order as the joint names in JointTrajectory.msg~%~%float64[] positions~%float64[] velocities~%float64[] accelerations~%float64[] effort~%duration time_from_start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Servo-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Servo-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Servo-request
    (cl:cons ':trajectory (trajectory msg))
))
;//! \htmlinclude Servo-response.msg.html

(cl:defclass <Servo-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Servo-response (<Servo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Servo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Servo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<Servo-response> is deprecated: use drivers-srv:Servo-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Servo-response>) ostream)
  "Serializes a message object of type '<Servo-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Servo-response>) istream)
  "Deserializes a message object of type '<Servo-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Servo-response>)))
  "Returns string type for a service object of type '<Servo-response>"
  "drivers/ServoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servo-response)))
  "Returns string type for a service object of type 'Servo-response"
  "drivers/ServoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Servo-response>)))
  "Returns md5sum for a message object of type '<Servo-response>"
  "2a0eff76c870e8595636c2a562ca298e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servo-response)))
  "Returns md5sum for a message object of type 'Servo-response"
  "2a0eff76c870e8595636c2a562ca298e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Servo-response>)))
  "Returns full string definition for message of type '<Servo-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Servo-response)))
  "Returns full string definition for message of type 'Servo-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Servo-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Servo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Servo-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Servo)))
  'Servo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Servo)))
  'Servo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servo)))
  "Returns string type for a service object of type '<Servo>"
  "drivers/Servo")