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
  "539851e77c84d32d3e4ea058da5cb99b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servo-request)))
  "Returns md5sum for a message object of type 'Servo-request"
  "539851e77c84d32d3e4ea058da5cb99b")
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
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform "")
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass Servo-response (<Servo-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Servo-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Servo-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<Servo-response> is deprecated: use drivers-srv:Servo-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <Servo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:success-val is deprecated.  Use drivers-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <Servo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:message-val is deprecated.  Use drivers-srv:message instead.")
  (message m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Servo-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:angle-val is deprecated.  Use drivers-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Servo-response>) ostream)
  "Serializes a message object of type '<Servo-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Servo-response>) istream)
  "Deserializes a message object of type '<Servo-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
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
  "539851e77c84d32d3e4ea058da5cb99b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Servo-response)))
  "Returns md5sum for a message object of type 'Servo-response"
  "539851e77c84d32d3e4ea058da5cb99b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Servo-response>)))
  "Returns full string definition for message of type '<Servo-response>"
  (cl:format cl:nil "bool success~%string message~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Servo-response)))
  "Returns full string definition for message of type 'Servo-response"
  (cl:format cl:nil "bool success~%string message~%float64 angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Servo-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Servo-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Servo-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
    (cl:cons ':angle (angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Servo)))
  'Servo-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Servo)))
  'Servo-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Servo)))
  "Returns string type for a service object of type '<Servo>"
  "drivers/Servo")