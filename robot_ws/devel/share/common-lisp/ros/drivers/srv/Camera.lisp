; Auto-generated. Do not edit!


(cl:in-package drivers-srv)


;//! \htmlinclude Camera-request.msg.html

(cl:defclass <Camera-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Camera-request (<Camera-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Camera-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Camera-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<Camera-request> is deprecated: use drivers-srv:Camera-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Camera-request>) ostream)
  "Serializes a message object of type '<Camera-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Camera-request>) istream)
  "Deserializes a message object of type '<Camera-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Camera-request>)))
  "Returns string type for a service object of type '<Camera-request>"
  "drivers/CameraRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera-request)))
  "Returns string type for a service object of type 'Camera-request"
  "drivers/CameraRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Camera-request>)))
  "Returns md5sum for a message object of type '<Camera-request>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Camera-request)))
  "Returns md5sum for a message object of type 'Camera-request"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Camera-request>)))
  "Returns full string definition for message of type '<Camera-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Camera-request)))
  "Returns full string definition for message of type 'Camera-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Camera-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Camera-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Camera-request
))
;//! \htmlinclude Camera-response.msg.html

(cl:defclass <Camera-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass Camera-response (<Camera-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Camera-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Camera-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<Camera-response> is deprecated: use drivers-srv:Camera-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <Camera-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:image-val is deprecated.  Use drivers-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Camera-response>) ostream)
  "Serializes a message object of type '<Camera-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Camera-response>) istream)
  "Deserializes a message object of type '<Camera-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Camera-response>)))
  "Returns string type for a service object of type '<Camera-response>"
  "drivers/CameraResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera-response)))
  "Returns string type for a service object of type 'Camera-response"
  "drivers/CameraResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Camera-response>)))
  "Returns md5sum for a message object of type '<Camera-response>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Camera-response)))
  "Returns md5sum for a message object of type 'Camera-response"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Camera-response>)))
  "Returns full string definition for message of type '<Camera-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Camera-response)))
  "Returns full string definition for message of type 'Camera-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Camera-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Camera-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Camera-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Camera)))
  'Camera-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Camera)))
  'Camera-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Camera)))
  "Returns string type for a service object of type '<Camera>"
  "drivers/Camera")