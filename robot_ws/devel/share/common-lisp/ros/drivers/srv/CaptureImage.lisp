; Auto-generated. Do not edit!


(cl:in-package drivers-srv)


;//! \htmlinclude CaptureImage-request.msg.html

(cl:defclass <CaptureImage-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass CaptureImage-request (<CaptureImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<CaptureImage-request> is deprecated: use drivers-srv:CaptureImage-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-request>) ostream)
  "Serializes a message object of type '<CaptureImage-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-request>) istream)
  "Deserializes a message object of type '<CaptureImage-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-request>)))
  "Returns string type for a service object of type '<CaptureImage-request>"
  "drivers/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-request)))
  "Returns string type for a service object of type 'CaptureImage-request"
  "drivers/CaptureImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-request>)))
  "Returns md5sum for a message object of type '<CaptureImage-request>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-request)))
  "Returns md5sum for a message object of type 'CaptureImage-request"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-request>)))
  "Returns full string definition for message of type '<CaptureImage-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-request)))
  "Returns full string definition for message of type 'CaptureImage-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-request
))
;//! \htmlinclude CaptureImage-response.msg.html

(cl:defclass <CaptureImage-response> (roslisp-msg-protocol:ros-message)
  ((image
    :reader image
    :initarg :image
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass CaptureImage-response (<CaptureImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CaptureImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CaptureImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drivers-srv:<CaptureImage-response> is deprecated: use drivers-srv:CaptureImage-response instead.")))

(cl:ensure-generic-function 'image-val :lambda-list '(m))
(cl:defmethod image-val ((m <CaptureImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drivers-srv:image-val is deprecated.  Use drivers-srv:image instead.")
  (image m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CaptureImage-response>) ostream)
  "Serializes a message object of type '<CaptureImage-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CaptureImage-response>) istream)
  "Deserializes a message object of type '<CaptureImage-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CaptureImage-response>)))
  "Returns string type for a service object of type '<CaptureImage-response>"
  "drivers/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage-response)))
  "Returns string type for a service object of type 'CaptureImage-response"
  "drivers/CaptureImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CaptureImage-response>)))
  "Returns md5sum for a message object of type '<CaptureImage-response>"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CaptureImage-response)))
  "Returns md5sum for a message object of type 'CaptureImage-response"
  "b13d2865c5af2a64e6e30ab1b56e1dd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CaptureImage-response>)))
  "Returns full string definition for message of type '<CaptureImage-response>"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CaptureImage-response)))
  "Returns full string definition for message of type 'CaptureImage-response"
  (cl:format cl:nil "sensor_msgs/Image image~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CaptureImage-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CaptureImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CaptureImage-response
    (cl:cons ':image (image msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CaptureImage)))
  'CaptureImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CaptureImage)))
  "Returns string type for a service object of type '<CaptureImage>"
  "drivers/CaptureImage")