;; Auto-generated. Do not edit!


(when (boundp 'drivers::CaptureImage)
  (if (not (find-package "DRIVERS"))
    (make-package "DRIVERS"))
  (shadow 'CaptureImage (find-package "DRIVERS")))
(unless (find-package "DRIVERS::CAPTUREIMAGE")
  (make-package "DRIVERS::CAPTUREIMAGE"))
(unless (find-package "DRIVERS::CAPTUREIMAGEREQUEST")
  (make-package "DRIVERS::CAPTUREIMAGEREQUEST"))
(unless (find-package "DRIVERS::CAPTUREIMAGERESPONSE")
  (make-package "DRIVERS::CAPTUREIMAGERESPONSE"))

(in-package "ROS")



(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass drivers::CaptureImageRequest
  :super ros::object
  :slots ())

(defmethod drivers::CaptureImageRequest
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

(defclass drivers::CaptureImageResponse
  :super ros::object
  :slots (_image ))

(defmethod drivers::CaptureImageResponse
  (:init
   (&key
    ((:image __image) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image __image)
   self)
  (:image
   (&rest __image)
   (if (keywordp (car __image))
       (send* _image __image)
     (progn
       (if __image (setq _image (car __image)))
       _image)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image
    (send _image :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image
       (send _image :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image
     (send _image :deserialize buf ptr-) (incf ptr- (send _image :serialization-length))
   ;;
   self)
  )

(defclass drivers::CaptureImage
  :super ros::object
  :slots ())

(setf (get drivers::CaptureImage :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get drivers::CaptureImage :datatype-) "drivers/CaptureImage")
(setf (get drivers::CaptureImage :request) drivers::CaptureImageRequest)
(setf (get drivers::CaptureImage :response) drivers::CaptureImageResponse)

(defmethod drivers::CaptureImageRequest
  (:response () (instance drivers::CaptureImageResponse :init)))

(setf (get drivers::CaptureImageRequest :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get drivers::CaptureImageRequest :datatype-) "drivers/CaptureImageRequest")
(setf (get drivers::CaptureImageRequest :definition-)
      "---
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

(setf (get drivers::CaptureImageResponse :md5sum-) "b13d2865c5af2a64e6e30ab1b56e1dd5")
(setf (get drivers::CaptureImageResponse :datatype-) "drivers/CaptureImageResponse")
(setf (get drivers::CaptureImageResponse :definition-)
      "---
sensor_msgs/Image image

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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



(provide :drivers/CaptureImage "b13d2865c5af2a64e6e30ab1b56e1dd5")


