; Auto-generated. Do not edit!


(cl:in-package turtlesim_cleaner-msg)


;//! \htmlinclude MyCustom.msg.html

(cl:defclass <MyCustom> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (arr
    :reader arr
    :initarg :arr
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass MyCustom (<MyCustom>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyCustom>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyCustom)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name turtlesim_cleaner-msg:<MyCustom> is deprecated: use turtlesim_cleaner-msg:MyCustom instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <MyCustom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlesim_cleaner-msg:x-val is deprecated.  Use turtlesim_cleaner-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <MyCustom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlesim_cleaner-msg:y-val is deprecated.  Use turtlesim_cleaner-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'arr-val :lambda-list '(m))
(cl:defmethod arr-val ((m <MyCustom>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader turtlesim_cleaner-msg:arr-val is deprecated.  Use turtlesim_cleaner-msg:arr instead.")
  (arr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyCustom>) ostream)
  "Serializes a message object of type '<MyCustom>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'arr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'arr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyCustom>) istream)
  "Deserializes a message object of type '<MyCustom>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'arr) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'arr)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyCustom>)))
  "Returns string type for a message object of type '<MyCustom>"
  "turtlesim_cleaner/MyCustom")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyCustom)))
  "Returns string type for a message object of type 'MyCustom"
  "turtlesim_cleaner/MyCustom")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyCustom>)))
  "Returns md5sum for a message object of type '<MyCustom>"
  "957a84dc144cfbccfcf214c35dd36d2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyCustom)))
  "Returns md5sum for a message object of type 'MyCustom"
  "957a84dc144cfbccfcf214c35dd36d2d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyCustom>)))
  "Returns full string definition for message of type '<MyCustom>"
  (cl:format cl:nil "float64 x~%float64 y~%float64[] arr~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyCustom)))
  "Returns full string definition for message of type 'MyCustom"
  (cl:format cl:nil "float64 x~%float64 y~%float64[] arr~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyCustom>))
  (cl:+ 0
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'arr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyCustom>))
  "Converts a ROS message object to a list"
  (cl:list 'MyCustom
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':arr (arr msg))
))
