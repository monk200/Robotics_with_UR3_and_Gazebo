; Auto-generated. Do not edit!


(cl:in-package ur3_driver-msg)


;//! \htmlinclude gripper_input.msg.html

(cl:defclass <gripper_input> (roslisp-msg-protocol:ros-message)
  ((DIGIN
    :reader DIGIN
    :initarg :DIGIN
    :type cl:integer
    :initform 0)
   (AIN0
    :reader AIN0
    :initarg :AIN0
    :type cl:float
    :initform 0.0)
   (AIN1
    :reader AIN1
    :initarg :AIN1
    :type cl:float
    :initform 0.0))
)

(cl:defclass gripper_input (<gripper_input>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gripper_input>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gripper_input)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur3_driver-msg:<gripper_input> is deprecated: use ur3_driver-msg:gripper_input instead.")))

(cl:ensure-generic-function 'DIGIN-val :lambda-list '(m))
(cl:defmethod DIGIN-val ((m <gripper_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_driver-msg:DIGIN-val is deprecated.  Use ur3_driver-msg:DIGIN instead.")
  (DIGIN m))

(cl:ensure-generic-function 'AIN0-val :lambda-list '(m))
(cl:defmethod AIN0-val ((m <gripper_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_driver-msg:AIN0-val is deprecated.  Use ur3_driver-msg:AIN0 instead.")
  (AIN0 m))

(cl:ensure-generic-function 'AIN1-val :lambda-list '(m))
(cl:defmethod AIN1-val ((m <gripper_input>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur3_driver-msg:AIN1-val is deprecated.  Use ur3_driver-msg:AIN1 instead.")
  (AIN1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gripper_input>) ostream)
  "Serializes a message object of type '<gripper_input>"
  (cl:let* ((signed (cl:slot-value msg 'DIGIN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'AIN0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'AIN1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gripper_input>) istream)
  "Deserializes a message object of type '<gripper_input>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'DIGIN) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AIN0) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AIN1) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gripper_input>)))
  "Returns string type for a message object of type '<gripper_input>"
  "ur3_driver/gripper_input")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gripper_input)))
  "Returns string type for a message object of type 'gripper_input"
  "ur3_driver/gripper_input")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gripper_input>)))
  "Returns md5sum for a message object of type '<gripper_input>"
  "a4b7d373885d48c37baffd91ce2f1c38")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gripper_input)))
  "Returns md5sum for a message object of type 'gripper_input"
  "a4b7d373885d48c37baffd91ce2f1c38")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gripper_input>)))
  "Returns full string definition for message of type '<gripper_input>"
  (cl:format cl:nil "int32 DIGIN ~%float64 AIN0~%float64 AIN1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gripper_input)))
  "Returns full string definition for message of type 'gripper_input"
  (cl:format cl:nil "int32 DIGIN ~%float64 AIN0~%float64 AIN1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gripper_input>))
  (cl:+ 0
     4
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gripper_input>))
  "Converts a ROS message object to a list"
  (cl:list 'gripper_input
    (cl:cons ':DIGIN (DIGIN msg))
    (cl:cons ':AIN0 (AIN0 msg))
    (cl:cons ':AIN1 (AIN1 msg))
))
