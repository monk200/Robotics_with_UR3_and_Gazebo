;; Auto-generated. Do not edit!


(when (boundp 'ur3_driver::gripper_input)
  (if (not (find-package "UR3_DRIVER"))
    (make-package "UR3_DRIVER"))
  (shadow 'gripper_input (find-package "UR3_DRIVER")))
(unless (find-package "UR3_DRIVER::GRIPPER_INPUT")
  (make-package "UR3_DRIVER::GRIPPER_INPUT"))

(in-package "ROS")
;;//! \htmlinclude gripper_input.msg.html


(defclass ur3_driver::gripper_input
  :super ros::object
  :slots (_DIGIN _AIN0 _AIN1 ))

(defmethod ur3_driver::gripper_input
  (:init
   (&key
    ((:DIGIN __DIGIN) 0)
    ((:AIN0 __AIN0) 0.0)
    ((:AIN1 __AIN1) 0.0)
    )
   (send-super :init)
   (setq _DIGIN (round __DIGIN))
   (setq _AIN0 (float __AIN0))
   (setq _AIN1 (float __AIN1))
   self)
  (:DIGIN
   (&optional __DIGIN)
   (if __DIGIN (setq _DIGIN __DIGIN)) _DIGIN)
  (:AIN0
   (&optional __AIN0)
   (if __AIN0 (setq _AIN0 __AIN0)) _AIN0)
  (:AIN1
   (&optional __AIN1)
   (if __AIN1 (setq _AIN1 __AIN1)) _AIN1)
  (:serialization-length
   ()
   (+
    ;; int32 _DIGIN
    4
    ;; float64 _AIN0
    8
    ;; float64 _AIN1
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _DIGIN
       (write-long _DIGIN s)
     ;; float64 _AIN0
       (sys::poke _AIN0 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _AIN1
       (sys::poke _AIN1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _DIGIN
     (setq _DIGIN (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _AIN0
     (setq _AIN0 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _AIN1
     (setq _AIN1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ur3_driver::gripper_input :md5sum-) "a4b7d373885d48c37baffd91ce2f1c38")
(setf (get ur3_driver::gripper_input :datatype-) "ur3_driver/gripper_input")
(setf (get ur3_driver::gripper_input :definition-)
      "int32 DIGIN 
float64 AIN0
float64 AIN1

")



(provide :ur3_driver/gripper_input "a4b7d373885d48c37baffd91ce2f1c38")


