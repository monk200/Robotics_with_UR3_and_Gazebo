;; Auto-generated. Do not edit!


(when (boundp 'ur3_driver::command)
  (if (not (find-package "UR3_DRIVER"))
    (make-package "UR3_DRIVER"))
  (shadow 'command (find-package "UR3_DRIVER")))
(unless (find-package "UR3_DRIVER::COMMAND")
  (make-package "UR3_DRIVER::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude command.msg.html


(defclass ur3_driver::command
  :super ros::object
  :slots (_destination _v _a _io_0 ))

(defmethod ur3_driver::command
  (:init
   (&key
    ((:destination __destination) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:v __v) 0.0)
    ((:a __a) 0.0)
    ((:io_0 __io_0) nil)
    )
   (send-super :init)
   (setq _destination __destination)
   (setq _v (float __v))
   (setq _a (float __a))
   (setq _io_0 __io_0)
   self)
  (:destination
   (&optional __destination)
   (if __destination (setq _destination __destination)) _destination)
  (:v
   (&optional __v)
   (if __v (setq _v __v)) _v)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:io_0
   (&optional __io_0)
   (if __io_0 (setq _io_0 __io_0)) _io_0)
  (:serialization-length
   ()
   (+
    ;; float64[] _destination
    (* 8    (length _destination)) 4
    ;; float64 _v
    8
    ;; float64 _a
    8
    ;; bool _io_0
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _destination
     (write-long (length _destination) s)
     (dotimes (i (length _destination))
       (sys::poke (elt _destination i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _v
       (sys::poke _v (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _a
       (sys::poke _a (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _io_0
       (if _io_0 (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _destination
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _destination (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _destination i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64 _v
     (setq _v (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _a
     (setq _a (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _io_0
     (setq _io_0 (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ur3_driver::command :md5sum-) "c8890f8a9e9a6dc32b97081f6283bc11")
(setf (get ur3_driver::command :datatype-) "ur3_driver/command")
(setf (get ur3_driver::command :definition-)
      "float64[] destination
float64 v
float64 a
bool io_0

")



(provide :ur3_driver/command "c8890f8a9e9a6dc32b97081f6283bc11")


