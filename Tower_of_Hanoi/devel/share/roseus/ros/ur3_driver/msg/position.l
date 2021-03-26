;; Auto-generated. Do not edit!


(when (boundp 'ur3_driver::position)
  (if (not (find-package "UR3_DRIVER"))
    (make-package "UR3_DRIVER"))
  (shadow 'position (find-package "UR3_DRIVER")))
(unless (find-package "UR3_DRIVER::POSITION")
  (make-package "UR3_DRIVER::POSITION"))

(in-package "ROS")
;;//! \htmlinclude position.msg.html


(defclass ur3_driver::position
  :super ros::object
  :slots (_position _isReady ))

(defmethod ur3_driver::position
  (:init
   (&key
    ((:position __position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:isReady __isReady) nil)
    )
   (send-super :init)
   (setq _position __position)
   (setq _isReady __isReady)
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:isReady
   (&optional __isReady)
   (if __isReady (setq _isReady __isReady)) _isReady)
  (:serialization-length
   ()
   (+
    ;; float64[] _position
    (* 8    (length _position)) 4
    ;; bool _isReady
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _position
     (write-long (length _position) s)
     (dotimes (i (length _position))
       (sys::poke (elt _position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _isReady
       (if _isReady (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _isReady
     (setq _isReady (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get ur3_driver::position :md5sum-) "86ad7fb432e90c4fcc2fa98c89466a9b")
(setf (get ur3_driver::position :datatype-) "ur3_driver/position")
(setf (get ur3_driver::position :definition-)
      "float64[] position
bool isReady

")



(provide :ur3_driver/position "86ad7fb432e90c4fcc2fa98c89466a9b")


