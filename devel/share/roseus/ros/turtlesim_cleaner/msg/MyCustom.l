;; Auto-generated. Do not edit!


(when (boundp 'turtlesim_cleaner::MyCustom)
  (if (not (find-package "TURTLESIM_CLEANER"))
    (make-package "TURTLESIM_CLEANER"))
  (shadow 'MyCustom (find-package "TURTLESIM_CLEANER")))
(unless (find-package "TURTLESIM_CLEANER::MYCUSTOM")
  (make-package "TURTLESIM_CLEANER::MYCUSTOM"))

(in-package "ROS")
;;//! \htmlinclude MyCustom.msg.html


(defclass turtlesim_cleaner::MyCustom
  :super ros::object
  :slots (_x _y _arr ))

(defmethod turtlesim_cleaner::MyCustom
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:arr __arr) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _arr __arr)
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:arr
   (&optional __arr)
   (if __arr (setq _arr __arr)) _arr)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; float64[] _arr
    (* 8    (length _arr)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _arr
     (write-long (length _arr) s)
     (dotimes (i (length _arr))
       (sys::poke (elt _arr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _arr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _arr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get turtlesim_cleaner::MyCustom :md5sum-) "957a84dc144cfbccfcf214c35dd36d2d")
(setf (get turtlesim_cleaner::MyCustom :datatype-) "turtlesim_cleaner/MyCustom")
(setf (get turtlesim_cleaner::MyCustom :definition-)
      "float64 x
float64 y
float64[] arr

")



(provide :turtlesim_cleaner/MyCustom "957a84dc144cfbccfcf214c35dd36d2d")


