(in-package :simulator)

(defconstant +mass-earth+ (* 6 (expt 10 24)))

(defconstant +radius-earth+ (* 6.357 (expt 10 6)))

(defconstant +gravity-constant+ (* 6.67428 (expt 10 -11)))

(defconstant +gravitational-parameter+ (* +gravity-constant+ +mass-earth+))

(defun distance-between-bodies (sx1 sy1 sx2 sy2)
  (sqrt
    (+ (expt (- sx2 sx1) 2)
       (expt (- sy2 sy1) 2))))

(defun distance-to-earth (sx sy)
  (distance-between-bodies sx sy 0 0))

(defun gravity-force (radius)
  (/ (* +gravity-constant+ +mass-earth+)
     (expt radius 2)))

(defun vector-length (x y)
  (sqrt
   (+ (expt x 2)
      (expt y 2))))

(defun tangent-vector (x y rotation)
  (let ((l (vector-length x y)))
    (ecase rotation
      (:cw (values (/ (- y) l) (/ x l)))
      (:ccw (values (/ y l) (/ (- x) l))))))

(defun hohmann-first-thrust (r-1 r-2)
  (* (sqrt (/ +gravitational-parameter+ r-1))
     (- (sqrt (/ (* 2 r-2)
                 (+ r-1 r-2)))
        1)))

(defun hohmann-second-thrust (r-1 r-2)
  (* (sqrt (/ +gravitational-parameter+ r-2))
     (- 1 (sqrt (/ (* 2 r-1)
                   (+ r-1 r-2))))))

(defstruct control-structure vm-semaphore ctl-semaphore v-x v-y simulator)

(defun control-structure-vm-wait (control-structure)
  (sb-thread:wait-on-semaphore (control-structure-vm-semaphore control-structure)))

(defun control-structure-ctl-wait (control-structure)
  (sb-thread:wait-on-semaphore (control-structure-ctl-semaphore control-structure)))

(defun control-structure-vm-signal (control-structure)
  (sb-thread:signal-semaphore (control-structure-ctl-semaphore control-structure)))

(defun control-structure-ctl-signal (control-structure)
  (sb-thread:signal-semaphore (control-structure-vm-semaphore control-structure)))

(defun hohmann-control-function (c)
  (control-structure-ctl-wait c)
  (finish-output)
  (let (first-tick-angle
        second-tick-angle
        rotation
        r-1
        r-2
        first-thrust
        second-thrust
        (max-radius 0.0d0)
        (simulator (control-structure-simulator c)))
    (let* ((info (simulator-info simulator :hohmann))
           (our-x (getf info :our-x))
           (our-y (getf info :our-y)))
      (setf r-1 (distance-to-earth our-x our-y)
            r-2 (getf info :target-orbit-radius)
            first-thrust (hohmann-first-thrust r-1 r-2)
            second-thrust (hohmann-second-thrust r-1 r-2)
            first-tick-angle (atan our-y our-x)
            (control-structure-v-x c) 0
            (control-structure-v-y c) 0))
    (format t "r-1 = ~A, r-2 = ~A, f-t = ~A, s-t = ~A, f-t-a = ~A~%" r-1 r-2 first-thrust second-thrust first-tick-angle)
    (control-structure-ctl-signal c)
    (control-structure-ctl-wait c)
    (finish-output)
    (let* ((info (simulator-info simulator :hohmann))
           (our-x (getf info :our-x))
           (our-y (getf info :our-y)))
      (setf second-tick-angle (atan our-y our-x)
            rotation (if (< first-tick-angle second-tick-angle) :ccw :cw))
      (multiple-value-bind (t-x t-y) (tangent-vector our-x our-y rotation)
        (setf (control-structure-v-x c) (* t-x first-thrust)
              (control-structure-v-y c) (* t-y first-thrust))
        (format t "fired thrusters (~A, ~A)~%" (control-structure-v-x c) (control-structure-v-y c))))
    (control-structure-ctl-signal c)
    (finish-output)
    (iter (control-structure-ctl-wait c)
          (for info = (simulator-info simulator :hohmann))
          (for our-x = (getf info :our-x))
          (for our-y = (getf info :our-y))
          (for radius = (vector-length our-x our-y))
          (if (< radius max-radius)
              (progn (multiple-value-bind (t-x t-y) (tangent-vector our-x our-y rotation)
                       (setf (control-structure-v-x c) (* t-x second-thrust)
                             (control-structure-v-y c) (* t-y second-thrust)))
                     (format t "fired thrusters (~A, ~A)~%" (control-structure-v-x c) (control-structure-v-y c))
                     (control-structure-ctl-signal c)
                     (finish))
              (setf max-radius radius
                    (control-structure-v-x c) 0.0d0
                    (control-structure-v-y c) 0.0d0))
          (control-structure-ctl-signal c))
    (iter (while t)
          (control-structure-ctl-wait c)
          (setf (control-structure-v-x c) 0.0d0
                (control-structure-v-y c) 0.0d0)
          (control-structure-ctl-signal c))))
