(in-package :simulator)

(defconstant +mass-earth+ (* 6 (expt 10 24)))

(defconstant +mass-moon+ (* 7.347 (expt 10 22)))

(defconstant +radius-earth+ (* 6.357 (expt 10 6)))

(defconstant +gravity-constant+ (* 6.67428 (expt 10 -11)))

(defconstant +gravitational-parameter+ (* +gravity-constant+ +mass-earth+))

(defconstant +gravitational-parameter-moon+ (* +gravity-constant+ +mass-moon+))

(defun distance-between-bodies (sx1 sy1 sx2 sy2)
  (sqrt
    (+ (expt (- sx2 sx1) 2)
       (expt (- sy2 sy1) 2))))

(defun distance-to-earth (sx sy)
  (distance-between-bodies sx sy 0 0))

(defun gravity-force-common (radius mass)
  (/ (* +gravity-constant+ mass)
     (expt radius 2)))

(defun gravity-force (radius)
  (gravity-force-common radius +mass-earth+))

(defun gravity-force-to-moon (radius)
  (gravity-force-common radius +mass-moon+))

(defun rotation-period (radius)
  (* 2 pi
     (sqrt (/ (expt radius 3)
	      +gravitational-parameter+))))

(defun time-to-change-orbit (from-radius to-radius)
  (* pi
     (sqrt (/ (expt (+ from-radius to-radius) 3)
	      (* 8 +gravitational-parameter+)))))

(defun time-diff-to-meet (from-radius to-radius)
  (multiple-value-bind (k time)
    (floor (time-to-change-orbit from-radius to-radius)
	   (rotation-period to-radius))
    time))

(defun vector-length (x y)
  (sqrt
   (+ (expt x 2)
      (expt y 2))))

(defun vectors-scalar-product (x1 y1 x2 y2)
  (+ (* x1 x2)
     (* y1 y2)))

(defun cosin-between-vectors (x1 y1 x2 y2)
  (/ (vectors-scalar-product x1 y1 x2 y2)
     (* (vector-length x1 y1) (vector-length x2 y2))))

(defun disposition-between-vectors (x1 y1 x2 y2)
  (let ((c (- (* x1 y2)
	      (* x2 y1))))
    (if (> c 0)
      :rightmost
      :leftmost)))

(defun tangent-vector (x y rotation)
  (let ((l (vector-length x y)))
    (ecase rotation
      (:ccw (values (/ (- y) l) (/ x l)))
      (:cw (values (/ y l) (/ (- x) l))))))

(defun hohmann-first-thrust (r-1 r-2)
  (if (< r-1 r-2)
      (* (sqrt (/ +gravitational-parameter+ r-1))
         (- (sqrt (/ (* 2 r-2)
                     (+ r-1 r-2)))
            1))
      (hohmann-first-thrust r-2 r-1)))

(defun hohmann-second-thrust (r-1 r-2)
  (if (< r-1 r-2)
      (* (sqrt (/ +gravitational-parameter+ r-2))
         (- 1 (sqrt (/ (* 2 r-1)
                       (+ r-1 r-2)))))
      (hohmann-second-thrust r-2 r-1)))

(defstruct control-structure vm-semaphore ctl-semaphore v-x v-y simulator)

(defun control-structure-vm-wait (control-structure)
  (sb-thread:wait-on-semaphore (control-structure-vm-semaphore control-structure)))

(defun control-structure-ctl-wait (control-structure)
  (sb-thread:wait-on-semaphore (control-structure-ctl-semaphore control-structure)))

(defun control-structure-vm-signal (control-structure)
  (sb-thread:signal-semaphore (control-structure-ctl-semaphore control-structure)))

(defun control-structure-ctl-signal (control-structure)
  (sb-thread:signal-semaphore (control-structure-vm-semaphore control-structure)))

(defun gravity-acceleration-at (position)
  (2d-vector-* (* -1 (gravity-force (2d-vector-length position)))
               (2d-vector-normalize position)))

(defun hohmann-change-circular-orbit (c target-radius)
  (let (x-1 x-2 y-1 y-2
	    rotation
	    r-1
	    (r-2 target-radius)
	    first-thrust
	    second-thrust
	    (simulator (control-structure-simulator c)))
    (let* ((info (simulator-info simulator :hohmann))
           (our-x (getf info :our-x))
           (our-y (getf info :our-y)))
      (setf r-1 (distance-to-earth our-x our-y)
            x-1 our-x
            y-1 our-y
            first-thrust (hohmann-first-thrust r-1 r-2)
            second-thrust (hohmann-second-thrust r-1 r-2))
      (format t "r-1 = ~A, r-2 = ~A, f-t = ~A, s-t = ~A~%" r-1 r-2 first-thrust second-thrust)
      (setf (control-structure-v-x c) 0
	    (control-structure-v-y c) 0))
    (control-structure-ctl-signal c)
    (control-structure-ctl-wait c)
    (let* ((info (simulator-info simulator :hohmann))
           (our-x (getf info :our-x))
           (our-y (getf info :our-y)))
      (setf x-2 our-x
            y-2 our-y
            rotation (if (> (- (* x-1 y-2) (* x-2 y-1)) 0) :cw :ccw))
      (multiple-value-bind (t-x t-y) (tangent-vector our-x our-y rotation)
        (let ((z (if (< r-1 r-2) first-thrust (- second-thrust))))
          (setf (control-structure-v-x c) (* t-x z)
                (control-structure-v-y c) (* t-y z)))
        (format t "fired thrusters (~A, ~A)~%" (control-structure-v-x c) (control-structure-v-y c))))
    (control-structure-ctl-signal c)
    (iter (control-structure-ctl-wait c)
	  (for position = (our-position c))
	  (for radius = (2d-vector-length position))
	  (for p-radius previous radius)
	  (unless (first-iteration-p)
	    (when (funcall (if (< r-1 r-2) '< '>) radius p-radius)
	      (format t "R-1:~a, R-2:~a, R:~a, PR:~a~%" r-1 r-2 radius p-radius)
	      (multiple-value-bind (t-x t-y) (tangent-vector (2d-vector-x position) (2d-vector-y position) rotation)
		(let ((z (if (< r-1 r-2) second-thrust (- first-thrust))))
		  (setf (control-structure-v-x c) (* t-x z)
			(control-structure-v-y c) (* t-y z))))
	      (format t "fired thrusters (~A, ~A)~%" (control-structure-v-x c) (control-structure-v-y c))
	      (control-structure-ctl-signal c)
	      (finish)))
	  (setf (control-structure-v-x c) 0.0d0
		(control-structure-v-y c) 0.0d0)
	  (control-structure-ctl-signal c))
    (control-structure-ctl-wait c)))

(defun skip-turn (c)
  (setf (control-structure-v-x c) 0.0d0
        (control-structure-v-y c) 0.0d0)
  (control-structure-ctl-signal c)
  (control-structure-ctl-wait c))

(defun skip-n-turns (c n)
  (iter (repeat n)
        (skip-turn c)))

(defun skip-turns (c)
  (iter (while t)
        (skip-turn c)))

(defun hohmann-control-function (c)
  (control-structure-ctl-wait c)
  (let* ((info (simulator-info (control-structure-simulator c) :hohmann))
         (x (getf info :our-x))
         (y (getf info :our-y))
         (our-orbit (vector-length x y))
         (target-orbit (getf info :target-orbit-radius))
         (orbit-diff (- target-orbit our-orbit))
         (orbit-delta (/ orbit-diff 2)))
    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 1)))
    (skip-n-turns c 1000)
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 2)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 3)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 4)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 5)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 6)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 7)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 8)))
;    (hohmann-change-circular-orbit c (+ our-orbit (* orbit-delta 9)))
    (hohmann-change-circular-orbit c target-orbit))
  (skip-turns c))

(defstruct 2d-vector x y)

(defun polar-angle (vector)
  (let ((angle (atan (2d-vector-y vector) (2d-vector-x vector))))
    (if (minusp angle)
        (+ angle (* 2 pi))
        angle)))

(defun 2d-vector-length (vector)
  (vector-length (2d-vector-x vector) (2d-vector-y vector)))

(defun 2d-vector-+ (a b)
  (make-2d-vector :x (+ (2d-vector-x a) (2d-vector-x b))
                  :y (+ (2d-vector-y a) (2d-vector-y b))))

(defun 2d-vector-- (a b)
  (make-2d-vector :x (- (2d-vector-x a) (2d-vector-x b))
                  :y (- (2d-vector-y a) (2d-vector-y b))))

(defun 2d-vector-* (c a)
  (make-2d-vector :x (* c (2d-vector-x a))
                  :y (* c (2d-vector-y a))))

(defun 2d-vector-normalize (v)
  (2d-vector-* (/ 1 (2d-vector-length v)) v))

(defun 2d-vector-perp (v rotation)
  (ecase rotation
    (:ccw (make-2d-vector :x (- (2d-vector-y v)) :y (2d-vector-x v)))
    (:cw (make-2d-vector :x (2d-vector-y v) :y (- (2d-vector-x v))))))

(defun angular-speed-at-orbit (radius)
  (sqrt (/ +gravitational-parameter+ (expt radius 3))))

(defun meet-and-greet-wait-time (position target-position)
  (let* ((alpha-0 (polar-angle position))
         (beta-0 (polar-angle target-position))
         (tau (time-to-change-orbit (2d-vector-length position) (2d-vector-length target-position)))
         (c-1 (- (angular-speed-at-orbit (2d-vector-length position))
                 (angular-speed-at-orbit (2d-vector-length target-position))))
         (c-2 (+ (- alpha-0)
                 pi
                 beta-0
                 (- (* tau (angular-speed-at-orbit (2d-vector-length target-position))))))
         (z (nth-value 1 (floor c-2 (* 2 pi))))
         (wait-time (if (plusp c-1)
                        (/ (- (* 2 pi) z)
                           c-1)
                        (/ (- z)
                           c-1))))
    (format t "r-1 = ~F, r-2 = ~F, alpha-0 = ~F, beta-0 = ~F, tau = ~F, c-1 = ~F, c-2 = ~F, z = ~F~%"
            (2d-vector-length position) (2d-vector-length target-position)
            alpha-0 beta-0 tau c-1 c-2 z)
    wait-time))

(defun our-position (c)
  (let* ((info (simulator-info (control-structure-simulator c) :hohmann))
         (x (getf info :our-x))
         (y (getf info :our-y)))
    (make-2d-vector :x x :y y)))

(defun target-position (c)
  (let* ((info (simulator-info (control-structure-simulator c) :meet-and-greet))
         (x (getf info :our-x))
         (y (getf info :our-y))
         (target-x (- x (getf info :target-rel-x)))
         (target-y (- y (getf info :target-rel-y))))
    (make-2d-vector :x target-x :y target-y)))

(defun correct-orbit (c delta-angle)
  (let* ((position (our-position c))
         (r (2d-vector-length position))
         (v (sqrt (* (gravity-force r) r)))
         (delta-v-tangential (* r delta-angle))
         (delta-v-normal (- (/ (expt (+ v delta-v-tangential) 2)
                               r)
                            (gravity-force r)))
         (normal (2d-vector-* -1 (2d-vector-normalize position)))
         (tangent (2d-vector-normalize (2d-vector-perp position :ccw)))
         (thrust (2d-vector-+ (2d-vector-* delta-v-tangential tangent)
                              (2d-vector-* delta-v-normal normal))))
    (format t "position = ~A~%r = ~A~%v = ~A~%d-v-t = ~A~%d-v-n = ~A~%normal = ~A~%tangent = ~A~%thrust = ~A~%"
            position r v delta-v-tangential delta-v-normal normal tangent thrust)
    (setf (control-structure-v-x c) (2d-vector-x thrust)
          (control-structure-v-y c) (2d-vector-y thrust))
    (control-structure-ctl-signal c)
    (control-structure-ctl-wait c)
    (setf (control-structure-v-x c) (- (2d-vector-x thrust))
          (control-structure-v-y c) (- (2d-vector-y thrust)))
    (control-structure-ctl-signal c)
    (control-structure-ctl-wait c)
    #+nil(let* ((position (our-position c))
           (normal (2d-vector-* -1 (2d-vector-normalize position)))
           (tangent (2d-vector-normalize (2d-vector-perp position :ccw)))
           (thrust (2d-vector-* -1 (2d-vector-+ (2d-vector-* delta-v-tangential tangent)
                                                (2d-vector-* delta-v-normal normal)))))
      (setf (control-structure-v-x c) (2d-vector-x thrust)
            (control-structure-v-y c) (2d-vector-y thrust))
      (control-structure-ctl-signal c)
      (control-structure-ctl-wait c))))

(defparameter *dat-path* #p "/tmp/")

(defun meet-and-greet-active-chase (c)
  (let* ((stream-x (open (merge-pathnames "position-x.dat1" *dat-path*) :direction :output :if-exists :supersede))
         (stream-y (open (merge-pathnames "position-y.dat1" *dat-path*) :direction :output :if-exists :supersede))
         (stream-t-x (open (merge-pathnames "target-x.dat1" *dat-path*) :direction :output :if-exists :supersede))
         (stream-t-y (open (merge-pathnames "target-y.dat1" *dat-path*) :direction :output :if-exists :supersede)))
    (unwind-protect
         (iter (for position = (our-position c))
               (for target = (target-position c))
               (while t)
               (format stream-x "~F " (2d-vector-x position))
               (format stream-y "~F " (2d-vector-y position))
               (format stream-t-x "~F " (2d-vector-x target))
               (format stream-t-y "~F " (2d-vector-y target))
               (for p-position previous position)
               (for p-target previous target)
               (for counter from 0)
               (if (first-iteration-p)
                   (skip-turn c)
                   (progn (for velocity = (2d-vector-- position p-position))
                          (for target-velocity = (2d-vector-- target p-target))
                          (for acceleration = (gravity-acceleration-at position))
                          (for target-acceleration = (gravity-acceleration-at target))
                          (for p = (2d-vector-+ (2d-vector-- position target)
                                                (2d-vector-+ (2d-vector-- velocity target-velocity)
                                                             (2d-vector-* 0.5d0 (2d-vector-+ acceleration target-acceleration)))))
                          (for q = (2d-vector-+ (2d-vector-- velocity target-velocity)
                                                (2d-vector-- acceleration target-acceleration)))
                          (for thrust = (2d-vector-+ (2d-vector-* 4/5 q)
                                                     (2d-vector-* 2/5 p)))
                          (if (and (< (2d-vector-length (2d-vector-- position target)) 100)
                                   (< (2d-vector-length (2d-vector-- velocity target-velocity)) 100))
                              (progn (setf (control-structure-v-x c) 0
                                           (control-structure-v-y c) 0)
                                     (control-structure-ctl-signal c)
                                     (control-structure-ctl-wait c))
                              (progn (setf (control-structure-v-x c) (* 0.001 (2d-vector-x thrust))
                                           (control-structure-v-y c) (* 0.001 (2d-vector-y thrust)))
                                     (control-structure-ctl-signal c)
                                     (control-structure-ctl-wait c)
                                     (skip-n-turns c 20))))))
      (progn (close stream-x)
             (close stream-y)
             (close stream-t-x)
             (close stream-t-y)))))

(defun meet-and-greet-control (c)
  (control-structure-ctl-wait c)
  (let* ((info (simulator-info (control-structure-simulator c) :meet-and-greet))
         (x (getf info :our-x))
         (y (getf info :our-y))
         (target-x (- x (getf info :target-rel-x)))
         (target-y (- y (getf info :target-rel-y)))
         (position (make-2d-vector :x x :y y))
         (target-position (make-2d-vector :x target-x :y target-y))
         (wait-time (meet-and-greet-wait-time position target-position))
         (r-2 (2d-vector-length target-position)))
    (format t "position = ~A,~%target-position = ~A,~%wait-time = ~A, r-1= = ~A, r-2 = ~A~%"
            position target-position wait-time (2d-vector-length position) r-2)
    (iter (repeat (round wait-time))
          (setf (control-structure-v-x c) 0
                (control-structure-v-y c) 0)
          (control-structure-ctl-signal c)
          (control-structure-ctl-wait c))
    (hohmann-change-circular-orbit c r-2)
    (let* ((info (simulator-info (control-structure-simulator c) :meet-and-greet))
           (x (getf info :our-x))
           (y (getf info :our-y))
           (target-x (- x (getf info :target-rel-x)))
           (target-y (- y (getf info :target-rel-y)))
           (position (make-2d-vector :x x :y y))
           (target-position (make-2d-vector :x target-x :y target-y)))
      (format t "position = ~A,~%target-position = ~A,~%distance = ~A, range = ~A, target-range = ~A~%"
              position target-position
              (2d-vector-length (2d-vector-- position target-position))
              (2d-vector-length position) (2d-vector-length target-position))))
  #+nil(iter (repeat 10)
             (let* ((position (our-position c))
                    (target-position (target-position c))
                    (delta-angle (* -1
                                    (- (polar-angle target-position)
                                       (polar-angle position)))))
               (correct-orbit c delta-angle))
             (format t "distance = ~A~%" (2d-vector-length (2d-vector-- (our-position c) (target-position c))))
             (skip-n-turns c 100))
  #+nil(iter (repeat 900)
             (for i from 0)
             (skip-turn c)
             (when (zerop (mod i 100))
               (format t "distance = ~A~%" (2d-vector-length (2d-vector-- (our-position c) (target-position c))))))
  (skip-n-turns c 5)
  (meet-and-greet-active-chase c)
  (skip-turns c))
