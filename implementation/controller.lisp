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

(defun make-hohmann-thrust-function ()
  "Create a thrust function for Hohmann orbit transfer problem.
Works in stages:
first tick - identify the source and target orbits
second tick - identify initial rotation: cw or ccw; first thrust; remember where to do the second thrust
n-th tick - second thrust"
  (let ((state :first)
        first-tick-angle
        second-tick-angle
        thrust-angle
        rotation
        r-1
        r-2
        first-thrust
        second-thrust)
    (flet ((first-tick (simulator)
             (let* ((info (simulator-info simulator :hohmann))
                    (our-x (getf info :our-x))
                    (our-y (getf info :our-y)))
               (setf r-1 (distance-to-earth our-x our-y)
                     r-2 (getf info :target-orbit-radius)
                     first-thrust (hohmann-first-thrust r-1 r-2)
                     second-thrust (hohmann-second-thrust r-1 r-2)
                     first-tick-angle (atan our-y our-x)))
             (format t "r-1 = ~A, r-2 = ~A, f-t = ~A, s-t = ~A, f-t-a = ~A~%" r-1 r-2 first-thrust second-thrust first-tick-angle)
             (values 0 0))
           (second-tick (simulator)
             (let* ((info (simulator-info simulator :hohmann))
                    (our-x (getf info :our-x))
                    (our-y (getf info :our-y)))
               (setf second-tick-angle (atan our-y our-x)
                     rotation (if (< first-tick-angle second-tick-angle) :ccw :cw)
                     thrust-angle (+ second-tick-angle pi))
               (multiple-value-bind (t-x t-y) (tangent-vector our-x our-y rotation)
                 (values (* t-x first-thrust)
                         (* t-y second-thrust)))))
           (should-fire-second-thrust (simulator) nil)
           (fire-second-thrust (simulator)))
      (lambda (simulator)
        (ecase state
          (:first (setf state :second)(first-tick simulator))
          (:second (setf state :wait-for-thrust) (second-tick simulator))
          (:wait-for-thrust (if (should-fire-second-thrust simulator)
                                (progn (setf state :idle)
                                       (fire-second-thrust simulator))
                                (values 0 0)))
          (:idle (values 0 0)))))))
