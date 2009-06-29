(eval-when (:compile-toplevel :load-toplevel :execute)
  (require 'iterate))

(defpackage :visualizer
  (:use :cl :iter :simulator)
  (:export :visualize-run :make-initial-thrust-function))

(in-package :visualizer)

(defstruct visualizer
  (xs (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t))
  (ys (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t)))

(defstruct (hohmann-visualizer (:include visualizer)) target-radius)

(defun problem->visualizer (problem)
  (case problem
    ((1001 1002 1003 1004) (make-hohmann-visualizer))
    ((2001 2002 2003 2004) (make-meet-and-greet-visualizer))
    ((3001 3002 3003 3004) (make-meet-and-greet-visualizer))
    (t (make-visualizer))))

(defun make-steps-finish-predicate (steps)
  (let ((counter 0))
    (lambda (simulator)
      (declare (ignore simulator))
      (incf counter)
      (> counter steps))))

(defun noop-thrust-function (simulator)
  (declare (ignore simulator))
  nil)

(defun visualize-run (result-path obf-path configuration control-function
                      &key
                      (max-steps 5000)
                      solution)
  (let* ((simulator (create-simulator obf-path))
         (visualizer (problem->visualizer configuration))
         (dumper (simulator::make-dumper))
         (control-structure (make-control-structure :vm-semaphore (sb-thread:make-semaphore)
                                                    :ctl-semaphore (sb-thread:make-semaphore)
                                                    :simulator simulator))
         (control-thread (let ((output *standard-output*))
                           (sb-thread:make-thread
                            (lambda ()
                              (let ((*standard-output* output))
                                (funcall control-function control-structure)))))))
    (when solution (simulator::start-dumper dumper solution simulator 247 configuration))
    (configure-simulator simulator configuration)
    (step-simulator simulator 0 0)
    (when solution (simulator::step-dumper dumper))
    (unwind-protect
         (iter (for steps from 0)
               (when (zerop (mod steps 1000))
                 (let* ((info (simulator-info simulator :hohmann))
                        (x (getf info :our-x))
                        (y (getf info :our-y))
                        (range (vector-length x y)))
                   (format t "~A steps, range ~A...~%" steps range)))
               (when (>= steps max-steps)
                 (format t "Time limit~%")
                 (finish))
               (control-structure-vm-signal control-structure)
               (control-structure-vm-wait control-structure)
               (for v-x = (control-structure-v-x control-structure))
               (for v-y = (control-structure-v-y control-structure))
               (setf (control-structure-v-x control-structure) 0.0d0
                     (control-structure-v-y control-structure) 0.0d0)
               (step-simulator simulator v-x v-y)
               (when solution (simulator::step-dumper dumper))
               (visualizer-collect visualizer simulator)
               (when (/= (getf (simulator-info simulator :hohmann) :score) 0.0d0)
                 (format t "Score at step ~A: ~A~%" steps (getf (simulator-info simulator :hohmann) :score))
                 (finish)))
      (ignore-errors (sb-thread:terminate-thread control-thread)))
    (let ((*print-case* :downcase)) (format t "Info at end:~%~S~%" (simulator-info simulator :hohmann)))
    (when solution (simulator::stop-dumper dumper))
    (visualizer-save visualizer result-path)))

(defgeneric visualizer-collect (visualizer simulator))

(defmethod visualizer-collect ((visualizer visualizer) simulator)
  (let* ((info (simulator-info simulator :hohmann))
         (our-x (getf info :our-x))
         (our-y (getf info :our-y)))
    (vector-push-extend our-x (visualizer-xs visualizer))
    (vector-push-extend our-y (visualizer-ys visualizer))))

(defmethod visualizer-collect :after ((visualizer hohmann-visualizer) simulator)
  (let* ((info (simulator-info simulator :hohmann)))
    (setf (hohmann-visualizer-target-radius visualizer)
          (getf info :target-orbit-radius))))

(defparameter *zoom* 0.1d7)

(defun vector-length (x y)
  (sqrt
   (+ (expt x 2)
      (expt y 2))))

(defgeneric visualizer-save (visualizer result-path))

(defvar *dx*)

(defvar *stream*)

(defun coord (c)
  (format nil "~F" (+ *dx* (/ c *zoom*))))

(defmethod visualizer-save :around ((visualizer visualizer) result-path)
  (let ((*dx* (+ (/ 6.357d6 *zoom*) 100)))
    (with-open-file (*stream* result-path :direction :output :if-exists :supersede)
      (format *stream* "<?xml version='1.0' encoding='UTF-8'?>~%")
      (format *stream* "<svg xmlns='http://www.w3.org/2000/svg' xmlns:xlink='http://www.w3.org/1999/xlink'>~%")
      (format *stream* "<circle cx='~A' cy='~A' r='~F' style='stroke: #000000; fill:#00ff00;' />~%" (coord 0) (coord 0) (/ 6.357d6 *zoom*))
      (call-next-method)
      (format *stream* "</svg>~%"))))

(defmethod visualizer-save ((visualizer visualizer) result-path)
  (iter (for x in-vector (visualizer-xs visualizer))
        (for y in-vector (visualizer-ys visualizer))
        (for p-x previous x)
        (for p-y previous y)
        (unless (first-iteration-p)
          (format *stream* "<line style='stroke: #000000' x1='~A' x2='~A' y1='~A' y2='~A' />~%" (coord p-x) (coord x) (coord p-y) (coord y)))))

(defmethod visualizer-save :after ((visualizer hohmann-visualizer) resul-path)
  (format *stream* "<circle cx='~A' cy='~A' r='~F' style='stroke: #ff0000; fill:transparent;' />~%" (coord 0) (coord 0)
                (/ (hohmann-visualizer-target-radius visualizer) *zoom*))
        (format *stream* "<circle cx='~A' cy='~A' r='~F' style='stroke: #0000ff; fill:transparent;' />~%" (coord 0) (coord 0)
                (/ (vector-length (aref (visualizer-xs visualizer) 0)
                                  (aref (visualizer-ys visualizer) 0)) *zoom*)))

(defstruct (meet-and-greet-visualizer (:include visualizer))
  (target-xs (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t))
  (target-ys (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t)))

(defmethod visualizer-collect :after ((visualizer meet-and-greet-visualizer) simulator)
  (let* ((info (simulator-info simulator :meet-and-greet))
         (rel-x (getf info :target-rel-x))
         (rel-y (getf info :target-rel-y))
         (x (getf info :our-x))
         (y (getf info :our-y))
         (target-x (- x rel-x))
         (target-y (- y rel-y)))
    (vector-push-extend target-x (meet-and-greet-visualizer-target-xs visualizer))
    (vector-push-extend target-y (meet-and-greet-visualizer-target-ys visualizer))))

(defun array-last-element (array)
  (aref array (1- (length array))))

(defmethod visualizer-save :after ((visualizer meet-and-greet-visualizer) result-path)
  (iter (for x in-vector (meet-and-greet-visualizer-target-xs visualizer))
        (for y in-vector (meet-and-greet-visualizer-target-ys visualizer))
        (for p-x previous x)
        (for p-y previous y)
        (unless (first-iteration-p)
          (format *stream* "<line style='stroke: #ff0000' x1='~A' x2='~A' y1='~A' y2='~A' />~%" (coord p-x) (coord x) (coord p-y) (coord y))))
  (format *stream* "<circle cx='~A' cy='~A' r='6' style='stroke:#000000; fill:#000000' />~%"
          (coord (array-last-element (visualizer-xs visualizer))) (coord (array-last-element (visualizer-ys visualizer))))
  (format *stream* "<circle cx='~A' cy='~A' r='3' style='stroke:#ff0000; fill:#ff0000' />~%"
          (coord (array-last-element (meet-and-greet-visualizer-target-xs visualizer)))
          (coord (array-last-element (meet-and-greet-visualizer-target-ys visualizer)))))
