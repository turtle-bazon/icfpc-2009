(eval-when (:compile-toplevel :load-toplevel :execute)
  (require 'iterate))

(defpackage :visualizer
  (:use :cl :iter :simulator)
  (:export :visualize-run :make-initial-thrust-function))

(in-package :visualizer)

(defstruct visualizer
  (xs (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t))
  (ys (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t))
  target-radius)

(defun make-steps-finish-predicate (steps)
  (let ((counter 0))
    (lambda (simulator)
      (declare (ignore simulator))
      (incf counter)
      (> counter steps))))

(defun noop-thrust-function (simulator)
  (declare (ignore simulator))
  nil)

(defun visualize-run (result-path obf-path configuration
                      &key
                      (finish-predicate 1000)
                      (thrust-function #'noop-thrust-function))
  (when (integerp finish-predicate)
    (setf finish-predicate (make-steps-finish-predicate finish-predicate)))
  (let ((simulator (create-simulator obf-path))
        (visualizer (make-visualizer)))
    (configure-simulator simulator configuration)
    (step-simulator simulator 0 0)
    (iter (until (funcall finish-predicate simulator))
          (for (values v-x v-y) = (funcall thrust-function simulator))
          (step-simulator simulator v-x v-y)
          (visualizer-collect visualizer simulator))
    (visualizer-save visualizer result-path)))

(defun visualizer-collect (visualizer simulator)
  (let* ((info (simulator-info simulator :hohmann))
         (our-x (getf info :our-x))
         (our-y (getf info :our-y)))
    (vector-push-extend our-x (visualizer-xs visualizer))
    (vector-push-extend our-y (visualizer-ys visualizer))
    (setf (visualizer-target-radius visualizer)
          (getf info :target-orbit-radius))))

(defparameter *zoom* 2.5d4)

(defun vector-length (x y)
  (sqrt
   (+ (expt x 2)
      (expt y 2))))

(defun visualizer-save (visualizer result-path)
  (let ((dx (+ (/ 6.357d6 *zoom*) 100)))
    (flet ((coord (c) (format nil "~F" (+ dx (/ c *zoom*)))))
      (with-open-file (stream result-path :direction :output :if-exists :supersede)
        (format stream "<?xml version='1.0' encoding='UTF-8'?>~%")
        (format stream "<svg xmlns='http://www.w3.org/2000/svg' xmlns:xlink='http://www.w3.org/1999/xlink'>~%")
        (format stream "<circle cx='~A' cy='~A' r='~F' style='stroke: #000000; fill:#00ff00;' />~%" (coord 0) (coord 0) (/ 6.357d6 *zoom*))
        (format stream "<circle cx='~A' cy='~A' r='~F' style='stroke: #ff0000; fill:transparent;' />~%" (coord 0) (coord 0)
                (/ (visualizer-target-radius visualizer) *zoom*))
        (format stream "<circle cx='~A' cy='~A' r='~F' style='stroke: #0000ff; fill:transparent;' />~%" (coord 0) (coord 0)
                (/ (vector-length (aref (visualizer-xs visualizer) 0)
                                  (aref (visualizer-ys visualizer) 0)) *zoom*))
        (iter (for x in-vector (visualizer-xs visualizer))
              (for y in-vector (visualizer-ys visualizer))
              (for p-x previous x)
              (for p-y previous y)
              (unless (first-iteration-p)
                (format stream "<line style='stroke: #000000' x1='~A' x2='~A' y1='~A' y2='~A' />~%" (coord p-x) (coord x) (coord p-y) (coord y))))
        (format stream "</svg>~%")))))

(defun make-initial-thrust-function (v-x v-y)
  (let (returned)
    (lambda (simulator)
      (declare (ignore simulator))
      (if returned
          (values 0 0)
          (progn
            (setf returned t)
            (values v-x v-y))))))