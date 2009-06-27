(eval-when (:compile-toplevel :load-toplevel :execute)
  (require 'iterate)
  (require 'cxml))

(defpackage :visualizer
  (:use :cl :iter :simulator :cxml)
  (:export :visualize-run :make-initial-thrust-function))

(in-package :visualizer)

(defstruct visualizer
  (xs (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t))
  (ys (make-array 0 :element-type 'double-float :adjustable t :fill-pointer t)))

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
    (iter (until (funcall finish-predicate simulator))
          (for (values v-x v-y) = (funcall thrust-function simulator))
          (step-simulator simulator v-x v-y)
          (funcall thrust-function simulator)
          (visualizer-collect visualizer simulator))
    (visualizer-save visualizer result-path)))

(defun visualizer-collect (visualizer simulator)
  (let* ((info (simulator-info simulator :hohmann))
         (our-x (getf info :our-x))
         (our-y (getf info :our-y)))
    (vector-push-extend our-x (visualizer-xs visualizer))
    (vector-push-extend our-y (visualizer-ys visualizer))))

(defparameter *zoom* 2.5d4)

(defun visualizer-save (visualizer result-path)
  (let ((dx (+ (/ 6.357d6 *zoom*) 100)))
    (flet ((coord (c) (format nil "~F" (+ dx (/ c *zoom*)))))
      (with-open-file (stream result-path :direction :output :if-exists :supersede :element-type '(unsigned-byte 8))
        (with-xml-output (make-octet-stream-sink stream :encoding :utf-8 :indentation 2 :canonical nil)
          (with-element "svg"
            (attribute "xmlns" "http://www.w3.org/2000/svg")
            (attribute "xmlns:xlink" "http://www.w3.org/1999/xlink")
            (with-element "circle"
              (attribute "cx" (coord 0))
              (attribute "cy" (coord 0))
              (attribute "r" (format nil "~F" (/ 6.357d6 *zoom*)))
              (attribute "style" "stroke: #000000; fill:#00ff00;"))
            (iter (for x in-vector (visualizer-xs visualizer))
                  (for y in-vector (visualizer-ys visualizer))
                  (for p-x previous x)
                  (for p-y previous y)
                  (unless (first-iteration-p)
                    (with-element "line"
                      (attribute "style" "stroke: #000000;")
                      (attribute "x1" (coord p-x))
                      (attribute "x2" (coord x))
                      (attribute "y1" (coord p-y))
                      (attribute "y2" (coord y)))))))))))

(defun make-initial-thrust-function (v-x v-y)
  (let (returned)
    (lambda (simulator)
      (declare (ignore simulator))
      (if returned
          (values 0 0)
          (progn
            (setf returned t)
            (values v-x v-y))))))