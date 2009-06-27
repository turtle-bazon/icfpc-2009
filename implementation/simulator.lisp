(require 'iterate)
(require 'ieee-floats)

(defpackage :simulator
  (:use :cl :iterate)
  (:export :create-simulator))

(in-package :simulator)

;;; A package for VM simulator
;;; requires: Iterate and IEEE-FLOATS
;;; IEEE-FLOATS: http://common-lisp.net/~sionescu/files/ieee-floats-0.1_pre20080922.tar.bz2

(defstruct instruction op r-1)

(defstruct (d-instruction (:include instruction)) r-2)

(defstruct (s-instruction (:include instruction)) imm)

(defstruct orbit-vm
  instruction-memory
  data-memory
  pc
  status-reg
  inputs
  outputs)

(defun d-op-code->op (op-code)
  (ecase op-code
    (1 :add)
    (2 :sub)
    (3 :mult)
    (4 :div)
    (5 :output)
    (6 :phi)))

(defun s-op-code->op (op-code)
  (ecase op-code
    (0 :noop)
    (1 :cmpz)
    (2 :sqrt)
    (3 :copy)
    (4 :input)))

(defun cmpz-imm-code->op (imm-code)
  (ecase imm-code
    (0 :ltz)
    (1 :lez)
    (2 :eqz)
    (3 :gez)
    (4 :gtz)))

(defun parse-d-instruction (qword)
  (let* ((op-code (ldb (byte 4 28) qword))
         (op (d-op-code->op op-code))
         (r-1 (ldb (byte 14 14) qword))
         (r-2 (ldb (byte 14 0) qword)))
    (make-d-instruction :op op :r-1 r-1 :r-2 r-2)))

(defun parse-s-instruction (qword)
  (let* ((op-code (ldb (byte 4 24) qword))
         (op (s-op-code->op op-code))
         (r-1 (ldb (byte 14 0) qword))
         (imm (ldb (byte 10 14) qword)))
    (make-s-instruction :op op :r-1 r-1 
			:imm (case op
			       (:cmpz (cmpz-imm-code->op 
					(ldb (byte 3 7) imm)))
			       (otherwise imm)))))

(defun parse-instruction (qword)
  (if (/= 0 (ldb (byte 4 28) qword))
      (parse-d-instruction qword)
      (parse-s-instruction qword)))

(defun read-integer-little-endian (stream length)
  (iter (with value = 0)
        (repeat length)
        (for shift from 0 by 8)
        (setf value (logior value (ash (read-byte stream) shift)))
        (finally (return value))))

(defun parse-double (word)
  (ieee-floats:decode-float64 word))

(defun parse-obf (path)
  (with-open-file (stream path :element-type '(unsigned-byte 8))
    (iter (with instruction-word = nil)
          (with data-word = nil)
          (while (< (file-position stream) (file-length stream)))
          (for counter from 0)
          (if (evenp counter)
              (setf data-word (read-integer-little-endian stream 8)
                    instruction-word (read-integer-little-endian stream 4))
              (setf instruction-word (read-integer-little-endian stream 4)
                    data-word (read-integer-little-endian stream 8)))
          (collect (parse-instruction instruction-word) into instructions result-type (vector instruction))
          (collect (parse-double data-word) into datas result-type (vector double-float))
          (finally (return (values instructions datas))))))

(defstruct simulator ip instructions data status input output)

(defun create-simulator (obf-file-path)
  (multiple-value-bind (instructions numbers) (parse-obf obf-file-path)
    (make-simulator :ip 0
                    :instructions instructions
                    :data numbers
                    :status 0
                    :input (make-array (expt 2 14) :element-type 'double-float :initial-element 0.0d0)
                    :output (make-array (expt 2 14) :element-type 'double-float :initial-element 0.0d0))))
