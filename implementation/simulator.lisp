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

(defun simulator-data-cell (simulator index)
  (aref (simulator-data simulator) index))

(defun (setf simulator-data-cell) (new-value simulator index)
  (setf (aref (simulator-data simulator) index) new-value))

(defun simulator-input-port (simulator index)
  (aref (simulator-input simulator) index))

(defun (setf simulator-input-port) (new-value simulator index)
  (setf (aref (simulator-input simulator) index) new-value))

(defun simulator-output-port (simulator index)
  (aref (simulator-output simulator) index))

(defun (setf simulator-output-port) (new-value simulator index)
  (setf (aref (simulator-output simulator) index) new-value))

(defun simulator-current-data-cell (simulator)
  (simulator-data-cell simulator (simulator-ip simulator)))

(defun (setf simulator-current-data-cell) (new-value simulator)
  (setf (simulator-data-cell simulator (simulator-ip simulator)) new-value))

(defun create-simulator (obf-file-path)
  (multiple-value-bind (instructions numbers) (parse-obf obf-file-path)
    (make-simulator :ip 0
                    :instructions instructions
                    :data numbers
                    :status 0
                    :input (make-array (expt 2 14) :element-type 'double-float :initial-element 0.0d0)
                    :output (make-array (expt 2 14) :element-type 'double-float :initial-element 0.0d0))))

(defgeneric instruction->sexp (op ip instruction simulator-var))

(defun simulator->function (simulator)
  (compile nil (simulator-code->lambda simulator)))

(defun simulator-code->lambda (simulator)
  (let ((simulator-var (gensym "SIMULATOR")))
    `(lambda (,simulator-var)
       ,@(iter (for instruction in-vector (simulator-instructions simulator))
               (for ip from 0)
               (for op = (instruction-op instruction))
               (for sexp = (instruction->sexp op ip instruction simulator-var))
               (collect sexp)))))

(defmethod instruction->sexp ((op (eql :add)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (+ (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))
            (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction)))))

(defmethod instruction->sexp ((op (eql :sub)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (- (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))
            (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction)))))

(defmethod instruction->sexp ((op (eql :mult)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (* (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))
            (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction)))))

(defmethod instruction->sexp ((op (eql :div)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (if (= 0.0d0 (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction)))
             0.0d0
             (/ (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))
                (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction))))))

(defmethod instruction->sexp ((op (eql :output)) ip instruction simulator-var)
  `(setf (simulator-output-port ,simulator-var ,(instruction-r-1 instruction))
         (simulator-data-cell ,simulator-var ,(d-instruction-r-2 instruction))))

(defmethod instruction->sexp ((op (eql :phi)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (simulator-data-cell ,simulator-var
                              (if (= (simulator-status ,simulator-var) 1)
                                  ,(instruction-r-1 instruction)
                                  ,(d-instruction-r-2 instruction)))))

(defmethod instruction->sexp ((op (eql :noop)) ip instruction simulator-var)
  nil)

(defun cmpz-imm->function (imm)
  (ecase imm
    (:ltz '<)
    (:lez '<=)
    (:eqz '=)
    (:gez '>=)
    (:gtz '>)))

(defmethod instruction->sexp ((op (eql :cmpz)) ip instruction simulator-var)
  (let ((op (cmpz-imm->function (s-instruction-imm instruction))))
    `(setf (simulator-status ,simulator-var)
           (if (,op (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction)) 0.0d0)
               1
               0))))

(defmethod instruction->sexp ((op (eql :sqrt)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (abs (sqrt (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))))))

(defmethod instruction->sexp ((op (eql :copy)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (simulator-data-cell ,simulator-var ,(instruction-r-1 instruction))))

(defmethod instruction->sexp ((op (eql :input)) ip instruction simulator-var)
  `(setf (simulator-data-cell ,simulator-var ,ip)
         (simulator-input-port ,simulator-var ,(instruction-r-1 instruction))))
