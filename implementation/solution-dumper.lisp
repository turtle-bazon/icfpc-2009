(in-package :simulator)

(defstruct dumper stream stream-opened-p simulator time last-frame)

(defun write-little-endian (stream number bytes)
  (iter (repeat bytes)
        (write-byte (ldb (byte 8 0) number) stream)
        (setf number (ash number -8))))

(defun start-dumper (dumper output simulator team-id scenario-id)
  (if (streamp output)
      (setf (dumper-stream dumper) output
            (dumper-stream-opened-p dumper) nil)
      (setf (dumper-stream dumper) (open output :direction :output :element-type '(unsigned-byte 8) :if-exists :supersede)
            (dumper-stream-opened-p dumper) t))
  (setf (dumper-simulator dumper) simulator
        (dumper-time dumper) 0
        (dumper-last-frame dumper) (make-array (expt 2 14) :element-type 'double-float :initial-element 0.0d0))
  (write-little-endian (dumper-stream dumper) #xCAFEBABE 4)
  (write-little-endian (dumper-stream dumper) team-id 4)
  (write-little-endian (dumper-stream dumper) scenario-id 4))

(defun output-difference (stream timestep frame last-frame)
  (let ((differences-count (iter (for i from 0 to (- (expt 2 14) 1))
                                 (for v-1 = (aref frame i))
                                 (for v-2 = (aref last-frame i))
                                 (count (/= v-1 v-2)))))
    (unless (zerop differences-count)
      (format t "~A differences~%" differences-count)
      (write-little-endian stream timestep 4)
      (write-little-endian stream differences-count 4)
      (iter ;(for i in '(2 3 #x3e80))
	    (for i from 0 to (- (expt 2 14) 1))
            (for v-1 = (aref frame i))
            (for v-2 = (aref last-frame i))
            (unless (= v-1 v-2)
              (format t "~A => ~F~%" i v-1)
              (write-little-endian stream i 4)
              (write-little-endian stream (ieee-floats:encode-float64 v-1) 8)))
              )))

(defun step-dumper (dumper)
  (let ((frame (simulator-input (dumper-simulator dumper))))
    (output-difference (dumper-stream dumper) (dumper-time dumper) frame (dumper-last-frame dumper))
    (setf (dumper-last-frame dumper) (copy-seq frame))
    (incf (dumper-time dumper))))

(defun stop-dumper (dumper)
  (write-little-endian (dumper-stream dumper) (dumper-time dumper) 4)
  (write-little-endian (dumper-stream dumper) 0 4)
  (when (dumper-stream-opened-p dumper)
    (close (dumper-stream dumper))))
