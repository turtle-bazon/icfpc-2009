(require :simulator "simulator")

(in-package :simulator)

(defun stype (number)
  (ecase number
    (1 :hohmann)
    (4 :meet-and-greet)))

(defun problem (number scenario)
  (let ((simulator (create-simulator (format nil "../task/bin~a.obf" number))))
    (configure-simulator simulator scenario)
    (dotimes (i 10)
      (step-simulator simulator 0 0)
      (format t "~a~%" (simulator-info simulator (stype number))))))
