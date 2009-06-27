(require :simulator "simulator")
(require :simulator "controller")

(in-package :simulator)

(defun problem-number (problem)
  (ecase problem
    (:hohmann 1)
    (:meet-ang-greet 2)))

(defun display-hohmann-info (info)
  (format t "R:~a, X:~a, Y:~a, F:~a~%"
	  (distance-to-earth (getf info :our-x) (getf info :our-y))
	  (getf info :our-x)
	  (getf info :our-y)
	  (getf info :fuel-remaining)
	  ))

(defun problem (problem scenario)
  (let ((simulator (create-simulator (format nil "../task/bin~a.obf" (problem-number problem)))))
    (configure-simulator simulator scenario)
    (dotimes (i 100)
      (step-simulator simulator 0 0)
      (ecase problem
	(:hohmann (display-hohmann-info (simulator-info simulator problem))))
      )))
