(require :simulator "simulator")
(require :simulator "controller")

(in-package :simulator)

(defun problem-number (problem)
  (ecase problem
    (:hohmann 1)
    (:meet-ang-greet 2)))

(defun display-hohmann-info (info)
  (format t "R:~a, X:~a, Y:~a, F:~a, TR:~a, S:~a~%"
	  (distance-to-earth (getf info :our-x) (getf info :our-y))
	  (getf info :our-x)
	  (getf info :our-y)
	  (getf info :fuel-remaining)
	  (getf info :target-orbit-radius)
	  (getf info :score)
	  ))

(defun step-V (info step)
  (let* ((r1 (distance-to-earth (getf info :our-x)
			       (getf info :our-y)))
	(r2 (getf info :target-orbit-radius))
	(x (getf info :our-x))
	(y (getf info :our-y))
	(tga (/ x (- 0 y)))
	(cosa (sqrt (/ 1 (+ 1 (expt tga 2)))))
	(sina (sqrt (- 1 (expt cosa 2)))))
  (case step
    (0 
     (let ((vval
     (* (sqrt (/ +gravitational-parameter+
		   r1))
	  (- (sqrt (/ (* 2 r2)
		      (+ r1 r2)))
	     1))))
       (format t "~a~%" vval)
       (values (* vval cosa) (* vval sina))
       ))
    (otherwise (values 0 0))
    ))
  )

(defun problem (problem scenario)
  (let ((simulator (create-simulator (format nil "../task/bin~a.obf" (problem-number problem)))))
    (configure-simulator simulator scenario)
    (step-simulator simulator 0 0)
    (dotimes (i 1)
      (multiple-value-bind (vx vy)
	(step-V (simulator-info simulator problem) i)
	(step-simulator simulator (- 0 vx) (- 0 vy)))
      (ecase problem
	(:hohmann (display-hohmann-info (simulator-info simulator problem))))
      )))
