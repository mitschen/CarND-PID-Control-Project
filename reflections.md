# CarND-PID Control Project
## Reflections

###Overview:
This project is about implementing a PID controller that should interact with the Udacity car simulator. The simulator is sending the distance to the road-border as well as speed, steering angle and throttle. The task of the PID controller is to identify a valid steering angle to make sure, that the car always stays in the lane.

###Implementation:
The PID-Controller is implemented in the corresponding PID.cpp/ PID.hpp files. During C'tor a set of parameter is expected which represents the coefficients using for the P, the I and the D part of the controller. These coefficients are used to identify the resulting steering angle.

PID class has two methods:
* UpdateError is used to push the error identified by the simulator to the PID-controller
* TotalError is used to derive the resulting steering angle based on the errors.

UpdateError:
While calling this method the first time, the PID controller identifies a bias for the CTE (=error). The intention of the PID is to stay in the middle of the road and not on the left or right corner of the lane. The simulator starts in the middle - therefore the initial CTE represents the bias. During subsequent calls of the UpdateError method, the bias is always substracted from the passed cte value. The result