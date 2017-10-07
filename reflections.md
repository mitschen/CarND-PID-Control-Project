# CarND-PID Control Project
## Reflections

[//]: # (Image References)
[video1]: ./movie/PID_startParam.mp4
[video2]: ./movie/PID_betterStartparam.mp4
[video3]: ./movie/PID_onlyP.mp4

###Overview:
This project is about implementing a PID controller that should interact with the Udacity car simulator. The simulator is sending the distance to the middle of the lane (as cross track error) as well as speed, steering angle and throttle. The task of the PID controller is to identify a valid steering angle to make sure, that the car always stays in the lane.

###Implementation:
The PID-Controller is implemented in the corresponding PID.cpp/ PID.hpp files. During C'tor a a list of set of `TControlCoefficient` is expected. The `TControlCoefficient` describes the type of controller we want to support (P, I or D) as well as the corresponding coefficient for calculation of the error. Internally this list is stored as an array of coefficients and fuction pointers, which will be called once after another to calculate the overall error value. The configuration via vector allows a very flexible configuration of the PID-controller.
The input for the steering value is fetched by the `TotalError()` method which in essence returns the error calculated during `UpdateError()`.  parameter is expected which represents the coefficients using for the P, the I and the D part of the controller. These coefficients are used to identify the resulting steering angle.
The major work happens in the `UpdateError()` method. Depending on the controller type an error fraction is calculated and added to the overall error stored in the member `m_currentError`. The controllers calculate the error the following way:

* P: simply multiply the cte with the coefficient
* I: multiply the coefficient with the sum of all collected cte (the sum in stored in the member `m_cteSum`)
* D: multiply the coefficient with the difference of cte(t) and cte(t-1) (cte(t-1) is stored in member `m_ctePrio`)


I've started the first testrun with the parameters suggested in lesson 16-11 and figured out, that these could not be directly applied to our simulation. The lesson 16-11 suggested the settings P=0.2, D=3, I=0.004 the result of the  The problem here was, that the car reacts really lazy when crossing the middle line. I've played around manually with the parameters and came really fast to the conclusion, that the I-part was too high. The high I-coefficient leads to the situation, that the error was still very high (and therefore the steering value) even though the car already crossed the middle line. This results in the fact, that the I-part could be seens as a kind of lazy control. Here' a video of the first few seconds with 
![first starting parameters with to large I coefficient][video1]

So I changed the startparameters for the I-coefficient, keep the remaining as they are:
* P = 0.2
* I = 0.0004
* D = 3

The result is shown here 
![better I-coefficient][video2].

At the beginning there is a slight threshing around the middle lane, but then the car almost stabilizes. 

So what is the impact of the different coefficients in the PID-control:
* P causes a direct reaction on an high cte. That means hard countersteer to the given error direction. A simple P-control therefore tends to threshing as you can see here: ![only P][video3].
* D causes a reaction on differences to the previous error. In other words, the reaction is given by the change. The combination of P and D therefore reduces the effect of threshing as it is given with the P. In this project i've made the experience that the difference between an PD and the PID control is almost neglible - both fulfill the requirements of the track very well.
* I causes a reaction based on, let's say the history of the error. In essence the I part leads to another damping of the results of the P and D part. 

###Parameter optimization
Even though the chosen parameters already worked very well, I've tried to optimize them using the twiddle algorithm introduces in lesson 16-13. In contrast to the setup in the lesson, we do not have a continous constant control path - but due to the fact that the car goes the same track again and again, we are very close to that. So every 7000 measurement points (which reflects more or less one loop of the whole track) I start a new iteration, applying twiddle to one of the three coefficients of the PID control. The twiddle algorithm itself is implemented in the `applyTwiddle()` method which is called everytime the `UpdateError()` method is called. As quality criterior of the twiddled parameters, I'm using the square of the CTE. The twiddle optimization is an optional part of the PID class and must be explicitly enabled by `setTwiddle(...)` method (see inline documentation for further details).

Unfortunately this `online` twiddle algorithm has two big disadvantages:
* in contrast to the setup of lesson 16-13, optimization takes very very long because the input is the measurement-update of the simulator
* furthermore, the control path and the measurement points aren't 100% the same

So the resulting criterion of "is a parameter combination better or worse" is not 100% compareable.

Anyhow run the algorithm for a while and came up with the final coefficients I'm using now in the PID control:
* P = 0.242832
* I = 0.00048777
* D = 3.85568


To verify the chosen parameters I've compared the twiddle quality criterior for the first 7000 measurement point (which I'm expecting to be almost compareable). The result is:
* initial coefficient set: error = 1489
* final coefficient set: error = 1176
* final coefficient PD only: error = 1460


## PID-Usage and Summary
There is still place for improvement. Currently i'm limiting the throttle and therefore the speed by two easy tweaks:
* if the resulting absolute steer_value is higher than 0.7, I drop the throttle.
* in case that we've reached 30mph, I drop the throttle as well

This allows me first of all to have more time reacting on high steering values (speed will not increase) and furthermore limits the speed to an almost constant value fo 30mph.
This could again be realized in a more elegant way by using another PID controller to identify the throttle value (and change the speed) based on CTE.
Concerning optimization, the online twiddle didn't work that well. So maybe a static testscenario based on speed, cte and steps would allow to tweak the PID-coefficients much better.
