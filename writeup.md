## IMPLEMENTATION

The source code for this project is comprised by:
  * `PID.h`/`PID.cpp`, which declare and define a class PID with the functionality of a classical PID controller.
  * main.cpp, with the implementation of the binary that drives the car in the simulator.
  
Section `CONFIGURABLE PARAMETERS` in `main.cpp` contains all configurable parameters.

In particular, note that the car can be driven in two configurations, depending on the value of flag `optimize`:
 * If `optimize:=true`, a PID parameter optimization for the steering angle control is run, starting with a initial guess given by `Kp`, `Ki` and `Kd`.
 * If `optimize:=false`, the car is driven undefinitely with PID controlle parameters for steering angle given by `Kp`, `Ki` and `Kd`.

The code submitted for the project has `optimize` set to `false` so that the car is driven with my selected PID controller parameters without any code modification.

## APPROACH

The vehicle takes two control inputs: steering angle and throttle.

* The steering angle is calculated with a PID controller that takes as input the cross-track error (CTE). Details are explained on the next secion "REFLECTION".
* The throttle is calculated with a PD controller that takes as input the difference between measured and desired speed. The desired speed is set inversely proportional to the steering angle, with a minimum value. For the code in the project submission, target speed goes from 50 mph for null steering angle down to 15 mpg for steering angle equal or larger than 17.5 deg. 

## REFLECTION

Effect of P, I and D components of the steering angle controller on the CTE:
 * The P component steers the car in a direction opposite to CTE, hence helps bring the vehicle near the center of the lane. Low P values are insufficient to keep the car within the lane in sharp curves, such as the one after the bridge. High P values, on the other hand, can result in oscillations in the steering angle and the CTE during straight paths.
 * The D component steers the car so as to oppose rate of changes in the CTE, hence helps the car follow the curvature of the road, but does not brig the car towards the lane centre if the car velocity is aligned with the road lane. Because the D component is proportional to *changes* in CTE rather than the CTE itself, it also reacts more rapidly than the P component to CTE deviations. A proper combined tuning of P and D improves reaction time while moderating oscillations.
 * The I component is useful in systems susteptible to persistent disturbances or modeling errors. If the simulator track was e.g. a circle, the constant curvature of the road could have been seen as a constant external disturbance to linear movement (it actually is if we consider the centrifugal force in the vehicle-attached frame), and the I component would have come up handy in countering the car bias to drive towards the circle tangent. In the project track, however, there are short and long curves to both hand-sides, hence the controler can build up large integral errors that can actually steer the car in the wrong direction if the curvature of a road stretch has opposite sign to the stretch in which the integral error was built up. Moreover, from the experience running the binary, the simulator does not seem to model any substantial external disturbances that would call for a I component in the PID.
 
 Choice of hyperparameters for steering angle control
 
 * First, I drove the car in the simulator with manually guesses, starting with a P-only controller, and later expanding to a PD-controller.
 * Then, I run the code in 'twiddle parameter optimization' mode starting with the manual tunnig as initial guess. As cost function, I selected the sum of squared CTE error over 1500 cycles (a bit passed the curve after the bridge). I performed optimizations both with null I component and with tunneable I component.
 * Twiddle helped reduce the sum of squared CTE by increasing the P and D components, but only at the expense of sudden steering angle (and throttle) actuations. In fact, it is possible to bring the cost function to extremelly low levels with a very responsive actuator, but it would be extremelly unconfortable for the passengers, bad for the mechanics and downright dangearous.
 * Therefore, I played a bit more manually with the tunning arrived at with Twiddle, this time setting the I component to zero, since it did not seem to help much and in fact probably contributed to "memorization" or "overfitting" to the simulator track.
 * As a last step, I increased the maximum speed until I found a good compromise of safe drive around the circuit, high average speed and reasonably smooth acutations. That is the tunning in the project submission.
