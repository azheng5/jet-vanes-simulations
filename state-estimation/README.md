This page is a guide for code development, tuning testing of the EKF (Extended Kalman Filter) module in the jet-vanes-simulations repo. 

The main user interface with the EKF will be via `main_ekf.m`, notably by manually specifying/tuning elements in the ekf struct (i.e. covariance matrices, sensor noise, trajectory file names).

## Prerequisite Knowledge
- **variance (sigma^2)**: A measure of the spreading of a data set from its mean. Higher variances means high uncertainty, while low variances means low uncertainty.
- **standard deviation (sigma)**: Square root of the variance
- **normal distribution (gaussian)**: a random variable that follows the normal distribution is one which:
   - <img width="314" alt="image" src="https://github.gatech.edu/storage/user/71356/files/90195e43-6cc2-43d9-8110-8966d601748a">
   - 68% of values lie within 1 stdev of the mean
   - 95% of values lie within 2 stdev of the mean
   - 99.8% of values lie within 3 stdev of the mean
   - NOTE: A stdev of 1 is a standard normal gaussian
- **covariance**: A measure of the strength of correlation between two random variables. Positive covariance means both random variables tend to be high or low at the same time, while negative covariance means both random variables tend to be opposite of each other at the same time. Close to zero covariance means two random variables have no correlation
- **covariance matrix**: Square matrix that reps the covariance btw each pair of elements in a multid random variable. All off diagonal terms are covariances, while diagonal terms are variances sigma^2.
   - <img width="345" alt="image" src="https://github.gatech.edu/storage/user/71356/files/5c1ab287-2b3e-49d3-8213-c492bff2144a">
   - Its sometimes a good assumption to make your covariances (all off diagonal terms) zero for 6DOF rocket state estimation (ie different translational or rotational variables have no relation to each other).
- In the EKF, state estimates and measurements are normally distributed multidimensional random variables. The actual values of estimates and measurements in the EKF are actually expected values (means), and at any point in time they each have an associated covariance matrix that describes the uncertainty of each value. The estimate uncertainty is referred to as the estimate covariance P, and the measurement uncertainty is referred to as the measurement covariance R. Each element in a covariance matrix represents a variance or covariance, NOT a stdev.
- In real life, we have no way of measuring the estimate error or measurement error. Instead, we use the estimate uncertainty and measurement uncertainty to judge how accurate our EKF is from the true state.

## Input and Output Files
- The input file is a generated csv from the `trajectory-generation` repo that represents the true trajectory for the EKF to utilize and compare itself to.
- The outputs are plots that the user specifies in the plotting subsection of `ekf_main.m`

## Estimate Covariance P
- Definition: The squared uncertainty of an estimate
- P gets propagated forward in time as the EKF loop runs. The closer our estimate is to the true state, the lower our estimate covariance is, so an accurate EKF should have a P that converges to zero as it gets propagated in time. A completely failed EKF should have a P that diverges to infinity.
- If your initial guess is imprecise, set a high estimate covariance.

## Process Noise Covariance Q
- Definition: the noise that the dynamic model does not account for
- Process noise is due to effects that the ideal dynamic model does not account for. This is measured using the process noise covariance, which is added to the covariance extrapolation equation. (the Q term only shows up in the covariance extrapolation equation). This means that Q is always inhibiting the decrease of P. High process noise covariance Q means its harder for estimate covariance to converge, while low Q means its easier for P to converge.
- A low Q means a highly accurate dynamic model.
- A constant gap between the true value and estimate is known as a **lag error**. There are two potential causes for a lag error: an incorrect dynamic model or a very low process noise (diagonal terms of Q), while the true state fluctuations are more significant.
   - This makes sense bc Q is just an additional constant term to the estimate covariance. A high Q would cause a high estimate uncertainty.
   - Note: since lag error is a constant error, the estimate curve should have the same slope as the true value curve.
- Process noise can be dependent along different states.
   - For example, a variance in acceleration causes a variance in velocity and position. If your rocket is highly maneuvering, then you might have a high variance in the acceleration term, then lower variance in velocity, then even lower in position.
- Q could change mid flight in certain trajectories or conditions
   - When a sudden maneuver is performed in flight that the dynamic model cannot accurately model, the process noise would increase, meaning Q should be increased to account for it

## Measurement Covariance R
- Definition: squared measurement uncertainty
- Measurement noise is due to inaccuracies in the sensor hardware. The variances and covariances in a measurement covariance matrix can be determined from the datasheet or determined experimentally.
- The variances in R are just the variances of the sensor measurements. For example, the variance of the x position state variable in R is just the variance of the GPS x measurment.
