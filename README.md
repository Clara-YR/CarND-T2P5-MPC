[//]: # (Image References)

[image0]: ./ReadmeImages/TheGlobalKinematicModel.png "Kinematic Model"
[image1]: ./ReadmeImages/PID.png "yaw_rate!=0"
[image2]: ./ReadmeImages/PID_formula.png "yaw_rate!=0"
[image3]: ./ReadmeImages/twiddle.png 
[image4]: ./ReadmeImages/vehicle_coordinate.png 


# CarND-Controls-MPC

 Install CppAD

**The Global Kinematic Model**

![alt text][image0]

```
Eigen::VectorXd globalKinematic(Eigen::VectorXd state,
								Eigen::VectorXd actuators, double dt) {
	Eigen::VectorXd next_state(state.size());

	// The next_state calculation ...
	next_state[0] = state[0] + state[3] * cos(state[2]) * dt;
	next_state[1] = state[1] + state[3] * sin(state[2]) * dt;
	next_state[2] = state[2] + state[3]/Lf * actuators[0]* dt;
	next_state[3] = state[3] + actuators[1] * dt;

	return next_state;
}
```

**3rd Polynomials**

[Reference](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl)

- Use polyfit to fit a 3rd order polynomial to the given x and y coordinates representing waypoints.

## Simulate - Predict - Select

MPC solution: optimal trajectory 

- __Simulate__ different actuator inputs
- __Predict__ the resulting trajectory
- __Select__ the trajectory with a minimize cost

If we know current state x[t] and the reference trajectory we want to follow.

- __Simulate__ -- Optimize actuator inputs
- __Predict__ -- Minimize cost of predicted trajectory
- __Select__ -- Find the lowest cost trajectory
- __Simulate__ -- Implement the very first set of actuation commands and throw away the rest of the trajectory we calculated 
- __Predict__ -- Take our new state to calculate a new optimal trajectory
- ... 

In that sense, we are constantly calculating inputs over a future horizon. That's why __MPC__(model predictive control) is sometimes called __RHC__(receding horizon control).

It's crucial that we constantly __re-evaluate__ to find the optimal actuations. 

## MPC Algorithm

### MPC Setup

**1. T = N x dt**

First, we set up everything for the Model Predictive Control loop. This consist of defining T by choosing N and dt.

- __T__ -- prediction horizon =  N*dt, the duration over which future predictions are made
- __N__ -- the number of timesteps in the horizon
- __dt__ -- how much time elapsed between actuations

**2. constraints**

Next we define the vehicle model and constraints such as actual liminations.

**3. cost**

Finally, we define the cost function.

### MPC Loop
First, we pass the current state to the Model Predictive Controller.

Next the optimization solver is called. The solver uses the initial state `[x_1, y_1, psi_1, v_1, cte_1, epsi_1]`, the model constraints and cost function to return a vector of control that minimize the cost function.

The solver we'll use is called __IPOpt__.




