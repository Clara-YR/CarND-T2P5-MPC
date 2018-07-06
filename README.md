[//]: # (Image References)

[image0]: ./ReadmeImages/vehicle_coordinate.png 
[image1]: ./ReadmeImages/MPC_setup.png "MPC Setup"
[image2]: ./ReadmeImages/MPC_loop.png "MPC Loop"
[image3]: ./ReadmeImages/Model.png 


# CarND-Controls-MPC

The project structure is a loop of __Simulate - Predict - Select__:

- Simulate different actuator inputs
- Predict the resulting trajectory
- Select

It's crucial that we constantly __re-evaluate__ to find the optimal actuations. 

**Catalog**

Part 1 __Code in main.cpp__
	
- 1.1 [Convert Coordinate and Unit](#coordinate_unit)
- 1.2 [Polynominal Order](#polyfit)
- 1.3 [Calculate cte and e𝜓](#cte_epsi)
- 1.4 [Latency](#latency)
- 1.5 [Calculate Steering Angle and Throttle](#steering_throttle)
- 1.6 [Visualization](#visualization)

Par 2 __Code in MPC.cpp__

- 2.0 [Variables in MPC.cpp](#variables) 
- 2.1 [Set N & dt](#N&dt)
- 2.2 [Start Index](#start_index)
- 2.3 [`fg` in `FG_eval`](#fg)
- 2.4 [Model Variables `vars`](#model_variables)
- 2.5 [Constraints](#constraints)
- 2.6 [Limits for Variables](#variable_limits)
- 2.7 [IPOpt Returns Actuator Values](#ipopt)
- 2.8 [Tuning MPC](#tune)

## Part 1. Code in main.cpp


<a name="coordinate_unit"></a>
### 1.1 Convert Coordinate and Unit
My reviewers suggested me to convert `ptsx` and `ptsy` from map to car coordinates to make the computation a bit easier since `px`, `py` and `psi` in car coordinates will all be 0.

![alt text][image0] 

```
// convert from map to coordinates, px, py, psi = 0
for (int i=0; i<ptsx.size(); i++) {

  // shift x,y of car position to 0
  double x_shift = ptsx[i] - px;
  double y_shift = ptsy[i] - py;

  // shift car reference angle to 90 degree
  ptsx[i] = x_shift * cos(0 - psi) - y_shift * sin(0 - psi);
  ptsy[i] = x_shift * sin(0 - psi) + y_shift * cos(0 - psi);
}
```

The unit of `j[1]["speed"]` is miles/hour.

- 1 miles/hour = 1609meters / 3600seconds = 0.44704 m/s

Hence for best accuracy, convert the velocity to m/s multiplying by __0.44704__ and doing all subsequent computations with this value.

```
double v = j[1]["speed"];
// for best accuracy, convert the velocity to m/s
v = v * 0.44704;
```

<a name="polyfit"></a>
### 1.2 Polynomial Order

__NOTE__: `ptsx` and `ptxy` are in `vector` form, but `polyfit()` requires inputs in `Eigen::VectorXd` form. So firstly we need to do as below:

```
double* ptrx = &ptsx[0];
Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
double* ptry = &ptsy[0];
Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
```

The reference trajectory is typically passed to the control block as a polynomial. This polynomial is usually __3rd order__, since third order polynomials will fit trajectories for most roads. 

```
// fit the polynominal to the waypoints(in car coordinate)
auto coeffs = polyfit(ptsx_c, ptsy_c, 3);
```

<a name="cte_epsi"></a>
### 1.3 Calculate cte and e𝜓

Use `polyfit()` to calculate `coeffs`, [reference link](https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl).

__cte__ calculation in car coordinate can be simplified as below:

```
cte = f(x) - y
 	= polyeval(coeffs, px) - py 
	= polyeval(coeffs, 0) - 0 
	= polyeval(coeffs, 0)
```	

__e𝜓__ can be calculated as the tangential angle of the polynominal f evaluated at xt, therefore e𝜓 calculation can be simplified as below:

```
e𝜓  = 𝜓 - 𝜓des
	= 0 - arctan(f'(x))
	= - arctan(f'(x))
since f(x) = a0 + a1 * x + a2 * x^2 + a3 * x^3
arctan(f'(x)) = arctan(a1 + 2x * a2 + 3x^2 * a3)
thus when px = 0 we can get
e𝜓 = - arctan(a1)
```
My code is:

```
// calculate initial cross track error and orientation error values
double cte = polyeval(coeffs, 0);
double epsi = - atan(coeffs[1]);
```


<a name="latency"></a>
### 1.4 Lactency

In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

In order to handle latency, I predict the state of the car 100ms in the future before passing it to the solver:

```
// In order to handle latency, predict the state of
// the car 100ms in the future before passing it to the solver
const double latency_dt = 0.1;
const double Lf = 2.67;
px = v * cos(psi) * latency_dt;  // px = 0 in car coordinate
py = v * sin(psi) * latency_dt;  // py = 0 in car coordinate
psi = v * delta / Lf * latency_dt;  // psi = 0 in car coordinate
double v_ = v + a * latency_dt;
cte = cte + v * sin(epsi) * latency_dt;
epsi = epsi - v * delta / Lf * latency_dt;  // psi - psi_des = epsi in car coordinate

Eigen::VectorXd state(6);
state << px, py, psi, v_, cte, epsi;
```

__NOTE__: variable `v_` is created to store the prediciton of v in 100ms. If I use `v = v + a * lantency_dt;` directly, `v` used to calculate predictions of `cte` and `epsi` is actually the prediction of `v` rather than the current value.

Thus I need to get `delta` and `a` as below:

```
double delta = j[1]["steering_angle"];
double a = j[1]["throttle"];
```


<a name="steering_throttle"></a>
### 1.5 Calculate Steering Angle and Throttle

<a name="visualization"></a>
### 1.6 Visualization

We can display these connected point paths in the simulator by sending a list of optional x and y values to the `mpc_x`, `mpc_y`, `next_x`, and `next_y` fields in the C++ main script.

The `mpc_x` and `mpc_y` variables display a line projection in green. The `next_x` and `next_y` variables display a line projection in yellow.

```
double poly_inc = 2.5;
int num_points = 25;
for (int i=1; i<num_points; i++) {
  next_x_vals.push_back(poly_inc*i);
  next_y_vals.push_back(polyeval(coeffs, poly_inc*i));
}
```


## Part 2. Code in MPC.cpp

MPC (model predictive controller)

![alt text][image1]

<a name="variables"></a>
### 2.0 Variables in MPC.cpp

`FG_eval` class

- `FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }`
	- `coeffs`-- are the coefficients of the fitted polynominal 	
- `void operator()(ADvector& fg, const ADvector& vars)`	
	- `fg`-- where the cost function and vehicle model constraints is defined
	- `vars`-- is the vector of variables (from the previous section)

`vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)`

- `state`-- the initial state `[x0, y0, 𝜓0, v0, cte0, e𝜓0]`
- `coeffs`-- are the coefficients of the fitted polynominal 


<a name="N&dt"></a>
### 2.1 Set N & dt

First, we set up everything for the Model Predictive Control loop. This consist of defining T by choosing N and dt.

__T__ -- prediction horizon T = N x dt, the duration over which future predictions are made

__N__ -- the number of timesteps in the horizon

___dt___ -- how much time elapsed between actuations

T should be as large as possible, while dt should be as small as possible.
In the case of driving a car, T should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

```
// TODO: Set the timestep length and duration
size_t N = 25;
double dt = 0.05;
```

<a name="start_index"></a>
### 2.2 Start Index

_code in MPC.cpp line 8~20_

|variable in `vars`|x|y|𝜓|v|cte|e𝜓|𝛿|a|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|index|0|N|2N|3N|4N|5N|6N|7N-1|

<a name="fg"></a>
### 2.3 `fg` in `FG_eval`

`fg` is the vector of constraints.


#### 2.3.1 cost defined in `fg[0]`

The table below suggests fuctions of each cost costituent:

|Cost Costituent|Function|
|:---:|:---|
|cte², e𝜓²|The cost based on the reference state.|
|(v - v_ref)²|Punish the car if it stop or halt|
|𝛿², a²|Minimize the use of actuators|
|[𝛿(t+1)-𝛿(t)]²<br>[a(t+1)-a(t)]²|Minimize the value gap between sequential actuations|

|state cost|actuator cost|value gap cost|
|:---:|:---:|:---:|
|cte², e𝜓², (v - v_ref)²|𝛿², a²|[𝛿(t+1)-𝛿(t)]², [a(t+1)-a(t)]²|


Thus I define the cost as below:

```
// The part of the cost based on the reference state.
for (int t=0; t < N; t++) {
  // cost += cte^2 + epsi^2 + (v - ref_v)^2
  fg[0] += CppAD::pow(vars[cte_start + t], 2);
  fg[0] += CppAD::pow(vars[epsi_start + t], 2);
  fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
}

// Minimize the use of actuators.
for (int t=0; t < N-1; t++) {
  // cost += delta^2 + a^2
  fg[0] += CppAD::pow(vars[delta_start + t], 2);
  fg[0] += CppAD::pow(vars[a_start + t], 2);
}

// Minimize the value gap between sequential actuations.
for (int t=0; t < N-2; t++) {
  // cost += D_delta^2 + D_a^2
  fg[0] += 500 * CppAD::pow((vars[delta_start + t+1] - vars[delta_start + t]), 2);  // TUNE here !
  fg[0] += CppAD::pow((vars[a_start + t+1] - vars[a_start + t]), 2);
}
```

#### 2.3.2 vehicle constraints defined in `fg[1~(N-1)]`



##### 2.3.2.1 Set the 1st value for each variable constraint

Assuming all variable at time t < 0 is 0, we can get __state[1] - prediction[1|0] = state[1]__. The `fg` vector is 1 element larger than `vars` since `fg[0]` stores the cost value. 

|constraints for|x|y|𝜓|v|cte|e𝜓|𝛿|a|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|1st index in `fg`|1+x_start|1+y_start|1+psi_start|1+v_start|1+cte_start|1+epsi_start|1+delta_start|1+a_start|
|1st value|var[x_start]|var[y_start]|var[psi_start]|var[v_start]|var[cte_start]|var[epsi_start]|

##### 2.3.2.1 Set the 2nd ~ Nth value for each variable 

![alt text][image3]

Having the Global Kinematic Model above, we can use reference state[t] to calculate prediction[t+1|t]. Then calculate the difference between reference state[1] and prediction[1|0] as constraints.

```
// constraints = state[t+1] - prediction[t+1|t]
fg[1 + x_start + t] = x_1 - (x_0 + v_0 * CppAD::cos(psi_0) * dt);
fg[1 + y_start + t] = y_1 - (y_0 + v_0 * CppAD::sin(psi_0) * dt);
fg[1 + psi_start + t] = psi_1 - (psi_0 - v_0 * delta_0 / Lf * dt);  // delta is positive we rotate counter-clockwise, or turn left
fg[1 + v_start + t] = v_1 - (v_0 + a_0 * dt);
fg[1 + cte_start + t] = cte_1 - ((f_0 - y_0) + (v_0 * CppAD::sin(epsi_0) * dt));
fg[1 + epsi_start + t] = epsi_1 - ((psi_0 - psi_des_0) + v_0 * delta_0 / Lf * dt);
```
__NOTE__: all index plus 1 because fg[0] is used to store cost.

<a name="n_var&n_constraints"></a>
### 2.4 Model Variables `vars`

The table below shows the components in `vars`:

|variable in `vars`|x|y|𝜓|v|cte|e𝜓|𝛿|a|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|number|N|N|N|N|N|N|N-1|N-1|
|index|0 ~ N-1|N ~ 2N-1|2N ~ 3N-1|3N ~ 4N-1|4N ~ 5N-1|5N ~ 6N-1|6N ~ 7N-2|7N-1 ~ 8N-2|
|start index in `vars`|0|N|2N|3N|4N|5N|6N|7N-1|
|1st item value|state[0]|state[1]|state[2]|state[3]|state[4]|state[5]||

<a name="constraints"></a>
### 2.5 Constraints

We want the __reference state[t+1] - prediction[t+1|t]__ to be as closer to 0 as  possible, the same to the cost. So we set all constraints for `fg` to be 0.

```
for (i=0; i < n_constraints; i++) {
  constraints_lowerbound[i] = 0;
  constraints_upperbound[i] = 0;
}
```

<a name="variable_limits"></a>
### 2.6 Limits for Variables

No particular limits for state variables, so I just set them to range -/+infinity

```
// set the range of state variables to -/+ infinity
for (unsigned int i = 0; i < delta_start; i++) {
  vars_lowerbound[i] = -1.79769E+308;
  vars_upperbound[i] =  1.79769E+308;
}
```

𝛿 ∈ [-25˚, 25˚]

```
// set the range of values delta to [-25, 25] in radians
  for (unsigned i=delta_start; i < a_start; i++) {
  vars_lowerbound[i] = -0.436332 * Lf;  // -25/180 * PI
  vars_upperbound[i] =  0.436332 * Lf;  //  25/180 * PI
}
```
a ∈ [-1, 1], where -1 = Full Brake and 1 = Full Acceleration.

```
// set the range of values a to [-1, 1]
for (unsigned i=a_start; i<n_vars; i++) {
  vars_lowerbound[i] = -1;
  vars_upperbound[i] = 1;
}
```

<a name="ipopt"></a>
### 2.7 IPOpt Returns Actuator Values

![alt text][image2]

Next the optimization solver is called. The solver uses the initial state `[x0, y0, 𝜓0, v0, cte0, e𝜓0]`, the model constraints and cost function to return a vector of control that minimize the cost function.

The solver we'll use is called __IPOpt__.

`CppAD::ipopt::solve_result` is a class that contains information about solve problem result. Refering [solve_result.hpp](https://www.coin-or.org/CppAD/Doc/doxydoc/html/solve__result_8hpp_source.html)

```
/// the approximation solution
Dvector x;
```
I use the very first delta and acceleration  in `solve_result.x`.  The structure of `solve_result.x` is similar to `vars`.

|variable in `solution.x`|x|y|𝜓|v|cte|e𝜓|𝛿|a|
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
|number|N|N|N|N|N|N|N-1|N-1|
|index|0 ~ N-1|N ~ 2N-1|2N ~ 3N-1|3N ~ 4N-1|4N ~ 5N-1|5N ~ 6N-1|6N ~ 7N-2|7N-1 ~ 8N-2|
|start index in `solution.x`|0|N|2N|3N|4N|5N|delta_start = 6N|a_start = 7N-1|

 So the index of the very first delta and acceleration is also `delta_start` and `a_start`.

```
return {solution.x[delta_start], solution.x[a_start]};
```

<a name="tune"></a>
### 2.8 Tuning MPC

See code in MPC.cpp line 49 ~ 73.

  