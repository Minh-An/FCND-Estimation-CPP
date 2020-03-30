# Implemented body rate control in C++.
#### The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments.

Implemented a simple P controller that took desired roll, pitch, yaw rates in `V3F pqrCmd` and actual roll pitch yaw rates in `V3F pqr` to get the rotational accelerations $\dot p , \dot q , \dot r$

Then multiplied each acceleration with the moment of intertia for each respective axis, to get commanded moments `momentCmd` ($M_x, M_y, M_z$)

```
V3F moment_intertia(Ixx, Iyy, Izz);
V3F momentCmd = moment_intertia * kpPQR * (pqrCmd - pqr);
return momentCmd;
```

Tuned `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
```
kpPQR = 60,70,15
```


# Implement roll pitch control in C++.
#### The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles.

In the function `RollPitchControl`:
1. Calculated total acceleration c by dividing `-collThrustCmd` ($F_{total}$) by mass 
2. $$\begin{pmatrix} b^x_c \\ b^y_c \end{pmatrix} = \frac{1}{c}\begin{pmatrix} \ddot{x} \\ \ddot{y} \end{pmatrix} $$
3. $$\begin{pmatrix} b^x_a \\ b^y_a \end{pmatrix} = \begin{pmatrix} R_{13} \\ R_{23} \end{pmatrix} $$
4. Use a P controller with the commanded and actual $b^x$ and $b^y$ to calculate $\dot{b^x_c}$ and $\dot{b^y_c}$
5. Using $\dot{b^x_c}$ and $\dot{b^y_c}$, calculate $p_c$ and $q_c$ with equation 
$$
\begin{pmatrix} p_c \\ q_c \\ \end{pmatrix}  = \frac{1}{R_{33}}\begin{pmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{pmatrix} \times \begin{pmatrix} \dot{b}^x_c \\ \dot{b}^y_c  \end{pmatrix} 
$$

```
V3F pqrCmd;

Mat3x3F R = attitude.RotationMatrix_IwrtB();

float c = -collThrustCmd / mass;
float b_x_c = CONSTRAIN(accelCmd[0] / c, -maxTiltAngle, maxTiltAngle);
float b_y_c = CONSTRAIN(accelCmd[1] / c, -maxTiltAngle, maxTiltAngle);
float b_x_a = R(0,2);
float b_y_a = R(1,2);
float b_dot_x_c = kpBank * (b_x_c - b_x_a);
float b_dot_y_c = kpBank * (b_y_c - b_y_a);

pqrCmd[0] = (R(1,0) * b_dot_x_c - R(0, 0) * b_dot_y_c)/R(2,2);
pqrCmd[1] = (R(1,1) * b_dot_x_c - R(0, 1) * b_dot_y_c)/R(2,2);

return pqrCmd;
```
Tuned `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot

```
kpBank = 14
```

# Implement altitude controller in C++.
#### The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Implemented a cascaded P controller (with integrated error) to calculated desired altitude acceleration $\ddot{z}_c$ given $z_t, \dot{z_t}, \ddot{z_t}, z_a, \dot{z_a}, dt$ 

Once $\ddot{z_c}$ is calculated, calculate total `thurst` $F_{total}$ given equation 

$$F_{total} = \frac {(g-\ddot{z_c}) * m} {R_{33}}$$

where $m$ is the drone mass, $g$ is the acceleration of gravity, and , $R$ is the rotation matrix derived from the attitude of the drone

```
float g = 9.81;
float e = (posZCmd - posZ);
float z_dot_t = CONSTRAIN(kpPosZ * e + velZCmd, -maxAscentRate, maxDescentRate);
integratedAltitudeError += e * dt;
float z_dot_dot = kpVelZ * (z_dot_t - velZ) + KiPosZ*integratedAltitudeError + accelZCmd;
thrust = -(z_dot_dot -g) * mass / R(2,2);
```

Tuned parameters `kpPosZ` and `kpVelZ` in file `QuadControlParams.txt` 

```
kpPosZ = 3
kpVelZ = 10
```

#### Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4.

Implemented a I (integrated) part of the controller that would update the integrated altitude error for each call and add it to the controller. 

```
integratedAltitudeError += e * dt;
float z_dot_dot = kpVelZ * (z_dot_t - velZ) + KiPosZ*integratedAltitudeError + accelZCmd;
```

Tuned `KiPosZ` in file `QuadControlParams.txt` to be = 25.

# Implement lateral position control in C++.

#### The controller should use the local NE position and velocity to generate a commanded local acceleration.

In function `LateralPositionControl`, parameters: 
- `V3F posCmd` is a vector of 3 containing $x_t, y_t, z_a$ 
- `V3F velCmd`is a vector of 3 containing $\dot{x}_t, \dot{y}_t, 0$
- `V3F pos` is a vector of 3 containing $x_a, y_a, z_a$ 
- `V3F vel` is a vector of 3 containing  $\dot{x}_a, \dot{y}_a, 0$
- `V3F accelCmdFF` is a vector of 3 containing $\ddot{x}_t, \ddot{y}_t, 0$ 

** Note that the z of posCmd is equal to z of pos

Implemented a cascaded P controller to calculated desired lateral x acceleration $\ddot{x}_c$ given $x_t, \dot{x_t}, \ddot{x_t}, x_a, \dot{x_a}$ 

Implemented a cascaded P controller to calculated desired lateral y acceleration $\ddot{y}_c$ given $y_t, \dot{y_t}, \ddot{y_t}, y_a, \dot{y_a}$ 

```
float x_dot_t = CONSTRAIN(kpPosXY * (posCmd[0] - pos[0]) + velCmd[0], -maxSpeedXY, maxSpeedXY);
float y_dot_t = CONSTRAIN(kpPosXY * (posCmd[1] - pos[1]) + velCmd[1], -maxSpeedXY, maxSpeedXY);

//printf("%f, %f, %f, %f, %f, %f\n", posCmd[0], pos[0], velCmd[0], posCmd[1], pos[1], velCmd[1]);

V3F vel_t = V3F(x_dot_t, y_dot_t, 0);

accelCmd += kpVelXY * (vel_t - vel);
accelCmd[0] = CONSTRAIN(accelCmd[0], -maxAccelXY, maxAccelXY);
accelCmd[1] = CONSTRAIN(accelCmd[1], -maxAccelXY, maxAccelXY);
```

Tuned parameters `kpPosXY` and `kpVelXY` in file `QuadControlParams.txt` 

```
kpPosXY = 3
kpVelXY = 12
```

# Implement yaw control in C++.

#### The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required).

In the function `YawControl`, implemented a simple P controller to control the yaw of the drone, with inputting the desired and acutal yaw angles $\psi_c$ and $\psi$, and outputting $\dot\psi$, the commanded yaw rate. 

`yawRateCmd = kpYaw * (yawCmd - yaw);`

Tuned parameters `kpYaw` and the 3rd (z) component of `kpPQR` in file `QuadControlParams.txt`
```
kpYaw = 3
kpPQR = 60,70,20
```


# Implement calculating the motor commands given commanded thrust and moments in C++.

#### The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

Using the system of equations below, solved for the respective $F_1 , F_2 , F_3 , F_4$

$$
\begin{pmatrix} 1 & 1 & 1 & 1 \\ 1 & -1 & -1 & 1 \\ 1 & 1 & -1 & -1\\ 1 & -1 & 1 & -1 \end{pmatrix} \times \begin{pmatrix} F_1 \\ F_2 \\ F_3\\ F_4 \end{pmatrix} = \begin{pmatrix} F_{total} \\ M_x / l \\ M_y / l \\ - M_z / \Kappa \end{pmatrix}
$$

In the function `GenerateMotorCommands`, $F_{total}$ is a parameter called `collThrustCmd`, while $M_x, M_y, M_z$ (moments about each axis) is represented as a V3F parameter that contains all moments called `momentCmd`

```
float l = L / (sqrt(2));
float a = momentCmd.x / l;
float b = momentCmd.y / l;
float c = -momentCmd.z / kappa;

cmd.desiredThrustsN[0] = CONSTRAIN((collThrustCmd + a + b + c)/4.0, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[1] = CONSTRAIN((collThrustCmd - a + b - c)/4.0, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[2] = CONSTRAIN((collThrustCmd + a - b - c)/4.0, minMotorThrust, maxMotorThrust);
cmd.desiredThrustsN[3] = CONSTRAIN((collThrustCmd - a - b + c)/4.0, minMotorThrust, maxMotorThrust);

return cmd;
```
