# Quad-Rotor-Controller
Udacity Autonomous Flight Nanodegree Project 3 - Control of a Quad-Rotor Drone

## Generating Motor Commands

The GenerateMotorCommands method computes the thrust forces to be applied on each motor in order to generate the desired moments along the x,y and z axes and the desired collective thrust. 

Initially, the four auxiliary variables are defined as cBar, pBar, qBar and rBar:
```
float cBar = collThrustCmd;
float pBar = momentCmd.x/l;
float qBar = momentCmd.y/l;
float rBar = momentCmd.z/kappa;
```

Variable "l" is the moment arm along the x and y axes and is computed as l = L/1.4142, where L is the distance between each motor and the center of the drone. The desired thrust on each motor is then computed as follows:

```
cmd.desiredThrustsN[0] = (cBar + pBar + qBar - rBar) / 4.f;
cmd.desiredThrustsN[1] = (cBar - pBar + qBar + rBar) / 4.f;
cmd.desiredThrustsN[2] = (cBar + pBar - qBar + rBar) / 4.f;
cmd.desiredThrustsN[3] = (cBar - pBar - qBar - rBar) / 4.f;
```

## Body-Rate Control

The Body-Rate Controller (method BodyRateControl) is a proportional controller. The control signal is computed as the product of the body rate error (commanded body rate minus current body rate) multiplied by the moment of inertia and a proportional control gain kp:

```
momentCmd[0] = (pqrCmd[0] - pqr[0])*Ixx*kpPQR[0];
momentCmd[1] = (pqrCmd[1] - pqr[1])*Iyy*kpPQR[1];
momentCmd[2] = (pqrCmd[2] - pqr[2])*Izz*kpPQR[2];
```

## Roll-Pitch Control

The Roll-Pitch Controller (method RollPitchControl) computes a body-rate command (p_cmd and q_cmd) from a desired acceleration in x and y. Initially, it is necessary to convert the collective thrust command (collThrustCmd), which is in Newtons, into an acceleration value c_d in m/s^2: 

```
    float c_d = collThrustCmd/mass;
``` 

In case the collective thrust command is positive, then the p_cmd and q_cmd are computed as follows. 

The equations represent the computation of a desired lateral acceleration in X and Y (target_R13 and target_R23):

```
target_R13 = -accelCmd[0]/c_d;
target_R23 = -accelCmd[1]/c_d;

```

which are then constrained to within limits (maxTiltAngle) :

```
target_R13 = CONSTRAIN(target_R13,-maxTiltAngle, maxTiltAngle);
target_R23 = CONSTRAIN(target_R23,-maxTiltAngle, maxTiltAngle);
```

The control signal is then computed as the product of a control gain kp and the angular rate error in the global coordinates :

```
float b_c_x_dot = kpBank*(R(0,2)-target_R13);
float b_c_y_dot = kpBank*(R(1,2)-target_R23);
```

Which are then converted to local coordinates to find p_cmd and q_cmd in the local frame :

```
p_cmd = (1/R(2,2))* (-R(1,0)*b_c_x_dot + R(0,0)*b_c_y_dot);
q_cmd = (1/R(2,2))* (-R(1,1)*b_c_x_dot + R(0,1)*b_c_y_dot);
```
If the collective thrust command is negative, p_cmd and q_cmd are set to 0.

## Lateral Position Control

The controller should use the local NE position and velocity to generate a commanded local acceleration."

In the Lateral Position Controller (method LateralPositionControl), the velocity command in x and y is initially constrained (within +/- maxSpeedXY).

```
velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
```

The controller then computes a desired acceleration in x and y  as a the sum of a position error multiplied by gain kpPosXY and a velocity error multiplied by gain kpVelXY.

```
accelCmd.x += kpPosXY*(posCmd.x - pos.x) + kpVelXY*(velCmd.x - vel.x);
accelCmd.y += kpPosXY*(posCmd.y - pos.y) + kpVelXY*(velCmd.y - vel.y);
accelCmd.z = 0;
    
accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```

The output is constrained to within maxAccelXY.

## Altitude Control

The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4."

The vertical velocity command is initially constrained to within maxDescentRate and maxAcentRate.

```
velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);
```

A vertical acceleration control signal u_bar is then computed as the sum of the vertical position error times control gain kpPosZ, the vertical velocity error times control gain kpVelZ and a commanded acceleration feedforward term accelZCmd. Additionally, an integral term is added with control gain KiPosZ.

```
integratedAltitudeError += (posZCmd - posZ)*dt;
    
float u_bar = kpPosZ*(posZCmd - posZ) + kpVelZ*(velZCmd - velZ) + accelZCmd + integratedAltitudeError * KiPosZ;
``` 

The effect of gravity is introduced and the variable is then divided by term R33 of the rotation matrix to account for the fact that the drone may be at a non-neutral attitude. Variable c (in m/sˆ2) is then converted to a thrust value (in Newtons) by dividing it by the drone mass.

```
float c = (u_bar - 9.81f)/R(2,2);
thrust = -c*0.5;
```

## Yaw Control

The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required)."

```
float b = (-3.1413,3.1413);

float error_yaw = fmodf(yaw,b) - fmodf(yawCmd,b);
yawRateCmd = -kpYaw*error_yaw;
```

## Flight Evaluation



