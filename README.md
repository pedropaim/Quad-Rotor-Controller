# Quad-Rotor-Controller
Udacity Autonomous Flight Nanodegree Project 3 - Control of a Quad-Rotor Drone

## Generating Motor Commands

The thrust and moments should be converted to the appropriate 4 different desired thrust forces for the moments. Ensure that the dimensions of the drone are properly accounted for when calculating thrust from moments.

```
float l = L/1.4142;
float cBar = collThrustCmd;
float pBar = momentCmd.x/l;
float qBar = momentCmd.y/l;
float rBar = momentCmd.z/kappa;
```

```
cmd.desiredThrustsN[0] = (cBar + pBar + qBar - rBar) / 4.f;
cmd.desiredThrustsN[1] = (cBar - pBar + qBar + rBar) / 4.f;
cmd.desiredThrustsN[2] = (cBar + pBar - qBar + rBar) / 4.f;
cmd.desiredThrustsN[3] = (cBar - pBar - qBar - rBar) / 4.f;
```


## Body-Rate Control

"The controller should be a proportional controller on body rates to commanded moments. The controller should take into account the moments of inertia of the drone when calculating the commanded moments."

```
momentCmd[0] = (pqrCmd[0] - pqr[0])*Ixx*kpPQR[0];
momentCmd[1] = (pqrCmd[1] - pqr[1])*Iyy*kpPQR[1];
momentCmd[2] = (pqrCmd[2] - pqr[2])*Izz*kpPQR[2];
```


## Roll-Pitch Control

"The controller should use the acceleration and thrust commands, in addition to the vehicle attitude to output a body rate command. The controller should account for the non-linear transformation from local accelerations to body rates. Note that the drone's mass should be accounted for when calculating the target angles."

```
    float c_d = collThrustCmd/mass;
``` 

```
    if (collThrustCmd > 0.0)
    {
        target_R13 = -accelCmd[0]/c_d;
        target_R23 = -accelCmd[1]/c_d;
        
        target_R13 = CONSTRAIN(target_R13,-maxTiltAngle, maxTiltAngle);
        target_R23 = CONSTRAIN(target_R23,-maxTiltAngle, maxTiltAngle);
        
        p_cmd = (1/R(2,2))* (-R(1,0) * kpBank * (R(0,2) - target_R13) + R(0,0)*kpBank*(R(1,2) - target_R23));        
        q_cmd = (1/R(2,2))* (-R(1,1)*kpBank*(R(0,2) - target_R13)+R(0,1)*kpBank*(R(1,2)-target_R23));
    }
    else
    {
        p_cmd = 0.0;
        q_cmd = 0.0;
    }
    
    pqrCmd[0] = p_cmd;
    pqrCmd[1] = q_cmd;
    pqrCmd[2] = 0;
```

## Lateral Position Control

The controller should use the local NE position and velocity to generate a commanded local acceleration."

```
velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
    
accelCmd.x += kpPosXY*(posCmd.x - pos.x) + kpVelXY*(velCmd.x - vel.x);
accelCmd.y += kpPosXY*(posCmd.y - pos.y) + kpVelXY*(velCmd.y - vel.y);
accelCmd.z = 0;
    
accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);
```

## Altitude Control

The controller should use both the down position and the down velocity to command thrust. Ensure that the output value is indeed thrust (the drone's mass needs to be accounted for) and that the thrust includes the non-linear effects from non-zero roll/pitch angles.

Additionally, the C++ altitude controller should contain an integrator to handle the weight non-idealities presented in scenario 4."

```
velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);
    
integratedAltitudeError += (posZCmd - posZ)*dt;
    
float u_bar = kpPosZ*(posZCmd - posZ) + kpVelZ*(velZCmd - velZ) + accelZCmd + integratedAltitudeError * KiPosZ;
float c = (u_bar - 9.81f)/R(2,2);
thrust = -c*0.5;
```

## Yaw Control

The controller can be a linear/proportional heading controller to yaw rate commands (non-linear transformation not required)."

## Flight Evaluation


## Scenario 5
