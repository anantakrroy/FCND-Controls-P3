# FCND-Controls-P3
CPP Controller for a simple quadcopter

This is the README for the Controls project of Udacity Flying Car ND [Link](https://github.com/udacity/FCND-Controls-CPP).

We need to build a CPP controller and tune the parameters based on the different requirements in each scenarios - 1 to 5.

**Files in submission**
* QuadControl.cpp - The main CPP controller code.
* QuadControlParams.txt - Configuration for the controller containing the tuned parameters.

**Requirements of the project**
1. Development environment setup - Xcode for Mac OS X.
2. Compile and test the simulator on the IDE.
3. Adjust the `mass` to make the quad hover at the same altitude.
4. Implement the body Rate control using the `GenerateMotorCommands` and `BodyRateControl` methods.Tune `kpPQR` to get the vehicle to stop spinning.
5. Implement roll / pitch control using the `RollPitchControl` method.Tune `kpBank` to minimize settling time.
6. Implement code in `LateralPositionControl` and `AltitudeControl` . Tune parameters `kpPosXY` and `kpPosZ` , `kpVelXY` and `kpVelZ`. 
7. Implement code in `YawControl` . Tune `kpYaw`since one quad is rotated in yaw.
8. Explore some of the non-idealities and robustness of a controller by tweaking all parameters and adding in a integral control and tuning it.

**Cascaded Controller**

We will be implementing this basic cascaded controller for this project as shown in the image.
<p align="center">
<img src="cascade_controller.png" width="650"/>
</p>

### Scenarios ###

###### Scenario 1 - Intro ######

This is the Intro scene. The `mass` is so adjusted that the quad is able to hover at a constant altitude as long as possible.
<p align="center">
<img src="animation/1.gif" width="500"/>
<img src="animation/1.png" width="500"/>
</p>

###### Scenario 2 - Body rate and roll/pitch control ######

Quad is created with a small initial rotation speed about its roll axis. The controller needs to stabilize the rotational motion and bring the vehicle back to level attitude.
<p align="center">
<img src="animation/2.gif" width="500"/>
<img src="animation/2.png" width="500"/>
</p>

###### Scenario 3 - Position/velocity and yaw angle control ######

There are 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees. On successful implementation and tuning, the quads should be going to their destination points.
<p align="center">
<img src="animation/3.gif" width="500"/>
<img src="animation/3.png" width="500"/>
</p>

###### Scenario 4 - Non-idealities and robustness ######

This scenario explores some of the non-idealities and robustness of a controller. There are three quads.
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual
 
<p align="center">
<img src="animation/4.gif" width="500"/>
<img src="animation/4.png" width="500"/>
</p>

###### Scenario 5 - Tracking trajectories ######

Aim of this scenario is to test the quad's performance once again on a trajectory. This scenario has two quadcopters:
 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt`
<p align="center">
<img src="animation/5.gif" width="500"/>
<img src="animation/5.png" width="500"/>
</p>

### Implemented Controller ###

#### 1. Implemented Body rate control

```
  V3F momentI;

  momentI.x = Ixx;
  momentI.y = Iyy;
  momentI.z = Izz;
    
  V3F err_rate = (pqrCmd - pqr);

  momentCmd = momentI * kpPQR * err_rate
 ```
#### 2. Implemented roll pitch control

```
float targetBX = 0.0;
  float targetBY = 0.0;
  if (collThrustCmd > 0.0)
  {
      float c = collThrustCmd/mass;
      targetBX = -CONSTRAIN(accelCmd.x/c, -maxTiltAngle, maxTiltAngle);
      targetBY = -CONSTRAIN(accelCmd.y/c, -maxTiltAngle, maxTiltAngle);
  }
  float bX = targetBX - R(0, 2);
  float bY = targetBY - R(1, 2);

  pqrCmd.x = kpBank *((R(1, 0) * bX) - (R(0, 0) * bY)) / R(2, 2);
  pqrCmd.y = kpBank *((R(1, 1) * bX) - (R(0, 1) * bY)) / R(2, 2);
  pqrCmd.z = 0.f;
  
  ```
  
  #### 3. Implemented Altitude control
  
  ```
  float b_z = R(2,2);

  float err_z = posZCmd - posZ;

  integratedAltitudeError += err_z * dt;

  //PID control altitude controller

  float p = kpPosZ * err_z;
  float i = KiPosZ * integratedAltitudeError;

 //float acc_z_cmd = p + i + d + accelZCmd;
    
  float velZRef = velZCmd + p + i;
  velZRef = -CONSTRAIN(-velZRef, -maxDescentRate, maxAscentRate);
    
  float err_z_vel = velZRef - velZ;
  float d = kpVelZ * err_z_vel;
    
  float accelCmd = accelZCmd + d;
  thrust = mass * (9.81f - (accelCmd / b_z));
  
  ```
  
  #### 4. Implemented lateral position control
  
  ```
  velCmd.constrain(-maxSpeedXY,maxSpeedXY);

  V3F posErr = posCmd - pos;
  V3F velErr = velCmd - vel;

  accelCmd = accelCmdFF + (kpPosXY * posErr) + (kpVelXY * velErr); //z compent is zero, so let's ignore use kpPosXY/kpVelXY for pos.z/vel.z as well
  accelCmd.constrain(-maxAccelXY,maxAccelXY);

  accelCmd.z = 0;
  
 ```
 
 #### 5. Implemented yaw control
 
 ```
 yawCmd = fmod(yawCmd, (2.0f*F_PI));

  if (yawCmd <= -F_PI)
  {
     yawCmd += (2.0f*F_PI);
  }
  else if (yawCmd > F_PI)
  {
     yawCmd -= (2.0f*F_PI);
  }

  yawRateCmd = kpYaw * (yawCmd - yaw);
  
  ```
  #### 6. Implemented Motor commands
  
  ```
  // Total thrust is given by F_total
  float len = L / sqrtf(2.f);
  float F_total = collThrustCmd ;

  // Moment = Force * length
  float F_rotX = momentCmd.x / len; //Moment about the X-axis, roll i.e F1+F4-F2-F3
  float F_rotY = momentCmd.y / len; //Moment about the Y-axis, pitch i.e F1+F2-F3-F4
  float F_rotZ = -momentCmd.z / kappa; //Moment about the Z-axis, yaw i.e F

  // front left  - f1, front right - f2, rear left   - f4, rear right  - f3

  cmd.desiredThrustsN[0] = (F_total + F_rotX + F_rotY + F_rotZ)/4.f;  
  cmd.desiredThrustsN[1] = (F_total - F_rotX + F_rotY - F_rotZ )/4.f;  
  cmd.desiredThrustsN[2] = (F_total + F_rotX - F_rotY - F_rotZ )/4.f ;  
  cmd.desiredThrustsN[3] = (F_total - F_rotX - F_rotY + F_rotZ )/4.f;  
  
  ```
  
  ### Flight Evaluation
  
  1. Scenario-1 - 
  <p align="center">
  <img src="1.png" width="650"/>
  </p>
