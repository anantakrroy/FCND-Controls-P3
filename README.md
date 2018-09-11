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
<img src="cascaded_controller.png" width="500"/>
</p>
