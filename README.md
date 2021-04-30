# BallTracking
You need to install https://github.com/mu-opensource/MuVisionSensor3 as a zip library in Arduino IDE.
Set the four dial switches on the camera as **v ^ v v** (the second switch dialed up to I2C) and connect the camera module to the I2C grove on NyBoard.
The battery should be turned on to drive the servos. 

You can use this 3D printed [bone](https://github.com/PetoiCamp/NonCodeFiles/blob/master/stl/bone.stl) to attach the camera module. After uploading the code, you may need to press the reset buttons on the module and the NyBoard. 

The tracking demo works the best with a yellow tennis ball or some other round objects. Below is a short demo.
 
 [![BallTracking](https://img.youtube.com/vi/CxGI-MzCGWM/0.jpg)](https://www.youtube.com/watch?v=CxGI-MzCGWM)
