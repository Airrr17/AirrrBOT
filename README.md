## ROBOT4 by (c)Airrr(r) 28.06.2020  
  
This is a HOST(robot) part of AirrrBOT code as seen on  
https://www.youtube.com/user/AirrrOk  
  
  
Server part can be implemeted in any language.  
Currently it is done in VB.NET  
See somehow poorly discribed API below.  
  
  
  
This code is under development.  
  
  
  
--It can precisely moves to any directions with simple joystick control :)  
--Measure it’s own voltage and current consumption.  
--Knows it’s own yaw, pitch and roll. Only yaw are used for now as compass.  
--Fully auto retractable 300-degree LIDAR including hatch opening. Over-current and sensors protected to prevent self-wounding.  
--16 channels PWM for servos and headlights (right and left independable).  
--Can return it’s both current RPM’s form right and left tracks every 50ms. Good for knowing distance moved without LIDAR.  
--Rotate automatically to any desired angle 1-180 cw or ccw. Reporting back difference between rotated and desired angle.  
  Once failed to rotate an error message being reported, also difference between angles reported.  
--Can draw a map of it's current location in 0.29 degree resolution. Much slow for now :(  
--Servo positions are readable.  
  
OrangePI zero used for converting from tcp-ip to 0.5Mb serial routed in stm32f103zet6 as main controller.  
BNO080 as compass*  
XL320 servos directly form STM32.  
PCA9685 16ch PWM controller.  
TFmini as LIDAR (LED actually).  
  
To DO:  
-- Arm with gripper and camera.  
-- Fully auto moveing forward at any desired range in two modes: LIDAR or RPM's counting.  
  
![alt text](https://github.com/Airrr17/AirrrBOT/blob/master/picture01.jpg)  
  
![alt text](https://github.com/Airrr17/AirrrBOT/blob/master/block.png)  
  
  
  
---------------------------------------------  
The packet (bi-dir) is a 5 bytes array (UDP ip:port)  
#1-    start byte - always ascii "A", dec 65, byte  
#2-    command (byte), byte  
#3+#4- data (two bytes combined in short), short  
#5-    checksum, all xor, byte  
  
*From here and further only two variables are used: #2(command[0-255]) and #3+#4 (data[0-65535])  
(As startbyte is a constant. Checksum are autocalculated.)  
[65][0-255][0-65535][xor]  
---------------------------------------------  
For list of commands see .doc file.  
