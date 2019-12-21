## ROBOT4 by (c)Airrr(r) 22.12.2019  
  
This is a HOST(robot) part of AirrrBOT as seen on  
  
https://www.youtube.com/user/AirrrOk  
  
This code is under development.  
  
  
  
--It can precisely moves to any directions with simple joystick control :)  
--Measure it’s own voltage and current consumption.  
--Knows it’s own yaw, pitch and roll. Only yaw are used for now as compass.  
--Auto retractable LIDAR including hatch opening. Over-current and sensors protected to prevent self-wounding.  
--16 channels PWM for servos and headlights (right and left independable).  
--Can return it’s both current RPM’s form right and left tracks every 50ms. Good for knowing distance moved without LIDAR.  
--Rotate automatically to any desired angle 1-180 cw or ccw. Reporting back difference between rotated and desired angle. Once failed to rotate an error message being reported, also difference between angles reported.  
  
OrangePI zero used for converting from tcp-ip to 1Mb serial routed in stm32f103zet6 as main controller.  
BNO080 as compass*  
XL320 servos directly form STM32.  
PCA9685 16ch PWM controller.  
TFmini as LIDAR (LED actually).  
  
  
  
The PACKET (bi-directional) is a 5 bytes array sent byte by byte (UDP on ip:port)  
---------------------------------------------  
#1-    start byte - always ascii "A", dec 65, byte  
#2-    command (byte), byte  
#3+#4- data (two bytes combined in short), short  
#5-    checksum, all xor, byte  
*From here and further only two variables are used: #2(command[0-255]) and #3+#4 (data[0-65535])  
(As startbyte is a constant. Checksum are autocalculated.)  
[65][0-255][0-65535][xor]  
---------------------------------------------  
ABSTRACT:  
   
API consists of two concurrent modes: scout and manual.  
Modes definition:  
(S)-Scout:  Manipulator (with camera) are constantly above LIDAR and pointed 0(360) degrees forward (unmovable). LIDAR are ready (hatch are opened of course).  
(M)-Manual: LIDAR are hidden (down) while hatch can be open or closed. Manipulator are enabled. Totally depends on operator. No automatic function are engaged.  
(B)-Both:   Other commands that NOT combine manipulator movement and LIDAR being up and enabled to prevent manipulator-LIDAR collision.  
  
******this manual describes (B) or (S) modes as long for now******  
  
#1. ******skip down all, no (M) mode for now****** Switch between modes (S) or (M) depends on LIDAR being UP or DOWN.  
#2. Hatch open\close and LIDAR up\down can be controlled independently with restriction:  
#3. While hatch are closed LIDAR can't be UP.  
#4. !LIDAR_UP   engages both commands: hatch_open and lidar_up. (3.5 seconds to complete).  
#5. !LIDAR_DOWN engages both commands: lidar_down and hatch_close. (3.5 seconds to complete).  
  
  
And two types of commands: one-directional(1)- only order to implement OR bi-directional (2), command that involves an answer.  
  
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ START HERE @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  
COMMANDS:  
  
(B)(2): Get battery voltage.  
  REQ: command 20, data 20  
  ANS: command 20, data VOLTAGE. voltage = data*0.00344 in V.  
  
(B)(2): Get battery current.  
  REQ: command 19, data 19  
  ANS: command 19, data CURRENT. current = ((((data/4095)*3289)-2530)/170*1000) in mA.  
  
(B)(1): Driving (set speed of every track, direction are calculated automatically with dead zone in a middle).  
  LEFT  REQ: command 31, data 0-4095 where: 0(full back)->2043(super slow back) ; 2043->2051 dead zone (stop) ; 2051(super slow forward)<-4095(full forward)  
  RIGHT REQ: command 30, same as left: (0)````---___(2043) ; <(2043)XdeadXzoneX(2051)> ; (2051)___---````(4095)  
  
(B)(2): Get absolute direction on YAW ("horizontal").  
  REQ: command 119, data 10  
  ANS: command 119, data ANGLE in degrees (0-360)  
 *ABSOLUTE direction is always 0 degree on startup. (not a compass, magnetic sensor (compass, degradable) can be enables by software in future.  
  
(B)(2): Enable\Disable RPM report on both tracks*.  
  REQ: command 116, data 1- on,  RPM report enabled.  
						  0- off, RPM report disabled.  
  ANS: command 117, data left RPM +  
       command 118, data right RPM  
 *WARNING! While rpm_enable are triggered ON robot sends it's RPM's every 50ms until RPM report are disabled (command 116, data 0).  
 *RPM are _current_ rotations per second of every track, its immediately sets to 0 when corresponding track are stopped.  
 *Requires a lot of bandwidth.  
  
(B)(2): Get hatch status.  
  REQ: command 21, data 21  
  ANS: command 21, data:  0- hatch closed  
						  1- hatch open  
						  2- hatch absent (no top part of the robot are connected)  
						  3- opened and closed at the same time, never can happen.  
						  4- Over-current. Something stopping the hatch!  
  
(B)(1): LIDAR up\down ena\disa. (Switches (S) and (M) modes)  
  REQ: command 39, data: 0- up.   Opens hatch (3sec) and rising LIDAR (0.5sec), manipulator moves to static position.  
						  1- down. Closes hatch (3sec) and hiding LIDAR (0.5sec), manipulator enabled. Operator only.  
 ******ALWAYS (S) for now******  
  
   
(S)(1): Rotate LIDAR to position.  
  REQ: command 40, data: 0-1023. Points LIDAR to desired position while 300 degrees divided by 1023 steps. Each step 0.29 degree. 1.5sec on  
  
(S)(2): Get range at current LIDAR's position in cm.  
  REQ: command 50, data 50  
  ANS: command 50, data RANGE, 30cm min, 1200cm max (0.3-1.2m) if (> 1200) = 1200.  
*User MUST send packets delayed proportionally to (current position -to- desired position) before getting distance from LIDAR.  
 E.g. The difference between current LIDAR position and desired position calculated as 0to300 degree = 1.5 seconds.  
 10 degree rotation will take 50ms + network lag.  
*In undefined conditions advised to use more get_range requests per every LIDAR positioning.  
  
(B)(2): Auto-turn CW.  
  REQ: command 32, data ANGLES TO TURN RIGHT (1-180 degree).  
  ANS: command 32, data ANGLE. Returns difference between desired and actual angle after completing. Beep and stop on error. REAL=1000 <- turn error.  
  
(B)(2): Auto-turn CCW.  
  REQ: command 33, data ANGLES TO TURN LEFT (1-180 degree).  
  ANS: command 33, data ANGLE. Returns difference between desired and actual angle after completing. Beep and stop on error. REAL=1000 <- turn error.  
