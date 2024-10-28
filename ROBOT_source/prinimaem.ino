void prinimaem() {
  Prinimaem[0] = Serial1.read();
  if (Prinimaem[0] != 65) return;    //A - start byte for some reason
  Prinimaem[1] = Serial1.read();
  Prinimaem[2] = Serial1.read();
  Prinimaem[3] = Serial1.read();
  Prinimaem[4] = Serial1.read();
  if (Prinimaem[4] != (Prinimaem[0] xor Prinimaem[1] xor Prinimaem[2] xor Prinimaem[3]))    return;     //some error

  comandaRX = Prinimaem[1];
  dataRX = (Prinimaem[2] * 256) + Prinimaem[3];


  if (comandaRX == 30) {              // =======================================RIGHT TRACK=======================================
    if (dataRX <= 2043) gpio_write_bit(GPIOF, 11, 1), gpio_write_bit(GPIOF, 12, 1);  // Set DIR and EN
    if (dataRX >= 2051) gpio_write_bit(GPIOF, 11, 0), gpio_write_bit(GPIOF, 12, 1);  // Set DIR and EN
    if (dataRX > 2043 and dataRX < 2051) gpio_write_bit(GPIOF, 12, 0);               // Stop in dead zone
    pwmR = dataRX - 2047, pwmR = abs(pwmR);
    dac_write_channel(DAC, DAC_CH1, map(pwmR, 0, 2048, 180, 1200)), comandaRX = 0;   // RIGHT DAC, Channel, value  map(value, fromLow, fromHigh, toLow, toHigh)
  }
  if (comandaRX == 31) {              // =======================================LEFT TRACK=======================================
    if (dataRX <= 2043) gpio_write_bit(GPIOF, 13, 0), gpio_write_bit(GPIOF, 14, 1);  // Set DIR and EN
    if (dataRX >= 2051) gpio_write_bit(GPIOF, 13, 1), gpio_write_bit(GPIOF, 14, 1);  // Set DIR and EN
    if (dataRX > 2043 and dataRX < 2051) gpio_write_bit(GPIOF, 14, 0);               // Stop in dead zone
    pwmL = dataRX - 2047, pwmL = abs(pwmL);
    dac_write_channel(DAC, DAC_CH2, map(pwmL, 0, 2048, 180, 1200)), comandaRX = 0;   // LEFT DAC, Channel, value
  }


  if (comandaRX == 39 and dataRX == 0 and hatch == 0) OPENUP();                                           //LIDAR+power UP 456, servo-red color
  if (comandaRX == 39 and dataRX == 1 and hatch == 1) CLOSEDOWN();                                        //LIDAR+power DOWN, servo-green color
  if (comandaRX == 41) servo.moveJoint(1, dataRX);
  if (comandaRX == 42) servo.moveJoint(2, dataRX);
  if (comandaRX == 43) servo.moveJoint(3, dataRX);
  if (comandaRX == 44) servo.moveJoint(4, dataRX);
  if (comandaRX == 116 and dataRX == 1) RPMena = 1;                                                                             //Start sending RPMs out.
  if (comandaRX == 116 and dataRX == 0) RPMena = 0;                                                                             //Stop sending both RPMs.
  if (comandaRX == 45 and (dataRX < 6))  comandaRX = 0, servoPOS();                                                             //GetServoPos  5 serv-to - #5 na lidare
  if (comandaRX == 119 and dataRX == 10) myIMU.enableGameRotationVector(50), IMUen = 1;                                         //def:50 - start update every 50ms from IMU, in real about 200ms. dataRX=10 - get yaw, other two below
  if (comandaRX == 119 and dataRX == 11) myIMU.enableGameRotationVector(50), IMUen = 1;
  if (comandaRX == 119 and dataRX == 12) myIMU.enableGameRotationVector(50), IMUen = 1;
  if (comandaRX == 119 and dataRX == 17) myIMU.enableGameRotationVector(50), IMUen = 1;                                         //3D!!!
  if (comandaRX == 119 and dataRX == 0) myIMU.enableGameRotationVector(0), IMUen = 0;                                           //Stop IMU data  '''REMOVE, NOT NEEDED?'''
  if ((comandaRX == 40) and (LIDAR == 1)) moveLIDAR();                                                                          //Move LIDAR
  if ((comandaRX == 50) and dataRX == 50 and (LIDAR == 1)) getDist();                                                           //Get DISTANCE
  if ((comandaRX == 51) and dataRX < 1024 and (LIDAR == 1)) getDistNew();                                                       //Get DISTANCE new implementation
  if ((comandaRX == 20) and (dataRX == 20)) voltage = analogRead(PC0), comandaTX = 20, dataTX = voltage, posilka();             //Get Battery's Voltage
  if ((comandaRX == 19) and (dataRX == 19)) cCurrent = analogRead(PC2), comandaTX = 19, dataTX = cCurrent, posilka();           //Get mains current
  if ((comandaRX == 21) and (dataRX == 21)) hatchStatus();                                                                      //Get hatch status
  if ((comandaRX == 32) and dataRX > 0 and dataRX < 181) turnRight();                                                           //Perform turn based on IMU data. data allowed: 1-180
  if ((comandaRX == 33) and dataRX > 0 and dataRX < 181) turnLeft();                                                            //Perform turn based on IMU data. data allowed: 1-180
  if ((comandaRX == 14) and (dataRX < 4095)) pwmController.setChannelPWM(14, dataRX);                                           //Left headlight
  if ((comandaRX == 15) and (dataRX < 4095)) pwmController.setChannelPWM(15, dataRX);                                           //Right headlight
  if (comandaRX == 55 and dataRX == 55) comandaTX = 55, dataTX = 55, posilka();                                                 //STM32 Ping
  if (comandaRX == 61 and dataRX < 10000) tachoMoveForward();                                                                   //Tacho Move Forward. MILLIMETERS!
  if (comandaRX == 68 and dataRX < 10000) tachoMoveBackward();                                                                  //tacho Move Backward. MILLIMETERS!
  if (comandaRX == 71 and dataRX < 1000 and dataRX > 1 and (LIDAR == 1)) LIDARmoveF();                                          //LIDAR move. CANTIMETERS! 1 meter max!!!

  //if ((comandaRX == 18) and (dataRX == 18)) hatchCurrent = analogRead(PC1), comandaTX = 18, dataTX = hatchCurrent, posilka(); //Return hatch's motor current. nah nado?
  comandaRX = 0;

}
