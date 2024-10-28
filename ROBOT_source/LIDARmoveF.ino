//Auto move forward routine based on LIDAR readings.
//Distance to move in CANTIMETERS

void LIDARmoveF() {
  int toMove = dataRX;
  int distance = 0;
  int prevDist = 0;
  int remainMove = 0;
  int strength = 0;
  int neEdet = 0;
  int startD = 0;

  if (LIDAR == 0) return;                                                        //abort if somehow lidar DOWN
  servo.moveJoint(5, lidar_center);                                              //turn lidar ON, move to center
  delay(400);  

  while (!distance) {
    getTFminiData(&distance, &strength);
    if (distance) {
      if (toMove > distance) {
        comandaTX = 71, dataTX = 5000,  posilka();                               //Return error toMove > distance
        errorBeep();
        goto vse;
      }
      if (distance <= 30) {
        comandaTX = 71, dataTX = 4000,  posilka();                               //Return error too close
        errorBeep();
        goto vse;
      }
      if ((distance - toMove) <= 30) {
        comandaTX = 71, dataTX = 3000,  posilka();                               //Return error too close for maneuver
        errorBeep();
        goto vse;
      }
      remainMove = (distance - toMove);
      startD = distance;
      prevDist = distance;
      gpio_write_bit(GPIOF, 12, 1);                                               //Enable right
      gpio_write_bit(GPIOF, 14, 1);                                               //Enable left
      gpio_write_bit(GPIOF, 11, 0);                                               //Dir right
      gpio_write_bit(GPIOF, 13, 1);                                               //Dir left
      dac_write_channel(DAC, DAC_CH1, maneuverSpeed + 200), dac_write_channel(DAC, DAC_CH2, maneuverSpeed + 206);   //Go
    }
  }

  delay(20);        //At last all defined, lets watch the move and stop!

theMove:
  distance = 3333;                                                           //dummy set
  while (distance == 3333) {
    getTFminiData(&distance, &strength);
    if (distance <= 30) {
      comandaTX = 71, dataTX = 6000,  posilka();                             //somewhy got too close? wtf? Report and Abort!
      errorBeep();
      goto vse;
    }
    if (distance <= 1200) {
      goto sled;
    }
  }

sled:
  if (prevDist == distance) neEdet++;
  if (prevDist > distance) neEdet = 0;
  if (neEdet >= 33) {                                                        //some specific timeout for default maneuver speed and delay
    comandaTX = 71, dataTX = 2000,  posilka();                               //not moving, abort, report
    errorBeep();
    goto vse;
  }
  prevDist = distance;
  if ((distance - 1) <= remainMove) {
    goto vse;
  }
  delay(20);
  goto theMove;

vse:
  dac_write_channel(DAC, DAC_CH1, 0);                                         //Set right 180-1200
  dac_write_channel(DAC, DAC_CH2, 0);                                         //Set left  180-1200
  gpio_write_bit(GPIOF, 12, 0);                                               //Disable right
  gpio_write_bit(GPIOF, 14, 0);                                               //Disable left
  dac_write_channel(DAC, DAC_CH1, 0), dac_write_channel(DAC, DAC_CH2, 0);     //Stop
  neEdet = 0;
  delay(10);
  comandaTX = 71, dataTX = abs(startD - distance - toMove - 1), posilka();    //Return real turn summary
  while (Serial1.available())  char t = Serial1.read();                       //Purge main serial
}
