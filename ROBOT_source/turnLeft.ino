//Auto turn LEFT routine based on IMU.

void turnLeft() {
  int oldYAW = heading, neKrutitsa = 0;
  gpio_write_bit(GPIOF, 12, 1);                                               //Enable right
  gpio_write_bit(GPIOF, 14, 1);                                               //Enable left
  gpio_write_bit(GPIOF, 11, 0);                                               //Dir right
  gpio_write_bit(GPIOF, 13, 0);                                               //Dir left
  dac_write_channel(DAC, DAC_CH1, maneuverSpeed);
  dac_write_channel(DAC, DAC_CH2, maneuverSpeed);
  TargetTurn = heading - dataRX;                                              //Mozhet bit azh heading 0-360, turn 0-180 => -180 to 360
  if (TargetTurn < 0) {                                                       //tut idet 0 -179 // +overturn
    TargetTurn = TargetTurn + 360;
    while (heading < 355) {                                                   //slozhnii variant
      delay(10);
      while (Serial1.available())  char t = Serial1.read();                   //purge main serial
      oldYAW = heading, getYAW();
      if (oldYAW == heading) neKrutitsa ++;
      if (oldYAW != heading) neKrutitsa = 0;
      if (neKrutitsa >= 33) {                                                 //Doesn't turn?
        comandaTX = 33, dataTX = 1000, posilka();
        errorBeep();
        goto vse;
      }
    }
  }
  neKrutitsa = 0;
  while (TargetTurn < heading) {                                              //prostoi variant
    delay(10);
    while (Serial1.available())  char t = Serial1.read();                     //purge main serial
    oldYAW = heading, getYAW();
    if (oldYAW == heading) neKrutitsa ++;
    if (oldYAW != heading) neKrutitsa = 0;
    if (neKrutitsa >= 33) {                                                   //Doesn't turn?
      comandaTX = 33, dataTX = 1000, posilka();
      errorBeep();
      goto vse;
    }
  }
vse:
  neKrutitsa = 0;
  dac_write_channel(DAC, DAC_CH1, 0);                                         //Set right 180-1200
  dac_write_channel(DAC, DAC_CH2, 0);                                         //Set left  180-1200
  gpio_write_bit(GPIOF, 12, 0);                                               //Disable right
  gpio_write_bit(GPIOF, 14, 0);                                               //Disable left
  dac_write_channel(DAC, DAC_CH1, 0), dac_write_channel(DAC, DAC_CH2, 0);     //Stop
  delay(75);
  comandaTX = 33, dataTX = abs(heading - TargetTurn), posilka();              //Return real turn summary
}
