//Auto move forward routine based on tachometer counting.
//Distance to move in MILLIMETERS

void tachoMoveForward() {
  bool LeftTacho = 0;  //digitalRead(PA6);
  bool RightTacho = 0; //digitalRead(PA0);
  float ToMoveTacho = dataRX * 6.5;          //6.424
  unsigned long Start = 0;
  bool flipped = 0;

  gpio_write_bit(GPIOF, 12, 1);                                               //Enable right
  gpio_write_bit(GPIOF, 14, 1);                                               //Enable left
  gpio_write_bit(GPIOF, 11, 0);                                               //Dir right
  gpio_write_bit(GPIOF, 13, 1);                                               //Dir left
  dac_write_channel(DAC, DAC_CH1, maneuverSpeed + 200), dac_write_channel(DAC, DAC_CH2, maneuverSpeed + 206);   //Go

  while (Start < ToMoveTacho) {
    if ((GPIOA->regs->IDR & 0b0000000000000001) != 0 and (flipped == 0)) {    //(digitalRead(PA0) == 1)
      Start++;
      flipped = 1;
    }
    if ((GPIOA->regs->IDR & 0b0000000000000001) == 0) flipped = 0;
  }

  //pa0 pa6 tacho in
  gpio_write_bit(GPIOG, 0, 0), gpio_write_bit(GPIOG, 1, 0);                   //Brake ON!!
  delay(100);
  gpio_write_bit(GPIOF, 12, 0);                                               //Disable right
  gpio_write_bit(GPIOF, 14, 0);                                               //Disable left
  dac_write_channel(DAC, DAC_CH1, 0), dac_write_channel(DAC, DAC_CH2, 0);     //Stop
  gpio_write_bit(GPIOG, 0, 1), gpio_write_bit(GPIOG, 1, 1);                   //Brake OFF!!
  
  while (Serial1.available())  char t = Serial1.read();                       //Purge main serial
}
