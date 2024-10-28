void OPENUP() {//=================================================== OPENUP ========================================================
  int mili = millis();
  gpio_write_bit(GPIOF, 3, 0), gpio_write_bit(GPIOF, 5, 1);       //Start opening. Lets watch the current

  while ((millis() - mili) < 3000) {                              //do
    delay(50);
    if (analogRead(PC1) > 3210) {                                 //Over-current!
      gpio_write_bit(GPIOF, 3, 0), gpio_write_bit(GPIOF, 5, 0);   //Stop
      comandaTX = 21, dataTX = 4,  posilka();                     //Report
      errorBeep();
    }
  }
  hatch = 1;        //done
  hatchStatus();    //lets check again and confirm safe to go on

  if (hatch == 1) {
    pwmController.setChannelPWM(11, 456);                 //rise up lidar by analog servo channel 11
    gpio_write_bit(GPIOB, 0, 1);                          //power on lidar
    servo.LED(5, &rgb[0]);                                //Light it RED
    LIDAR = 1;                                            //All done
  }
  while (Serial1.available())  char t = Serial1.read();   //purge main serial. why not?
}

void CLOSEDOWN() {//================================================ CLOSEDOWN =====================================================
  int mili = millis();
  gpio_write_bit(GPIOB, 0, 0);                                            //power off lidar to save power
  servo.moveJoint(5, lidar_center), delay(500);                           //Center move & wait some
  pwmController.setChannelPWM(11, 700);                                   //retract lidar by analog servo channel 11
  servo.LED(5, &rgb[1]), delay(75), LIDAR = 0;                            //Set it green
  gpio_write_bit(GPIOF, 3, 1), gpio_write_bit(GPIOF, 5, 0);               //Close hatch. Watch current

  while ((millis() - mili) < 3800) {                                      //do
    delay(50);
    if (analogRead(PC1) > 3210) {                                         //Over-current!
      gpio_write_bit(GPIOF, 3, 0), gpio_write_bit(GPIOF, 5, 0);
      comandaTX = 21, dataTX = 4,  posilka();                             //report!
      errorBeep();
    }
  }
  hatch = 0;                                                //set
  hatchStatus();                                            //and confirm
  while (Serial1.available())  char t = Serial1.read();     //purge main serial
}

void hatchStatus() {//================================================ Hatch status =====================================================
  hatch = 3;                                                                                                               // def-dummy
  pinMode(PF3, INPUT);                                                                                                     // Hatch input
  pinMode(PF5, INPUT);                                                                                                     // Hatch input
  if ((GPIOF->regs->IDR & 0b0000000000001000) != 0 and (GPIOF->regs->IDR & 0b0000000000100000) == 0) hatch = 0, comandaTX = 21, dataTX = 0,  posilka();    //Hatch closed
  if ((GPIOF->regs->IDR & 0b0000000000001000) == 0 and (GPIOF->regs->IDR & 0b0000000000100000) != 0) hatch = 1, comandaTX = 21, dataTX = 1,  posilka();    //Hatch opened
  if ((GPIOF->regs->IDR & 0b0000000000001000) != 0 and (GPIOF->regs->IDR & 0b0000000000100000) != 0) hatch = 2, comandaTX = 21, dataTX = 2,  posilka();    //No hatch detected. absent
  if (hatch == 3) comandaTX = 21, dataTX = 3,  posilka();                                                                  // Hatch ERROR?
  pinMode(PF3, OUTPUT);                                                                                                    // Hatch output
  pinMode(PF5, OUTPUT);
}
