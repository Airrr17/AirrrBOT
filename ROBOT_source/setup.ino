void setup() {
  pinMode(PG15, OUTPUT);              // On-board LED
  pinMode(PF12, OUTPUT);              // TRACKS: EN1
  pinMode(PF14, OUTPUT);              // TRACKS: EN2
  pinMode(PF11, OUTPUT);              // TRACKS: DIR1
  pinMode(PF13, OUTPUT);              // TRACKS: DIR2
  pinMode(PA0, INPUT);                // TRACKS: tacho 1 in //pulled down, with ~272us pulse + every H1 (turn), 1ms+ (1.15ms) until next
  pinMode(PA6, INPUT);                // TRACKS: tacho 2 in //same
  pinMode(PF2, OUTPUT);               // Speaker
  pinMode(PC0, INPUT_ANALOG);         // Voltage ADC in
  pinMode(PC1, INPUT_ANALOG);         // hatchCurrent ADC in
  pinMode(PC2, INPUT_ANALOG);         // cCurrent ADC in
  pinMode(PB0, OUTPUT);               // Lidar power EN (0-off)
  pinMode(PB9, OUTPUT);               // PWMservo EN    (0-on)
  pinMode(PF3, INPUT);                // Hatch control 1
  pinMode(PF5, INPUT);                // Hatch control 2
  pinMode(PB8, OUTPUT);               // BNO080 reset
  pinMode(PB5, INPUT_PULLUP);         // BNO080 Interrupt, active low, pulls low when the BNO080 is ready for communication
  pinMode(PA1, OUTPUT);               // half-duplex servo control dir. (74hc241 or MAX485)
  pinMode(PG0, OUTPUT);               // Brake L
  pinMode(PG1, OUTPUT);               // Brake R

  gpio_write_bit(GPIOA, 1, HIGH);     // half-duplex control SEE "XL320.cpp" for this pin. HIGH = to servo. ENABLE XL320 CONTROL
  gpio_write_bit(GPIOG, 15, 0);       // Onboard led
  gpio_write_bit(GPIOB, 8, 0);        // IMU reset
  delay(250);
  gpio_write_bit(GPIOG, 15, 1);       // Onboard led
  gpio_write_bit(GPIOB, 8, 1);        // IMU reset
  delay(250);
  gpio_write_bit(GPIOF, 11, 0);       // TEMP for motors enabled
  gpio_write_bit(GPIOF, 12, 1);
  gpio_write_bit(GPIOF, 13, 0);       // & forward
  gpio_write_bit(GPIOF, 14, 1);
  gpio_write_bit(GPIOG, 0, 1);        // Both
  gpio_write_bit(GPIOG, 1, 1);        // Brakes off
  gpio_write_bit(GPIOB, 0, 0);        // Lidar power off
  gpio_write_bit(GPIOB, 9, 1);        // PWMservo power off

  dac_init(DAC, DAC_CH1 | DAC_CH2);   // Enable both DAC channels (Channel 1 and Channel 2).
  pinMode(PA4, OUTPUT);               // DAC_CH1 shares the same pin with SPI1_NSS pin (PA4). So it has to be initialized again, as output.
  gpio_write_bit(GPIOA, 4, LOW);      // DAC low.
  gpio_write_bit(GPIOA, 5, LOW);      // DAC low.
  Serial2.begin(1000000);             // XL-320 dynamixel  PA2  def 1000000
  servo.begin(Serial2);               // XL-320
  Serial1.begin(500000);              // MAIN: 115200/230400/345600/460800/230400/250000/500000/1000000/2000000
  Serial1.setTimeout(100);            // sets the maximum milliseconds to wait for serial data when using serial.readBytesUntil() or serial.readBytes()
  Serial3.begin(115200);              // TFmini PB11, PB10 - 300 degree LIDAR
  Serial3.setTimeout(10);             // cheta ne pomogaet
  Wire.begin();
  Wire.setClock(400000L);                                                      // 400000, tipa 400Hz

  //randomSeed(analogRead(PA7));
  comandaTX = 98, dataTX = random(0, 65000), posilka(), delay(20);             // dlya nachala propihnut`, esli connection uzhe lost
  hatchStatus();                                                               // Check and update `hatch` var
  
  gpio_write_bit(GPIOB, 9, 0);                                                 // PWMservo power Enable
  //pwmController.resetDevices();                                                // Software resets all PCA9685 devices on Wire line
  pwmController.init(B000000);                                                 // Address pins A5-A0 set to B000000
  pwmController.setPWMFrequency(100);                                          // Default is 200Hz, supports 24Hz to 1526Hz
  
  servo.setJointTorque(5, 1023), delay(50);                                    // LIDAR's torque. 0-1023, 100 ele-ele // LIDAR 400 def //1023 full
  servo.setJointTorque(1, 250);                                                // Base's torque. 0-1023, 100 ele-ele  //1023 full
  servo.setJointSpeed(1, 100);                                                 // 1023-Full speed

  if (hatch != 0) CLOSEDOWN();

  pwmController.setChannelPWM(14, 1);                                          // Left headlight dim
  pwmController.setChannelPWM(15, 1);                                          // Right headlight dim

  for ( unsigned char p = 0; p < 100; p++)                                     // Startup beep sound
  {
    gpio_write_bit(GPIOF, 2, 1);
    delay(1);
    gpio_write_bit(GPIOF, 2, 0);
    delay(1);
  }


  delay(100);
  voltage = analogRead(PC0);
  hatchCurrent = analogRead(PC1);
  cCurrent = analogRead(PC2);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);   //Ready to disply, just connect.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("  ROBOT4 06.12.2021");
  display.setCursor(0, 56);
  display.print(voltage * 0.00344);
  display.print(" V");
  display.display();

  if (myIMU.begin() == false)  {
    gpio_write_bit(GPIOG, 15, 0);  //============================if onboard LED on - reset stm32, todo automatic later
    display.setCursor(32, 32);
    display.print("IMU error!");
    display.display();
    while (1);
  }

  //====================================================Tachometers routine====================================================
  Timer2.pause();
  Timer3.pause();
  Timer2.setMode(TIMER_CH1, TIMER_INPUT_CAPTURE);
  Timer3.setMode(TIMER_CH1, TIMER_INPUT_CAPTURE);
  Timer2.setPrescaleFactor(144);  // prescaler to count 72,000,000 / 72 counts per second.
  Timer3.setPrescaleFactor(144);
  Timer2.setInputCaptureMode(TIMER_CH1, TIMER_IC_INPUT_DEFAULT);
  Timer3.setInputCaptureMode(TIMER_CH1, TIMER_IC_INPUT_DEFAULT);
  Timer2.setPolarity(TIMER_CH1, 0);
  Timer3.setPolarity(TIMER_CH1, 0);
  Timer2.setSlaveFlags(TIMER_SMCR_TS_TI1FP1 | TIMER_SMCR_SMS_RESET);
  Timer3.setSlaveFlags(TIMER_SMCR_TS_TI1FP1 | TIMER_SMCR_SMS_RESET);
  Timer2.refresh();
  Timer3.refresh();
  Timer2.resume();
  Timer3.resume();
  //====================================================Tachometers routine====================================================
  servo.LED(1, &rgb[random(0, 7)]), delay(50);
  servo.LED(2, &rgb[random(0, 7)]), delay(50);
  servo.LED(3, &rgb[random(0, 7)]), delay(50);
  servo.LED(4, &rgb[random(0, 7)]), delay(50);

  //servo.setJointSpeed(5, 1023), delay(50);   //0-1023 , 300 norm tak
  //servo.TorqueON(5), delay(50);     //
  //servo.sendPacket(1, XL_ID, 5);
  //servo.sendPacket(5, XL_BAUD_RATE, 3);
}
