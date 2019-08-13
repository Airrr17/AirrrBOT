#include "libmaple\dac.h"   // Generic STM32F103Z // tested 11.08.19
#include <Wire.h>
//#include <SoftWire.h>
#include "XL320.h"
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306_STM32.h>
#include "PCA9685.h"
#include "SparkFun_BNO080_Arduino_Library.h"
//#include "TFMini.h"
//#include "Adafruit_VL53L0X.h"
#define OLED_RESET -1

unsigned int voltage = 0, rpm = 0, rpm2 = 0, rpm_old, rpm2_old, dataRX = 0, dataTX, servPOS = 0, current_lidar = 501, headingTemp = 0; //0-65535
int bias, pwmR = 0, pwmL = 0, heading = 0, TargetTurn = 0;                                                      //-32768 - 32767
unsigned int a, timeout = 0, timeout2 = 0, hatch = 3;                                          //0-65535  hatch: 0-closed 1-opened 2-absent 3-dummy startup
byte Prinimaem[5], Posilaem[5], comandaRX = 0, comandaTX, RPMena = 0, IMUen = 0, LIDAR = 0;               //byte 0-255
char rgb[] = "rgbypcwo";// servo.LED(254, &rgb[random(0, 7)] );  //0-Red 1-Green 2-Blue 3-Yellow 4-Pink 5-Cyan 6-White 7-NoLED

BNO080 myIMU;
Adafruit_SSD1306 display(OLED_RESET);
XL320 servo;
PCA9685 pwmController;
//Adafruit_VL53L0X FLidar = Adafruit_VL53L0X();
//TFMini tfmini;
//''''''''''''''''''''''''''''''''''''''''''''IMU variables
struct Euler {
  float yaw;
  float pitch;
  float roll;
};
struct Quat {
  float i;
  float j;
  float k;
  float real;
};
Euler getAngles(Quat q, bool degrees);
Quat myQuat;
Euler eul;
//''''''''''''''''''''''''''''''''''''''''''''IMU variables

void setup() {
  pinMode(PG15, OUTPUT);              // On-board LED
  pinMode(PF12, OUTPUT);              // EN1
  pinMode(PF14, OUTPUT);              // EN2
  pinMode(PF11, OUTPUT);              // DIR1
  pinMode(PF13, OUTPUT);              // DIR2
  pinMode(PF2, OUTPUT);               // Speaker
  pinMode(PC0, INPUT_ANALOG);         // Voltage ADC
  pinMode(PB0, OUTPUT);               // Lidar power EN (0-off)
  pinMode(PB9, OUTPUT);               // PWMservo EN    (0-on)
  pinMode(PF3, INPUT);                // Hatch
  pinMode(PF5, INPUT);                // Hatch
  pinMode(PB8, OUTPUT);               // BNO080 reset
  pinMode(PB5, INPUT_PULLUP);         // BNO080 Interrupt, active low, pulls low when the BNO080 is ready for communication.
  gpio_write_bit(GPIOG, 15, 0), gpio_write_bit(GPIOB, 8, 0), delay(250), gpio_write_bit(GPIOG, 15, 1), gpio_write_bit(GPIOB, 8, 1), delay(250);   //LED blink + IMU reset
  gpio_write_bit(GPIOF, 11, 0), gpio_write_bit(GPIOF, 12, 1), gpio_write_bit(GPIOF, 13, 0), gpio_write_bit(GPIOF, 14, 1);                         //TEMP for motors enabled & forward
  gpio_write_bit(GPIOB, 0, 0);        //Lidar power Disable
  gpio_write_bit(GPIOB, 9, 1);        //PWMservo power Disable
  //pinMode(PC3, OUTPUT);               //pc3  -hald-duplex control
  //gpio_write_bit(GPIOC, 3, HIGH);     //pc3  -hald-duplex control
  pinMode(PA0, INPUT_PULLDOWN);       //tacho 1
  pinMode(PA6, INPUT_PULLDOWN);       //tacho 2
  dac_init(DAC, DAC_CH1 | DAC_CH2);   // Enable both DAC channels (Channel 1 and Channel 2).
  pinMode(PA4, OUTPUT);               // DAC_CH1 shares the same pin with SPI1_NSS pin (PA4). So it has to be initialized again, as output.
  gpio_write_bit(GPIOA, 4, LOW);      // DAC low. It's a lot faster than the digitalWrite function
  gpio_write_bit(GPIOA, 5, LOW);      // DAC low
  gpio_write_bit(GPIOE, 8, HIGH);     // enable  DMX TX 6n137
  Serial2.begin(1000000);             // XL-320 dynamixel?  PA2  def 1000000
  servo.begin(Serial2);
  Serial1.begin(500000);              //  115200, 230400, 345600,   460800 230400/250000/500000/1000000/2000000
  Serial1.setTimeout(100);            //sets the maximum milliseconds to wait for serial data when using serial.readBytesUntil() or serial.readBytes()
  Serial3.begin(115200);              //TFmini PB11, PB10
  //  Serial3.setTimeout(10);
  //  tfmini.begin(&Serial3);
  Wire.begin();
  Wire.setClock(400000L);              //400000
  comandaTX = 98, dataTX = random(0, 65000), posilka(), delay(20); // dlya nachala propihnut`, esli connection uzhe lost
  HatchStatus();                      //Check and update `hatch` var
  gpio_write_bit(GPIOB, 9, 0);        //PWMservo power Enable
  pwmController.resetDevices();       // Software resets all PCA9685 devices on Wire line
  pwmController.init(B000000);        // Address pins A5-A0 set to B000000
  pwmController.setPWMFrequency(100); // Default is 200Hz, supports 24Hz to 1526Hz
  servo.setJointTorque(5, 1023), delay(50);   //0-1023, 100 eleele // LIDAR 400 def
  servo.moveJoint(5, current_lidar), delay(500);                    //LIDAR CENTER! current_lidar=501 = 2004 /4
  pwmController.setChannelPWM(11, 700), servo.LED(5, &rgb[1]), delay(50);      //LIDAR DOWN (700-down, 456~-UP), Servo-green
  gpio_write_bit(GPIOF, 3, 1), gpio_write_bit(GPIOF, 5, 0), hatch = 0;         //Close hatch

  for ( unsigned char p = 0; p < 100; p++) //Startup sound
  {
    digitalWrite(PF2, HIGH);
    delay(1);//wait for 1ms
    digitalWrite(PF2, LOW);
    delay(1);//wait for 1ms
  }


  delay(100);
  voltage = analogRead(PC0);
  // tfmini.setSingleScanMode();//  tfmini.setStandardOutputMode()
  randomSeed(analogRead(PA1));

  if (myIMU.begin() == false)  {
    gpio_write_bit(GPIOG, 15, 0);  //onboard LED
    while (1);
  }

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("ROBOT4 Starting.");
  display.println("someline1");
  display.println("someline2");
  display.println("someline3");
  display.println("someline4");
  display.println("someline5");
  display.println("someline6");
  display.print("08.06.2019");
  display.display();

  Timer2.pause();
  Timer3.pause();
  Timer2.setMode(TIMER_CH1, TIMER_INPUT_CAPTURE);
  Timer3.setMode(TIMER_CH1, TIMER_INPUT_CAPTURE);

  Timer2.setPrescaleFactor(144);  // prescaler to count 72,000,000 / 72 counts per second. i.e count microseconds
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

  servo.LED(1, &rgb[random(0, 7)]), delay(50);
  servo.LED(2, &rgb[random(0, 7)]), delay(50);
  servo.LED(3, &rgb[random(0, 7)]), delay(50);
  servo.LED(4, &rgb[random(0, 7)]), delay(50);
  //servo.LED(5, &rgb[random(0, 7)]), delay(50);

  //servo.setJointSpeed(5, 1023), delay(50);   //0-1023 , 300 norm tak
  //servo.TorqueON(5), delay(50);     //
  //servo.sendPacket(1, XL_ID, 5);
  //servo.sendPacket(5, XL_BAUD_RATE, 3);

}


void loop() {///////////////////////////////////////////////////////////////////LOOP START//////////////////////////////////////////////////////////////////
  if (Serial1.available() >= 5) prinimaem();

  if ((Timer2.getInputCaptureFlag(TIMER_CH1)) and (RPMena == 1)) timeout = 0, comandaTX = 117, dataTX = (120000 / Timer2.getCompare(TIMER_CH1)), posilka();  //12000000
  if ((Timer3.getInputCaptureFlag(TIMER_CH1)) and (RPMena == 1)) timeout2 = 0, comandaTX = 118, dataTX = (120000 / Timer3.getCompare(TIMER_CH1)), posilka();  //12000000
  timeout += 1;
  timeout2 += 1;
  if ((timeout >= 65000) and (RPMena == 1)) timeout = 0, comandaTX = 117, dataTX = 0, posilka();
  if ((timeout2 >= 65000) and (RPMena == 1)) timeout2 = 0, comandaTX = 118, dataTX = 0, posilka();

  //display.clearDisplay(), display.setCursor(0, 0), display.print(timeout), display.display();
  // if (timeout2 >= 1000) rpm2 = 0, timeout2 = 0, comandaTX = 118, dataTX = 0, posilka();
  //if (rpm < 5) rpm = 0;
  //if (rpm2 < 5) rpm2 = 0;
  //servo.LED(1, &rgb[random(0, 7)])
  //currTime = millis();
  //  if ((currTime - prevTimeRPM > 2000) and (RPMena == 1)) {
  //    prevTimeRPM = currTime;
  //   if (rpm != rpm_old) {
  //  comandaTX = 117, dataTX = rpm;
  //     posilka();
  //   }
  //   if (rpm2 != rpm2_old) {
  //      comandaTX = 118, dataTX = rpm2;
  //      posilka();
  //    }
  //}
  // rpm_old = rpm, rpm2_old = rpm2;

  if (IMUen == 1) GetYAW();
}///////////////////////////////////////////////////////////////////LOOP END//////////////////////////////////////////////////////////////////

void prinimaem() {///////////////////////////////////////////////////////////////////RX start//////////////////////////////////////////////////////////////////
  Prinimaem[0] = Serial1.read();
  if (Prinimaem[0] != 65) return;    //A - start byte
  Prinimaem[1] = Serial1.read();
  Prinimaem[2] = Serial1.read();
  Prinimaem[3] = Serial1.read();
  Prinimaem[4] = Serial1.read();
  if (Prinimaem[4] != (Prinimaem[0] xor Prinimaem[1] xor Prinimaem[2] xor Prinimaem[3]))    return;

  comandaRX = Prinimaem[1];
  dataRX = (Prinimaem[2] * 256) + Prinimaem[3];

  if (comandaRX == 30) {              // ---------------------------------RIGHT TRACK
    if (dataRX <= 2043) gpio_write_bit(GPIOF, 11, 1), gpio_write_bit(GPIOF, 12, 1);
    if (dataRX >= 2051) gpio_write_bit(GPIOF, 11, 0), gpio_write_bit(GPIOF, 12, 1);
    if (dataRX > 2043 and dataRX < 2051) gpio_write_bit(GPIOF, 12, 0);
    pwmR = dataRX - 2047, pwmR = abs(pwmR);
    dac_write_channel(DAC, DAC_CH1, map(pwmR, 0, 2048, 180, 1200)), comandaRX = 0; //RIGHT DAC, Channel, value  map(value, fromLow, fromHigh, toLow, toHigh)
  }
  if (comandaRX == 31) {              // ---------------------------------LEFT TRACK
    if (dataRX <= 2043) gpio_write_bit(GPIOF, 13, 0), gpio_write_bit(GPIOF, 14, 1);
    if (dataRX >= 2051) gpio_write_bit(GPIOF, 13, 1), gpio_write_bit(GPIOF, 14, 1);
    if (dataRX > 2043 and dataRX < 2051) gpio_write_bit(GPIOF, 14, 0);
    pwmL = dataRX - 2047, pwmL = abs(pwmL);
    dac_write_channel(DAC, DAC_CH2, map(pwmL, 0, 2048, 180, 1200)), comandaRX = 0; //LEFT DAC, Channel, value
  }

  if (comandaRX == 39 and dataRX == 0 and hatch == 0) OPENUP();                                           //LIDAR+power UP 456, servo-red
  if (comandaRX == 39 and dataRX == 1 and hatch == 1) CLOSEDOWN();                                        //LIDAR+power DOWN, servo-green
  if (comandaRX == 41) servo.moveJoint(1, dataRX); //pwmController.setChannelPWM(10, dataRX);//
  if (comandaRX == 42) servo.moveJoint(2, dataRX);
  if (comandaRX == 43) servo.moveJoint(3, dataRX);
  if (comandaRX == 44) servo.moveJoint(4, dataRX);
  //if ((comandaRX == 45) and LIDAR == 1) servo.moveJoint(5, dataRX), comandaRX = 0; //LIDAR rotation!
  if (comandaRX == 116 and dataRX == 1) RPMena = 1;
  if (comandaRX == 116 and dataRX == 0) RPMena = 0;
  //if ((comandaRX == 40) and (dataRX < 6))  comandaRX = 0, ServoPOS();                    //GetServoPos  5 serv-to - eto vremennaya hernya. ne pashet poka
  if (comandaRX == 119 and dataRX == 10) myIMU.enableGameRotationVector(50), IMUen = 1;   //def:50 - Send data update every 50ms myIMU.enableRotationVector(200) or enableGameRotationVector
  if (comandaRX == 119 and dataRX == 0) myIMU.enableGameRotationVector(0), IMUen = 0;      //def:50 - Send data update every 50ms  '''REMOVE, NOT NEEDED'''
  if ((comandaRX == 40) and (LIDAR == 1)) moveLIDAR();                                     //Move LIDAR
  if ((comandaRX == 50) and dataRX == 50 and (LIDAR == 1)) GetDist();                      //Get DISTANCE GetDist()
  if ((comandaRX == 20) and (dataRX == 20)) voltage = analogRead(PC0), comandaTX = 20, dataTX = voltage, posilka();  //Get Battery's Voltage
  if ((comandaRX == 21) and (dataRX == 21)) HatchStatus();                                 //Get hatch status
  if ((comandaRX == 32) and dataRX > 0 and dataRX < 181) TurnRight();
  if ((comandaRX == 33) and dataRX > 0 and dataRX < 181) TurnLeft();
  //  uint16_t strength = tfmini.getRecentSignalStrength();
  comandaRX = 0;
}///////////////////////////////////////////////////////////////////RX end//////////////////////////////////////////////////////////////////

void posilka() {
  Posilaem[0] = 65;    //A
  Posilaem[1] = comandaTX;
  Posilaem[2] = dataTX >> 8;
  Posilaem[3] = dataTX & 0xff;
  Posilaem[4] = (Posilaem[0] xor Posilaem[1] xor Posilaem[2] xor Posilaem[3]);
  Serial1.write (Posilaem, 5);     //serial1.write //  delay(10);
}


void ServoPOS() {    /////////////////////////////////GET servo POSition///////////////////////////
  // servPOS = 0; //
  //  servPOS = servo.getJointPosition(dataRX);      // 3 kostilya  koroch eshe ne pashet
  //  gpio_write_bit(GPIOC, 3, LOW);                 //pc3  -hald-duplex control low = rx
  //  delay(500); //Default value Р В Р вЂ Р В РІР‚С™Р вЂ™Р’В250Р В Р вЂ Р В РІР‚С™Р Р†РІР‚С›РЎС›(500[Р В РЎвЂєР РЋР’Вsec])  //delayMicroseconds

  // if (Serial2.available() >= 1) {
  //while(Serial2.available()) ;
  //   comandaTX = 40, dataTX = Serial2.available(), posilka();
  //
  // gpio_write_bit(GPIOC, 3, HIGH);      //pc3  -hald-duplex control low = rx
}

///////////////////////////////////////// Return the Euler angle structure from a Quaternion structure
Euler getAngles(Quat q) {
  Euler ret_val;
  float x; float y;
  /* YAW */
  x = 2 * ((q.i * q.j) + (q.real * q.k));
  y = (q.real * q.real) - (q.k * q.k) - (q.j * q.j) + (q.i * q.i);
  ret_val.yaw = degrees(atan2(y, x));
  /* PITCH */
  ret_val.pitch = degrees(asin(-2 * (q.i * q.k - q.j * q.real)));
  /* ROLL */
  x = 2 * ((q.j * q.k) + (q.i * q.real));
  y = (q.real * q.real) + (q.k * q.k) - (q.j * q.j) - (q.i * q.i);
  ret_val.roll = degrees(atan2(y , x));

  return ret_val;
}

void moveLIDAR() {     /////////////////////// Rotate Lidar /////////////////////////
  current_lidar = dataRX;
  servo.moveJoint(5, current_lidar);
  //  servo.LED(5, &rgb[random(0, 7)]);
  comandaRX = 0;
}

void GetDist() {     /////////////////////// Distance /////////////////////////
  // hotya tut vrode mozhno  while (Serial3.available())  char t = Serial3.read();    //purge tfmini's serial
  //delay(40);
  int distance = 0;
  int strength = 0;

  getTFminiData(&distance, &strength);
  while (!distance) {
    getTFminiData(&distance, &strength);
    if (distance) {
      comandaTX = 50, dataTX = distance;
      posilka();
      comandaRX = 0;

    }
  }
}

void getTFminiData(int* distance, int* strength) {  /////////////////////// RANGE /////////////////////////
  static char ii = 0;
  char jj = 0;
  int checksum = 0;
  static int rx[9];
  if (Serial3.available())  {
    rx[ii] = Serial3.read();
    if (rx[0] != 0x59) {
      ii = 0;
    } else if (ii == 1 && rx[1] != 0x59) {
      ii = 0;
    } else if (ii == 8) {
      for (jj = 0; jj < 8; jj++) {
        checksum += rx[jj];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      ii = 0;
    } else
    {
      ii++;
    }
    //    while (Serial1.available())  char t = Serial1.read();  //purge main serial
  }
}

void HatchStatus() {  /////////////////////// Hatch /////////////////////////
  hatch = 3;                          // def-dummy
  pinMode(PF3, INPUT);                // Hatch
  pinMode(PF5, INPUT);                // Hatch
  if (digitalRead(PF3) == 1 and digitalRead(PF5) == 0) hatch = 0, comandaTX = 21, dataTX = 0,  posilka();  //Hatch closed
  if (digitalRead(PF3) == 0 and digitalRead(PF5) == 1) hatch = 1, comandaTX = 21, dataTX = 1,  posilka();  //Hatch opened
  if (digitalRead(PF3) == 1 and digitalRead(PF5) == 1) hatch = 2, comandaTX = 21, dataTX = 2,  posilka();  //No hatch detected. absent
  if (hatch == 3) comandaTX = 21, dataTX = 3,  posilka();                                                  //Hatch ERROR?
  pinMode(PF3, OUTPUT);                // Hatch
  pinMode(PF5, OUTPUT);                // Hatch
}

void OPENUP() { /////////////////////// OPENUP /////////////////////////
  gpio_write_bit(GPIOF, 3, 0), gpio_write_bit(GPIOF, 5, 1), delay(3000), hatch = 1;  //Open hatch
  HatchStatus();
  if (hatch == 1) pwmController.setChannelPWM(11, 456), gpio_write_bit(GPIOB, 0, 1), servo.LED(5, &rgb[0]), LIDAR = 1;
  //while (Serial1.available())  char t = Serial1.read();  //purge main serial
}

void CLOSEDOWN() { /////////////////////// CLOSEDOWN /////////////////////////
  gpio_write_bit(GPIOB, 0, 0), servo.moveJoint(5, 501), delay(500), pwmController.setChannelPWM(11, 700), servo.LED(5, &rgb[1]), delay(75), LIDAR = 0;
  gpio_write_bit(GPIOF, 3, 1), gpio_write_bit(GPIOF, 5, 0), hatch = 0;               //Close hatch
  while (Serial1.available())  char t = Serial1.read();  //purge main serial
}

void GetYAW() {
  if (digitalRead(PB5) == 1) return;
  if (myIMU.dataAvailable() == true)  {
    float quatJ = myIMU.getQuatJ();
    float quatI = myIMU.getQuatI();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    //  float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    myQuat.i = quatI;
    myQuat.j = quatJ;
    myQuat.k = quatK;
    myQuat.real = quatReal;
    eul = getAngles(myQuat);
    heading = (eul.yaw + 180);                                // +180 dlya -180->+180 => 0->360
    comandaTX = 120, dataTX = heading, posilka(); /////// YAW
    //comandaTX = 121, dataTX = (eul.pitch+1000), posilka(); ///////00000000000000
    //comandaTX = 122, dataTX = (eul.roll+1000), posilka(); ///////00000000000000
    IMUen = 0;
  }
  // (Serial3.available())  char t = Serial3.read();    //purge tfmini's serial
}

void TurnRight() {                           /////////////////////// TurnRight /////////////////////////
  gpio_write_bit(GPIOF, 12, 1);    //Enable right
  gpio_write_bit(GPIOF, 14, 1);    //Enable left
  gpio_write_bit(GPIOF, 11, 1);    //Dir right
  gpio_write_bit(GPIOF, 13, 1);    //Dir left
  dac_write_channel(DAC, DAC_CH1, 350), dac_write_channel(DAC, DAC_CH2, 350); //Set left  180-1200
  TargetTurn = heading + dataRX;   //Mozhet bit azh 359+180= 539

  if (TargetTurn > 359) {           //tut idet 0-179 // +overturn
    gpio_write_bit(GPIOG, 15, 0);
    TargetTurn = TargetTurn - 360;
    while (heading > 4) {          // slozhnii variant
      delay(10);
      while (Serial1.available())  char t = Serial1.read();  //purge main serial
      GetYAW();
    }
  }

  gpio_write_bit(GPIOG, 15, 1);
  while (TargetTurn > heading) {  // prostoi variant
    delay(10);
    while (Serial1.available())  char t = Serial1.read();  //purge main serial
    GetYAW();
  }
  dac_write_channel(DAC, DAC_CH1, 0); //Set right 180-1200
  dac_write_channel(DAC, DAC_CH2, 0); //Set left  180-1200
  gpio_write_bit(GPIOF, 12, 0); //Disable right
  gpio_write_bit(GPIOF, 14, 0); //Disable left
  delay(50);
  comandaTX = 32, dataTX = abs(heading - TargetTurn), posilka();  // return real turn summary
}

void TurnLeft() {                           /////////////////////// TurnLeft /////////////////////////
  gpio_write_bit(GPIOF, 12, 1);    //Enable right
  gpio_write_bit(GPIOF, 14, 1);    //Enable left
  gpio_write_bit(GPIOF, 11, 0);    //Dir right
  gpio_write_bit(GPIOF, 13, 0);    //Dir left
  dac_write_channel(DAC, DAC_CH1, 350), dac_write_channel(DAC, DAC_CH2, 350); //Set left  180-1200
  TargetTurn = heading - dataRX;   //Mozhet bit azh 0-180= -180

  if (TargetTurn < 0) {           //tut idet 0 -179 // +overturn
    gpio_write_bit(GPIOG, 15, 0);
    TargetTurn = TargetTurn + 360;
    while (heading < 355) {          // slozhnii variant
      delay(10);
      while (Serial1.available())  char t = Serial1.read();  //purge main serial
      GetYAW();
    }
  }

  gpio_write_bit(GPIOG, 15, 1);
  while (TargetTurn < heading) {  // prostoi variant
    delay(10);
    while (Serial1.available())  char t = Serial1.read();  //purge main serial
    GetYAW();
  }
  dac_write_channel(DAC, DAC_CH1, 0); //Set right 180-1200
  dac_write_channel(DAC, DAC_CH2, 0); //Set left  180-1200
  gpio_write_bit(GPIOF, 12, 0); //Disable right
  gpio_write_bit(GPIOF, 14, 0); //Disable left
  delay(50);
  comandaTX = 33, dataTX = abs(heading - TargetTurn), posilka();  // return real turn summary
}
