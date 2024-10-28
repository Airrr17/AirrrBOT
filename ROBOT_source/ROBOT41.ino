#include "libmaple\dac.h"                     //                                                                          Generic STM32F103Z // AirrrBOT
#include <Wire.h>
#include "XL320.h"                            //Smart Digital Servos, library must be modified: search for "int XL320::getJointPosition(int id)" in XL320.cpp
#include <math.h>
#include <Adafruit_GFX.h>                     //OLED Debug secreen. v 1.7.5(!!!)
#include <Adafruit_SSD1306_STM32.h>           //OLED
#include "PCA9685.h"                          //16-ch analog servo controller (PWM) also controlling LEDs
#include "SparkFun_BNO080_Arduino_Library.h"  //IMU - Inertial Measurement Unit (+ magnetic compass, not used now, drifting)

#define OLED_RESET -1

//Vars:
unsigned int voltage = 0;
unsigned int rpm = 0;
unsigned int rpm2 = 0;
unsigned int rpm_old;
unsigned int rpm2_old;
unsigned int dataRX = 0;
unsigned int dataTX;
unsigned int current_lidar = 512;
unsigned int headingTemp = 0;              //0-65535
unsigned int a;
unsigned int timeout1 = 0;
unsigned int timeout2 = 0;
unsigned int hatch = 3;
unsigned int cCurrent = 0;
unsigned int hatchCurrent = 0;                                                 //0-65535  hatch: 0-closed 1-opened 2-absent 3-dummy startup
int bias;
int pwmR = 0;
int pwmL = 0;
int heading = 0;
int TargetTurn = 0;
int maneuverSpeed = 400;
int lidar_center = 512;                                    //-32768 - 32767
byte Prinimaem[5];
byte Posilaem[5];
byte comandaRX = 0;
byte comandaTX;
byte RPMena = 0;
byte IMUen = 1;
byte LIDAR = 0;                                            //byte 0-255
char rgb[] = "rgbypcwo";                                                                                                               //Servo LEDS: 0-Red 1-Green 2-Blue 3-Yellow 4-Pink 5-Cyan 6-White 7-NoLED

struct Euler {  //====================================================IMU variables================================
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
//====================================================IMU variables====================================================

BNO080 myIMU;
Adafruit_SSD1306 display(OLED_RESET);
XL320 servo;
PCA9685 pwmController;


void loop() {//====================================================LOOP START====================================================
  if (Serial1.available() >= 5) prinimaem();       //Odnoznachno

  if ((Timer2.getInputCaptureFlag(TIMER_CH1)) and (RPMena == 1)) timeout1 = 0, comandaTX = 117, dataTX = (120000 / Timer2.getCompare(TIMER_CH1)), posilka();   //12000000
  if ((Timer3.getInputCaptureFlag(TIMER_CH1)) and (RPMena == 1)) timeout2 = 0, comandaTX = 118, dataTX = (120000 / Timer3.getCompare(TIMER_CH1)), posilka();   //12000000
  timeout1 += 1;
  timeout2 += 1;
  if ((timeout1 >= 65000) and (RPMena == 1)) timeout1 = 0, comandaTX = 117, dataTX = 0, posilka();          //Autozeroing
  if ((timeout2 >= 65000) and (RPMena == 1)) timeout2 = 0, comandaTX = 118, dataTX = 0, posilka();          //On timeout

  if (IMUen == 1) getYAW();
}//=========================================================LOOP END============================================================



void posilka() {//=========================================================PACKET FORMING============================================================
  Posilaem[0] = 65;            // it's Airrr :)
  Posilaem[1] = comandaTX;
  Posilaem[2] = dataTX >> 8;
  Posilaem[3] = dataTX & 0xff;
  Posilaem[4] = (Posilaem[0] xor Posilaem[1] xor Posilaem[2] xor Posilaem[3]);
  Serial1.write (Posilaem, 5);
}


void servoPOS() {//=======================================================getServoPosition=========================================================
  int servPOS = servo.getJointPosition(dataRX);
  if (servPOS < 0) servPOS = 2002 + servPOS;                                //-2 servo not detected, -1 packet error. So set `range` to abnormal value. Represent error #.
  unsigned int complexPosition = (dataRX << 11) | servPOS;                  //bit0-10 10-bit position, bit 11-15 servo #.
  comandaTX = 45, dataTX = complexPosition,  posilka();                     //sendout requested servo's position.
}

//======================================Return the Euler angle structure from a Quaternion structure. from example======================================
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

void moveLIDAR() {//======================================================= moveLIDAR =========================================================
  current_lidar = dataRX;
  servo.moveJoint(5, current_lidar);          //simple
  comandaRX = 0;
}

void getDist() {//================================================ Distance at current angle. legacy =====================================================
  while (Serial3.available())  char t = Serial3.read();    //purge tfmini's serial
  //delay(40);
  int distance = 0;
  int strength = 0;
  //  getTFminiData(&distance, &strength);                           //hvatit?
  while (!distance) {
    getTFminiData(&distance, &strength);
    if (distance) {
      comandaTX = 50, dataTX = distance;
      posilka();
      comandaRX = 0;
    }
  }
}
void getDistNew() {//================================================ Distance NEW implementation =====================================================
  int distance = 0;
  int strength = 0;
  servo.moveJoint(5, dataRX);                                      //dvigaem na tsel'
  int raznitsa = current_lidar - dataRX;
  raznitsa = abs(raznitsa) + 11;                                   // 10ms = 100Hz of TFmini
  current_lidar = dataRX;
  //getTFminiData(&distance, &strength);                           //fire raz

  delay (raznitsa);                                                //tut  dynamic delay !!!!!!!!!!!!!!!!!!!!
  while (Serial3.available())  char t = Serial3.read();            //purge

  while (!distance) {
    getTFminiData(&distance, &strength);                           //fire raz

    if (distance) {                                                //Here goes non-standart packet forming "R"+distance+current_lidar
      comandaTX = current_lidar;                                   //comandaTX = pervie 8 bit beret iz 12ti prosto
      unsigned int temp_forming = current_lidar >> 8;              //ostalnie 4 bita
      if (distance > 6666) distance == 6666;                       //mozhet ne nado, potom ogranichit'. mozhet 1200?
      dataTX = (temp_forming << 12) | distance;                    //dvigaem te 4 bita i nakladivaem na 12 bit distancii

      Posilaem[0] = 82;                                            //R - radar?
      Posilaem[1] = comandaTX;
      Posilaem[2] = dataTX >> 8;
      Posilaem[3] = dataTX & 0xff;
      Posilaem[4] = (Posilaem[0] xor Posilaem[1] xor Posilaem[2] xor Posilaem[3]);
      Serial1.write (Posilaem, 5);
      comandaRX = 0;
    }
  }
}

void getTFminiData(int* distance, int* strength) {//================================================//GET DIST//================================================//
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
  }
}


void getYAW() {//================================================ GetYAW =====================================================
  if (gpio_read_bit(GPIOB, 5) != 0) return;                   //IMU data ready pin. EXIT if nothing
  if (myIMU.dataAvailable() == true)  {
    float quatJ = myIMU.getQuatJ();
    float quatI = myIMU.getQuatI();
    float quatK = myIMU.getQuatK();
    float quatReal = myIMU.getQuatReal();
    //float quatRadianAccuracy = myIMU.getQuatRadianAccuracy();
    myQuat.i = quatI;
    myQuat.j = quatJ;
    myQuat.k = quatK;
    myQuat.real = quatReal;
    eul = getAngles(myQuat);
    heading = (eul.yaw + 180);                                // +180 dlya -180->+180 => 0->360
    if (dataRX == 10) comandaTX = 120, dataTX = heading, posilka();             /////// YAW   if (dataRX == 10) iz priema znaem. 10=yaw
    if (dataRX == 11) comandaTX = 121, dataTX = (eul.roll), posilka();          /////// PITCH, pitch and roll are swapped!
    if (dataRX == 12) comandaTX = 122, dataTX = (eul.pitch + 90), posilka();    /////// ROLL, t.k. ne v toi ploskosti IMU. Thats ok.
    if (dataRX == 17) {                                      // All 3D at once
      comandaTX = 120, dataTX = heading, posilka();
      delay(5);            //5-15...
      comandaTX = 121, dataTX = (eul.roll), posilka();
      delay(5);
      comandaTX = 122, dataTX = (eul.pitch + 90), posilka();
    }
    IMUen = 0;
  }
}



void errorBeep() {                                                            //ERROR beep sound
  dac_write_channel(DAC, DAC_CH1, 0), dac_write_channel(DAC, DAC_CH2, 0);     //Stop the tracks!
  for (unsigned char p = 0; p < 100; p++)
  {
    gpio_write_bit(GPIOF, 2, 1);
    delay(1);//wait for 1ms
    gpio_write_bit(GPIOF, 2, 0);
    delay(1);//wait for 1ms
  }
  for (unsigned char p = 0; p < 100; p++)
  {
    gpio_write_bit(GPIOF, 2, 1);
    delay(2);//wait for 1ms
    gpio_write_bit(GPIOF, 2, 0);
    delay(2);//wait for 1ms
  }
  while (Serial1.available())  char t = Serial1.read();                        //purge main serial
}




//    display.clearDisplay();
//    display.setCursor(0, 40);
//    display.print("te= ");
//    display.println(te);
//    display.display();
//if (gpio_read_bit(GPIOF, 3) != 0 and gpio_read_bit(GPIOF, 5) != 0) hatch = 2, comandaTX = 21, dataTX = 2,  posilka();
