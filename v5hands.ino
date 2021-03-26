int printbool = 1;
//PORT 5 DESKTOP PORT 10 LAPTOP
#include <I2Cdev.h>
#include <Wire.h> // Must include Wire library for I2C
#include "I2Cdev.h"
#include "MPU6050.h"
//#include <MPU9250_asukiaaa.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#define TCAADDR 0x70
#endif
#define MPU9250_ADDRESS_AD0_LOW  0x68
#define MPU9250_ADDRESS_AD0_HIGH 0x69
#define ACC_FULL_SCALE_2_G       0x00
#define ACC_FULL_SCALE_4_G       0x08
#define ACC_FULL_SCALE_8_G       0x10
#define ACC_FULL_SCALE_16_G      0x18
#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
#define GYRO_CONFIG_AD 0x1B
#define ACCEL_CONFIG_1_AD 0x1C
#define ACCEL_CONFIG_2_AD 0x1D
#define CONFIG_AD 0x1A
#define PWR_MGMT_1_AD 0x6B
//extern "C" {
//#include "utility/twi.h"  // from Wire library, so we can do bus scanning}
//}
//extern "C"
uint8_t twi_writeTo(uint8_t address, uint8_t* data, uint8_t length, uint8_t wait, uint8_t sendStop)
{
    if (!wait) return 4;
    Wire.beginTransmission(address);
    while (length) {
            Wire.write(*data++);
            length--;
    }
    return Wire.endTransmission(sendStop);
}
MPU6050 accels68[5];

uint8_t sensorId;
int fdata[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t byteBuff[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t buff[68] = {0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0, 0, 0, 0, 0, 0, 0,0,0,0,0,0};
float mvals[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
int16_t axh, ayh, azh, gxh, gyh, gzh;
//Acceleration Limit  |   Sensitivity
//----------------------------------------
//2g                  |    16,384
//4g                  |    8,192
//8g                  |    4,096
//16g                 |    2,048
//
//Angular Velocity Limit  |   Sensitivity
//----------------------------------------
//250º/s                  |    131
//500º/s                  |    65.5
//1000º/s                 |    32.8
//2000º/s                 |    16.4

const float ascale0 = .000061;
const float ascale1 = .000122;
const float ascale2 = .000244;
const float ascale3 = .000488;
const float ascales[4] = {.000061, .000122, .000244, .000488};

const float gscale0 = 0.007633;
const float gscale1 = 0.015267;
const float gscale2 = 0.030534;
const float gscale3 = 0.061068;
const float gscales[4] = {0.007633, 0.015267, 0.030534, 0.061068};

float ascale = 0;
float gscale = 0;
float flexi = 0;
char inByte = 0;         // incoming serial byte
int led = 13;

const int thumbPressure = A0;
const int thumbSwitch = 8;
const int indexf = 9;
const int middlef = 10;
const int ringf  = 11;
const int pinkyf = 12;
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

int16_t t1, t2, t3 = 0;
int iters = 0;
int lim = 1000;
int16_t timeoutcount = 0;
int sensorcount = 0;
int skinstate = 0;

void setup(void)
{
  pinMode(led, OUTPUT);
  pinMode(thumbPressure, INPUT);
  pinMode(thumbSwitch, OUTPUT);
  pinMode(indexf, INPUT_PULLUP);
  pinMode(middlef, INPUT_PULLUP);
  pinMode(ringf, INPUT_PULLUP);
  pinMode(pinkyf, INPUT_PULLUP);
  int ascalar = 3;
  int gscalar = 3;
  while (!Serial);
  delay(1000);
  Wire.begin();
  Wire.setClock(400000L);
  delay(1000);
  Serial.begin(20000000);
  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    delay(100);
    if (printbool == 1) {
      Serial.print("TCA Port #"); Serial.println(t);
    }
    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        if (addr != 0) {
          if (addr == 104) {
            sensorcount++;
            if (printbool == 1) {
              Serial.println("FOUND");
            }
          }
          if (printbool == 1) {
            Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
          }
        }
      }
    }
  }
  for (int i = 0; i < 5; i++) {
    tcaselect(i);
    accels68[i].initialize();
    accels68[i].setFullScaleAccelRange(ascalar);
    accels68[i].setFullScaleGyroRange(gscalar);
    delay(100);
  }
  ascale = ascales[ascalar];
  gscale = gscales[gscalar];
  digitalWrite(led, HIGH);
  delay(100);
  if (printbool == 0) {
    establishContact();
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("R");   // send a capital A
    delay(100);
  }
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(100);
  timeoutcount = 0;
}

int16_t thumbVar = 0;
uint8_t switches[4] = {0, 0, 0, 0};

float starttime=0;
float endtime=0;
float totaltime=0;

void loop() {
  if (printbool == 0) {
    starttime=millis();
    if (Serial.available() > 0)
    {
      thumbVar =  analogRead(thumbPressure);
      thumbVar =  analogRead(thumbPressure);
      digitalWriteFast(thumbSwitch, LOW);
      inByte = Serial.read();
      inByte = Serial.read();
      for (int i = 0; i < 5; i++) {
        bytes6050(accels68[i], i);
      }
      buff[60] = highByte(thumbVar);
      buff[61] = lowByte(thumbVar);
      buff[62] = uint8_t(digitalReadFast(indexf));
      buff[63] = uint8_t(digitalReadFast(middlef));
      buff[64] = uint8_t(digitalReadFast(ringf));
      buff[65] = uint8_t(digitalReadFast(pinkyf));
      buff[66] = highByte(t3);
      buff[67] = lowByte(t3);
      t2 = micros();
      t3 = t2 - t1;
      Serial.write(buff, sizeof(buff));
      t1 = micros();
      timeoutcount = 0;
    }
    else{
      while(Serial.available() == 0){
        totaltime = millis() - starttime;
        if(totaltime > 300){
          Serial.flush();
          digitalWrite(led, HIGH);
          delay(1000);
          establishContact();
        }
      }
    }
  }
  else
  {
    thumbVar =  analogRead(thumbPressure);
    thumbVar =  analogRead(thumbPressure);
    digitalWriteFast(thumbSwitch, LOW);
    t1 = micros();
    for (int i = 0; i < 5; i++) {
      pdata(accels68[i], i);
    }
//    Serial.print(digitalReadFast(indexf));
//    Serial.print(" ");
//    Serial.print(digitalReadFast(middlef));
//    Serial.print(" ");
//    Serial.print(digitalReadFast(ringf));
//    Serial.print(" ");
//    Serial.print(digitalReadFast(pinkyf));
//    Serial.print(" ");
    t2 = micros();
    t3 = t2 - t1;
    buff[60] = highByte(thumbVar);
    buff[61] = lowByte(thumbVar);
    buff[62] = uint8_t(digitalReadFast(indexf));
    buff[63] = uint8_t(digitalReadFast(middlef));
    buff[64] = uint8_t(digitalReadFast(ringf));
    buff[65] = uint8_t(digitalReadFast(pinkyf));
    buff[66] = highByte(t3);
    buff[67] = lowByte(t3);
    Serial.println();
  }
}

void bytes6050(MPU6050 acc, uint8_t i) {
  tcaselect(i);
  acc.getMotionBytes12(&byteBuff[0], &byteBuff[1], &byteBuff[2], &byteBuff[3], &byteBuff[4], &byteBuff[5],
                       &byteBuff[6], &byteBuff[7], &byteBuff[8], &byteBuff[9], &byteBuff[10], &byteBuff[11]);
  //accels
  buff[0 + (i * 12)] = byteBuff[0];
  buff[1 + (i * 12)] = byteBuff[1];
  buff[2 + (i * 12)] = byteBuff[2];
  buff[3 + (i * 12)] = byteBuff[3];
  buff[4 + (i * 12)] = byteBuff[4];
  buff[5 + (i * 12)] = byteBuff[5];
  //gyros
  buff[6 + (i * 12)] = byteBuff[6];
  buff[7 + (i * 12)] = byteBuff[7];
  buff[8 + (i * 12)] = byteBuff[8];
  buff[9 + (i * 12)] = byteBuff[9];
  buff[10 + (i * 12)] = byteBuff[10];
  buff[11 + (i * 12)] = byteBuff[11];
}

void pdata(MPU6050 acc, uint8_t i) {
  t1 = millis();
  tcaselect(i);
  acc.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax );
  Serial.print(", ");
  Serial.print(ay );
  Serial.print(", ");
  Serial.print(az );
  Serial.print(", ");
  Serial.print(gx );
  Serial.print(", ");
  Serial.print(gy );
  Serial.print(", ");
  Serial.print(gz );
  Serial.print(", ");
}

//void pdataarms(MPU6050 acc8, MPU6050 acc9, uint8_t i) {
//  t1 = millis();
//  tcaselect(i);
//  acc8.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  Serial.print(ax );
//  Serial.print(", ");
//  Serial.print(ay );
//  Serial.print(", ");
//  Serial.print(az );
//  Serial.print(", ");
//  Serial.print(gx );
//  Serial.print(", ");
//  Serial.print(gy );
//  Serial.print(", ");
//  Serial.print(gz );
//  Serial.print(", ");
//  acc9.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  Serial.print(ax );
//  Serial.print(", ");
//  Serial.print(ay );
//  Serial.print(", ");
//  Serial.print(az );
//  Serial.print(", ");
//  Serial.print(gx );
//  Serial.print(", ");
//  Serial.print(gy );
//  Serial.print(", ");
//  Serial.print(gz );
//  Serial.print(", ");
//}
