/*
   Choose communication mode define here:
      I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
      SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
*/
#define I2C_MODE
//#define SERIAL_MODE
#include <I2Cdev.h>
#include "OpenCat.h"


/*
   Choose MU address here: 0x60, 0x61, 0x62, 0x63
          default address: 0x60
*/
#define MU_ADDRESS        0x50 //in later versions we set the I2C device to 0x50, 0x51, 0x52, 0x53
#define ALT_MU_ADDRESS    0x60

#include <Arduino.h>
#include <MuVisionSensor.h>

#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif

MuVisionSensor *Mu;
MuVisionSensor Mu0(MU_ADDRESS);
MuVisionSensor Mu1(ALT_MU_ADDRESS);

int xCoord, yCoord; //the x y returned by the sensor
int xDiff, yDiff; //the scaled distance from the center of the frame
int currentX = 0, currentY = 0; //the current x y of the camera's direction in the world coordinate
int range = 100; //the frame size 0~100 on X and Y direction
int skip = 1, counter; //an efforts to reduce motion frequency without using delay. set skip >1 to take effect
int i2cdelay = 3;
long noResultTime = 0;
MuVisionType object[] = {VISION_BODY_DETECT, VISION_BALL_DETECT};
String objectName[] = {"body", "ball"};
int objectIdx = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  uint8_t err = 0;
#ifdef I2C_MODE
  Wire.begin();
  // initialized MU on the I2C port
  err = Mu0.begin(&Wire);
#elif defined SERIAL_MODE
  mySerial.begin(9600);
  // initialized MU on the soft serial port
  err = Mu0.begin(&mySerial);
#endif
  if (err == MU_OK) {
    Serial.println("MU initialized");
    Mu = &Mu0;
  }
  else {
    Serial.println("fail to initialize");
    err = Mu1.begin(&Wire);
    if (err == MU_OK) {
      Serial.println("MU initialized");
      Mu = &Mu1;
    }
    delay(1000);
  }

  pwm.begin();
  pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates

  for (byte i = 0; i < DOF; i++) {
    pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
    servoCalibs[i] = servoCalib(i);
    calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
  }
  //  shutServos();
  counter = 0;
  //  motion.loadBySkillName("rest");
  //  transform(motion.dutyAngles);

  (*Mu).VisionBegin(object[objectIdx]);
  noResultTime = millis();
}

void loop() {
  // read result
  bool objectDetected [] = {false, false};
  for (int test = 0; test < sizeof(object) / 2; test++) //switch between ball and body
    if (test == objectIdx && (objectDetected[objectIdx] = (*Mu).GetValue(object[objectIdx], kStatus))) { // update vision result and get status, 0: undetected, other:
      //      PTL(objectName[objectIdx]);
      noResultTime = millis(); //update the timer

      xCoord = (int)(*Mu).GetValue(object[objectIdx], kXValue);
      yCoord = (int)(*Mu).GetValue(object[objectIdx], kYValue);

      //      Serial.print(xCoord);       // get vision result: x axes value
      //      Serial.print('\t');
      //      Serial.print("y = ");
      //      Serial.println(yCoord);       // get vision result: y axes value
      //      Serial.print("width = ");

      //      if (objectIdx == 1)
      //        switch ((*Mu).GetValue(VISION_BALL_DETECT, kLabel)) {              // get vision result: label value
      //          case MU_BALL_TABLE_TENNIS:
      //            Serial.println("table tennis");
      //            break;
      //          case MU_BALL_TENNIS:
      //            Serial.println("tennis");
      //            break;
      //          default:
      //            Serial.println("unknow ball type");
      //            break;
      //        }
      //delay(i2cdelay);
    }

  if (objectDetected[0] || objectDetected[1]) {
    if (!(counter % skip)) {
      xDiff = max(min((xCoord - range / 2) / 4, 30), -30);
      yDiff = max(min((yCoord - range / 2) / 4, 20), -20);

      currentX = max(min(currentX - min(xDiff, 40), 80), -90);
      currentY = max(min(currentY - min(yDiff, 30), 60), -75);


      int a[DOF] = {currentX / 1.2, 0, 0, 0, \
                    0, 0, 0, 0, \
                    60 - currentY / 2 + currentX / 6, 60 - currentY / 2 - currentX / 6, 90 + currentY / 3 - currentX / 8, 90 + currentY / 3 + currentX / 8, \
                    15 + currentY / 1.2  - currentX / 3, 15 + currentY / 1.2 + currentX / 3, -30 - currentY / 3 + currentX / 4, -30 - currentY / 3 - currentX / 4\
                   };
      transform(a, 4);
    }
  }
  else if (millis() - noResultTime > 2000) {// if no object is detected for 2 seconds, switch object
    objectIdx = (objectIdx + 1) % (sizeof(object) / 2);
    (*Mu).VisionBegin(object[objectIdx]);
    PTL(objectName[objectIdx]);
    noResultTime = millis();
  }
  //    Serial.print("fps = ");
  //    Serial.println(1000/(millis()-time_start));
  //    Serial.println();
}
