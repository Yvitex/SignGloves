#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SoftwareSerial.h>

// Configurations
int toleranceRotation = 10;
int toleranceAcc = 100;
int toleranceFlex = 30;
int speed = 100;
int streamLimit = 10;

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

// Data CSV should look like this [Finger1, Finger2, Finger3, Finger4, Finger5, Y, P, R, AX, AY, AZ]

uint8_t devStatus;   
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Final value holders
float y;
float p;
float r;
float ax;
float ay;
float az;

int valFingers[5];

// Temporaries
float tempYprMovement = 0;
float tempAxyMovement = 0;
int tempFingerMovement = 0;
int dataCountSent = 0;

float yprMovement;
float axyMovement;
int fingerMovement;
int falseCounter = 0;
char bluetoothInput;
bool ledOn = false;
bool isMoving = false;
bool debugSwitch = false;
bool isContinuousMode = false;

// Pin outs
int altTxPin = 9;
int altRxPin = 8;
int button = 3;
int led = 2;
int sukunaFingers[5] = {0, 1, 2, 3, 6};

SoftwareSerial mySerial(altTxPin, altRxPin);

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  mySerial.println("Testicle Flex");
  pinMode(led, OUTPUT);
  pinMode(button, INPUT);

  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void loop() {
  delay(speed);

  if (mySerial.available()) {
    bluetoothInput = mySerial.read();
    Serial.println(bluetoothInput);
  }

  switch(bluetoothInput) {
    case 'o': 
      digitalWrite(led, HIGH);
      delay(500);
      break;
    case 'f': 
      digitalWrite(led, LOW);
      delay(500);
      break;
    case 's': 
      debugSwitch = !debugSwitch;
      break;
  }

  if (digitalRead(button) == LOW) {
    isContinuousMode = !isContinuousMode;
    Serial.println("Changed");
    Serial.println(isContinuousMode);
    delay(500);
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    y = ypr[0] * 180/M_PI;
    p = ypr[1] * 180/M_PI;
    r = ypr[2] * 180/M_PI;
      
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    ax = aaReal.x;
    ay = aaReal.y;
    az = aaReal.z;

    valFingers[0] = analogRead(sukunaFingers[0]);
    valFingers[1] = analogRead(sukunaFingers[1]);
    valFingers[2] = analogRead(sukunaFingers[2]);
    valFingers[3] = analogRead(sukunaFingers[3]);
    valFingers[4] = analogRead(sukunaFingers[4]);
    
    yprMovement = sqrt((y*y + p*p + r*r));
    axyMovement = sqrt((ax*ax + ay*ay + az*az));
    fingerMovement = sqrt((sq(valFingers[0]) + sq(valFingers[1]) + sq(valFingers[2]) + sq(valFingers[3]) + sq(valFingers[4])));

    if (abs(tempAxyMovement - axyMovement) > toleranceAcc || abs(tempYprMovement - yprMovement) > toleranceRotation || abs(tempFingerMovement - fingerMovement) > toleranceFlex) {
      isMoving = true;
      falseCounter = 0;
    } else {
      isMoving = false;
      falseCounter += 1;
    }

    if (isMoving && falseCounter < streamLimit && debugSwitch && !isContinuousMode) {
      outputWithBluetooth();
    }

    if (isMoving && falseCounter < streamLimit && !debugSwitch && !isContinuousMode) {
      outputWithComputer();
    }

    if (!debugSwitch && isContinuousMode) {
      outputWithComputer();
    }

    if (debugSwitch && isContinuousMode) {
      outputWithBluetooth();
    }
    
    if (isMoving && falseCounter < streamLimit && !isContinuousMode) {
      digitalWrite(led, HIGH);
      dataCountSent += 1;
    } else if (!isMoving && !isContinuousMode && !falseCounter < streamLimit) {
      digitalWrite(led, LOW);
      // mySerial.print("Stopped");
      // Serial.print("Data Sent: ");
      mySerial.println(dataCountSent);
      dataCountSent = 0;
      delay(500);
      Serial.print("Stopped");
    } else if (isContinuousMode) {
      digitalWrite(led, HIGH);
    }

    tempYprMovement = yprMovement;
    tempAxyMovement = axyMovement;
    tempFingerMovement = fingerMovement;
    
  }

}

void outputWithBluetooth() {
  mySerial.print(y);mySerial.print(",");
  mySerial.print(p);mySerial.print(",");
  mySerial.print(r);mySerial.print(",");
  mySerial.print(ax);mySerial.print(",");
  mySerial.print(ay);mySerial.print(",");
  mySerial.print(az);mySerial.print(",");
  mySerial.print(valFingers[0]);mySerial.print(",");
  mySerial.print(valFingers[1]);mySerial.print(",");
  mySerial.print(valFingers[2]);mySerial.print(",");
  mySerial.print(valFingers[3]);mySerial.print(",");
  mySerial.println(valFingers[4]);
}

void outputWithComputer() {
  Serial.print(y);Serial.print(",");
  Serial.print(p);Serial.print(",");
  Serial.print(r);Serial.print(",");
  Serial.print(ax);Serial.print(",");
  Serial.print(ay);Serial.print(",");
  Serial.print(az);Serial.print(",");
  Serial.print(valFingers[0]);Serial.print(",");
  Serial.print(valFingers[1]);Serial.print(",");
  Serial.print(valFingers[2]);Serial.print(",");
  Serial.print(valFingers[3]);Serial.print(",");
  Serial.println(valFingers[4]);
}
