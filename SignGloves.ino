#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SoftwareSerial.h>

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_REALACCEL

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
float valFinger1;
float valFinger2;
float valFinger3;
float valFinger4;
float valFinger5;

// Temporaries
float tempYprMovement = 0;
float yprMovement;
float axyMovement;
float tempAxyMovement = 0;
int falseCounter = 0;
bool isMoving = false;
char bluetoothInput;
bool ledOn = false;

// Configurations
int toleranceRotation = 2;
int toleranceAcc = 100;
int altTxPin = 9;
int altRxPin = 8;
int led = 2;

SoftwareSerial mySerial(altTxPin, altRxPin);

void setup() {
  mySerial.begin(115200);
  Serial.begin(115200);
  mySerial.println("Testicle Flex");
  pinMode(led, OUTPUT)

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
  delay(200);

  if (mySerial.available()) {
    bluetoothInput = mySerial.read();
    Serial.println(bluetoothInput);
  }

  if (bluetoothInput == 'o') {
    digitalWrite(led, HIGH);
    Serial.println("Pressed On");
  } else if (bluetoothInput == 'f') {
    digitalWrite(led, LOW);
    Serial.println("Pressed Off");
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    valFinger1 = analogRead(firstFinger);
    valFinger2 = analogRead(secondFinger);
    valFinger3 = analogRead(thirdFinger);
    valFinger4 = analogRead(fourthFinger);
    valFinger5 = analogRead(fifthFinger);

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

    yprMovement = sqrt((y*y + p*p + r*r));

    axyMovement = sqrt((ax*ax + ay*ay + az*az));
    
    if (abs(tempAxyMovement - axyMovement) > toleranceAcc || abs(tempYprMovement - yprMovement) > toleranceRotation) {
      isMoving = true;
      falseCounter = 0;
    } else {
      isMoving = false;
      falseCounter += 1;
    }

    if (isMoving && falseCounter < 3) {
      mySerial.print(y);Serial.print(",");
      mySerial.print(p);Serial.print(",");
      mySerial.print(r);Serial.print(",");
      mySerial.print(ax);Serial.print(",");
      mySerial.print(ay);Serial.print(",");
      mySerial.print(valFinger1);
      mySerial.print(",");
      mySerial.print(valFinger2);
      mySerial.print(",");
      mySerial.print(valFinger3);
      mySerial.print(",");
      mySerial.print(valFinger4);
      mySerial.print(",");
      mySerial.print(valFinger5);
      mySerial.println();
    }

    tempYprMovement = yprMovement;
    tempAxyMovement = axyMovement;

  }

}
