 #include <PID_v1.h>


#include <I2Cdev.h>

/*
  my plan:

  1. get the angle of the robot     (check)

  2. feed the error of the robot from it's desired position as input into PID controller


  4. PID controller outputs the speed at which the motor should be going


  5. the motor moves at that speed



*/






//VVVVVVV remove useless stuff in this below portion

//MPU DMP code - credit:
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
///////MPU code








double Setpoint, Input, Output;
double Kp = 50, Ki = 0, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);






// declare motor pins
int enablePin1 = 3;
int in1Pin = 8;
int in2Pin = 4;
int in3Pin = 6;
int in4Pin = 7;
int enablePin2 = 5;



float angle;


void setup() {
  // put your setup code here, to run once:


  //Motor set up
  pinMode(enablePin1, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(enablePin2, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  enableMotors();


  setUpMPU();


  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);// turn on PID controller


  // myPID.SetSampleTime(10);//maybe remove line
  myPID.SetOutputLimits(-255, 255);//  myPID.SetOutputLimits(-255, 255);


}

void loop() {
  // put your main code here, to run repeatedly:

  updateAngle();


  Input = ypr[2] * 180 / M_PI;


  Serial.print("current angle:  ");
  Serial.print(Input);
  //
  myPID.Compute();


  Serial.print("         ");
  Serial.println(Output);

  if (Output > 0) {
    motorSpeed(Output);
    backward(0);

    //Serial.println("F");
  }
  else if (Output < 0) {
    motorSpeed(Output * -1);
    forward(0);
    //Serial.println("B");
  }








  //if (angle > 3){
  //  //forward();
  //Serial.println("f");
  //  }
  //else if (angle < -3){
  //  //backward();
  //  Serial.println("b");
  //  }
  //else{
  //  //stop();
  //  Serial.println("s");
  //  }





}
