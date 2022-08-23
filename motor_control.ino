////////////////////////////////////////////////////
// Setup the pins for the motor controller.
// The enablePin1 and enablePin2 MUST be PWM (~) pins



//void loop() {
////You will need to
////get a barrel
////connector to go
// // Sets the speed to full.
// motorSpeed(255);
//
// // Forward for 1.5s
// forward (500);
// // Turn right for 1s
// //right (1000);
// //Forward for 1.5s
// backward (500);
//
// //turn left for 1s
// //left (1000);
//
// //Forward for 1.5s
// //forward (500);
// // Sets the speed to slow.
// //motorSpeed(150);
// //Forward for 1.5s
// //forward (500);
//
// //Stop 2s
// stop (2000);
//}

//Defining functiOns so it's easier to cOntrol the motors.
// Sets the speed for both motors.
// speed: A number between 0 and 255. 0 is stopped and 255 is full speed.
void motorSpeed(int speed)
{
 analogWrite(enablePin1, speed*0.9);// multiplied by 0.9 so the motor powers are equal
 analogWrite(enablePin2, speed);
}
// Sets the speed for motor A.
// speed: A number between 0 and 255. 0 is stopped and 255 is full speed.
void motorASpeed(int speed)
{
 analogWrite(enablePin1, speed);
}
// Sets the speed for motor B.
// speed: A number between 0 and 255. 0 is stopped and 255 is full speed.
void motorBSpeed(int speed)
{
 analogWrite(enablePin2, speed);
}
void motorAForward()
{
 digitalWrite (in1Pin, HIGH);
 digitalWrite (in2Pin, LOW);
}
void motorBForward()
{
 digitalWrite (in3Pin, LOW);
 digitalWrite (in4Pin, HIGH);
}
void motorABackward()
{
 digitalWrite (in1Pin, LOW);
 digitalWrite (in2Pin, HIGH);
}
void motorBBackward()
{
 digitalWrite (in3Pin, HIGH);
 digitalWrite (in4Pin, LOW);
}
void motorAStop()
{
 digitalWrite (in1Pin, LOW);
 digitalWrite (in2Pin, LOW);
}
void motorBStop()
{
 digitalWrite (in3Pin, LOW);
 digitalWrite (in4Pin, LOW);
}
void motorAOn()
{
 digitalWrite (enablePin1, HIGH);
}
void motorBOn()
{
 digitalWrite (enablePin2, HIGH);
}
void motorAOff()
{
 digitalWrite (enablePin1, LOW);
}
void motorBOff()
{
 digitalWrite (enablePin2, LOW);
}
// Movement functiOns
void forward (int duration)
{
 motorAForward();
 motorBForward();
 delay (duration);
}
void backward (int duration)
{
 motorABackward();
 motorBBackward();
 delay (duration);
}
void left (int duration)
{
 motorABackward();
 motorBForward();
 delay (duration);
}
void right (int duration)
{
 motorAForward();
 motorBBackward();
 delay (duration);
}
void stop (int duration)
{
 motorAStop();
 motorBStop();
 delay (duration);
}
void disableMotors()
{
 motorAOff();
 motorBOff();
}
void enableMotors()
{
 motorAOn();
 motorBOn();
}
