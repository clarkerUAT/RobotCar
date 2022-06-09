//Robot will move forward.
//When an object is sensed by utrasonic sensor the robot will slow then stop.
// Robot will back and spin 90 degrees to the left.
// Robot will go forward if path is clear.

//Install libraries needed for this project
#include "TimerOne.h"




// **********************************vehicle speed variables and functions ***************************

//***** variables *******

//disk slot variable
const float diskSlots = 20;
// motor encoder pin
const byte MotorEncoder = 2;
//counter variable
unsigned int counter = 0; 
// tire circumfrance in inches
const float circ = 8.64;

//********* functions **************

//increase count by 1;
void ISR_Count()
{
  counter++;
}
//ISR Timer
void ISR_Timer()
{
  //stop the timer
  Timer1.detachInterrupt();
  float rotation = (counter/circ)* 60.00; // equation for inches per minute
  // set counter back to 0
  counter = 0;
  //Start timer
  Timer1.attachInterrupt(ISR_Timer);
}

//***************************** Robot Operation Functions *****************************

// driving variables
const int rightForward = 6;
const int rightReverse = 7;
const int leftForward = 4;
const int leftReverse = 5;


// driving functions
void Forward()
{
  digitalWrite(rightForward,HIGH);
  digitalWrite(leftForward,HIGH);
}
void LeftReverse()
{
  digitalWrite(leftReverse,HIGH);
  delay(1000);
  Stop();
}

void Stop()
{
  digitalWrite(rightForward,LOW);
  digitalWrite(leftForward,LOW);
  digitalWrite(rightReverse,LOW);
  digitalWrite(leftReverse, LOW);
}

//****************************Analog Sensor Funcionality***************************
#define trigPin 12
#define echoPin 13
const int VCC = 11;

float distance()
{
    float duration, distance;
  digitalWrite(trigPin, LOW); 
  delayMicroseconds(2);
 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  distance = (duration / 2) * 0.0344;

  return distance;
}





void setup() 
{
  //setup functionalits of the timer 
  Serial.begin(9600);
  Timer1.initialize(1000000);
  attachInterrupt(digitalPinToInterrupt(MotorEncoder),ISR_Count,RISING);
  Timer1.attachInterrupt(ISR_Timer);
  //setup funcionality of the sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(VCC,OUTPUT);
  digitalWrite(VCC,HIGH);

}

void loop() {

if(distance()<5.00)
{
  Stop();
  delay(1000);
  LeftReverse();
  Stop();
  delay(1000);
}
else
{
  Forward();
}

}