//
// Author: Onurcan Cekem
// Version: 0.1
// Date: 22-05-2024
//***************************************************************************
/*
 keyestudio 4wd BT Car
 lesson 5.1
 Ultrasonic Sensor
 http://www.keyestudio.com
*/ 
#include <Arduino.h>
#include <Servo.h>
#include <IRremote.hpp>     //IRremote library statement 

#define DEBUG 1
#define IR_RECEIVE_PIN 3
#define servoPin A3  //servo Pin
#define ML_Ctrl 2  //define the direction control pins of group B motor
#define ML_PWM 5   //define the PWM control pins of group B motor
#define MR_Ctrl 4  //define the direction control pins of group A motor
#define MR_PWM 6   //define the PWM control pins of group A motor

// Pins
int trigPin = 12;    // Trigger
int echoPin = 13;    // Echo
int RECV_PIN = 3;        //define the pins of IR receiver as D3
int LED_PIN = 9;//define the pin of LED as pin 9


// Variables
int pos; //the angle variable of servo
int pulsewidth; //pulse width variable of servo
int distance;
long duration, cm, inches;
char ble_val;// An integer variable used to store the value received by Bluetooth
Servo servo_distance;  // create servo object to control a servo

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(servoPin, OUTPUT);  //set the pins of servo to output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_PIN,OUTPUT);//set pin 9 of LED to OUTPUT

  // IR receiver
  //procedure(0); //set the angle of servo to 0 degree
  servo_distance.attach(A3);  // attaches the servo on pin A3 to the servo object
  servo_distance.write(110);  // Start distance sensor servo at exactly in the middle, because of offset
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  // Motor
  pinMode(ML_Ctrl, OUTPUT);//set direction control pins of group B motor to output
  pinMode(ML_PWM, OUTPUT);//set PWM control pins of group B motor to output
  pinMode(MR_Ctrl, OUTPUT);//set direction control pins of group A motor to output
  pinMode(MR_PWM, OUTPUT);//set PWM control pins of group A motor to output
  
  Serial.println("Damn homie, we chilling");
}

// Function for distance sensor to measure distance
// Input: trigger pin: pin for sensor
// Input: echo    pin: pin for sensor       
// Return: distance in centimeters
int measure_distance(int trigPin, int echoPin)
{
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
   // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  duration = pulseIn(echoPin, HIGH);
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  
  #ifdef DEBUG
  Serial.print("distance sensor: ");
  //Serial.print(inches);
  //Serial.print("in, ");
  Serial.print(cm);
  Serial.println("cm");
  #endif

  return cm;
}

//function to control servo
void procedure(int myangle) {
  pulsewidth = myangle * 11 + 500;  //calculate the value of pulse width
  digitalWrite(servoPin,HIGH);
  delayMicroseconds(pulsewidth);   //The duration of high level is pulse width
  digitalWrite(servoPin,LOW);
  delay((20 - pulsewidth / 1000));  //the cycle is 20ms, the low level last for the rest of time
}

// Function for distance sensor to turn and scan
void turn_distance_sensor()
{
  int distance;
  for (pos = 0; pos <= 180; pos += 2) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //procedure(pos);              // tell servo to go to position in variable 'pos'
    servo_distance.write(pos);              // tell servo to go to position in variable 'pos'
    distance = measure_distance(trigPin, echoPin);
    delay(15);                   //control the rotation speed of servo
  }
  for (pos = 180; pos >= 0; pos -= 2) { // goes from 180 degrees to 0 degrees
    servo_distance.write(pos);              // tell servo to go to position in variable 'pos'
    //procedure(pos);              // tell servo to go to position in variable 'pos'
    distance = measure_distance(trigPin, echoPin);
    delay(15);                    
  }
}

// Driving functions
void drive_forward(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH
  analogWrite(MR_PWM,55);// set the PWM control speed of group A motor to 55
  delay(ms_time);//delay in 2000ms)
}

void drive_back(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);// set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,LOW);//set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

void drive_left(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);//set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH level
  analogWrite(MR_PWM,55);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

void drive_right(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH level
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55 
  digitalWrite(MR_Ctrl,LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

void drive_stop(int ms_time)
{
  digitalWrite(ML_Ctrl, LOW);// set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,0);//set the PWM control speed of group B motor to 0
  digitalWrite(MR_Ctrl, LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,0);//set the PWM control speed of group A motor to 0
  delay(ms_time);//delay in 2000ms)
}

// BT24 is the name of the module

void bluetooth_read()
{
  if (Serial.available() > 0) //Check whether there is data in the serial port cache
  {
    ble_val = Serial.read();  //Read data from the serial port cache
    Serial.print("DATA RECEIVED:");
    Serial.println(ble_val);
    if (ble_val == 'F') {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("led on");
    }
    if (ble_val == 'B') 
    {
      digitalWrite(LED_PIN, LOW);
      Serial.println("led off");
    }
   }
}

void loop() {
  bluetooth_read();
  // Distance sensor
  //distance = measure_distance(trigPin, echoPin);
  
  //Infrared sensor
  if (IrReceiver.decode()) {
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
      IrReceiver.resume(); // Enable receiving of the next value
      delay(1500);
  }
  //Serial.println("Bozo");
  
  //turn_distance_sensor();
  // digitalWrite(9, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(125);
  // // digitalWrite(9, LOW); // turn the LED off by making the voltage LOW
  // delay(125);


  // delay(2000);// delay in 2000ms
  
}
//***************************************************************************