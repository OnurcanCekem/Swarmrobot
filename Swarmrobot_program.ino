//
// Author: Onurcan Cekem
// Version: 0.2
// Date: 31-05-2024
//***************************************************************************
/*
 keyestudio 4wd BT Car
 lesson 5.1
 Ultrasonic Sensor
 http://www.keyestudio.com
*/ 
#include <Arduino.h>
#include <Servo.h> // Wheel library
#include <IRremote.hpp>     //IRremote library statement 
#include <SoftwareSerial.h> // Bluetooth library
#include <QMC5883L.h> // Compass library
#include <Wire.h> // I2C for compass library

#define DEBUG
#define IR_RECEIVE_PIN 3 //define the pins of IR receiver as D3
#define IR_SEND_PIN A0
#define servoPin A3  //servo Pin
#define ML_Ctrl 2  //define the direction control pins of group B motor
#define ML_PWM 5   //define the PWM control pins of group B motor
#define MR_Ctrl 4  //define the direction control pins of group A motor
#define MR_PWM 6   //define the PWM control pins of group A motor
#define LED_PIN 9 //define the pin of LED as pin 9
#define DECODE_NEC
//#define IR_SEND_PIN 3

// Pins
int trigPin = 12;    // Ultrasonic Trigger
int echoPin = 13;    // Ultrasonic Echo
IRsend irsend; // IRremote class

// Variables
int pos; //the angle variable of servo
int pulsewidth; //pulse width variable of servo
int distance;
int compass_data; // Variable to store compass data
unsigned long ir_recv_data; // Variable to store received infrared data
unsigned long ir_send_data; // Variable to store received infrared data
unsigned long data = 0xFF02FD; // Variable to store data
long duration, cm, inches;
char ble_val;// An integer variable used to store the value received by Bluetooth
Servo servo_distance;  // create servo object to control a servo
SoftwareSerial bt(0,1); /* Rx,Tx for bluetooth */	
QMC5883L compass; // Compass class

uint8_t grid_map[5][5] = {{0,0,0,0,0},
                          {0,0,0,0,0},
                          {0,0,1,0,0},
                          {0,0,0,0,0},
                          {0,0,0,0,0}};
uint8_t position[2] = {5,5}; // x,y
uint8_t destination_coordinates[2] = {0,0};

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
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // irsend.begin(IR_SEND_PIN);

  // Motor
  pinMode(ML_Ctrl, OUTPUT);//set direction control pins of group B motor to output
  pinMode(ML_PWM, OUTPUT);//set PWM control pins of group B motor to output
  pinMode(MR_Ctrl, OUTPUT);//set direction control pins of group A motor to output
  pinMode(MR_PWM, OUTPUT);//set PWM control pins of group A motor to output

  // Compass
  Wire.begin();
	compass.init();
	compass.setSamplingRate(50);

  bt.begin(9600);	/* Define baud rate for software serial communication */
  Serial.println("Damn homie, we chilling");
  
  delay(1000);
  drive_10cm();
}



// Function for distance sensor HC-SR04 to measure distance
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
  Serial.println(" cm");
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
// It will automatically do a 180 degree spin
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

// Driving functions, Set motor to drive forward
void drive_forward(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH
  analogWrite(MR_PWM,55);// set the PWM control speed of group A motor to 55
  delay(ms_time);//delay in 2000ms)
}

// Set motor to drive backwards
void drive_back(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);// set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,LOW);//set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

// Set motor to drive left
void drive_left(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);//set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH level
  analogWrite(MR_PWM,55);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

// Set motor to drive right
void drive_right(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH level
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55 
  digitalWrite(MR_Ctrl,LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

// Set motor to stop driving
void drive_stop(int ms_time)
{
  digitalWrite(ML_Ctrl, LOW);// set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,0);//set the PWM control speed of group B motor to 0
  digitalWrite(MR_Ctrl, LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,0);//set the PWM control speed of group A motor to 0
  delay(ms_time);//delay in 2000ms)
}

// BT24 is the name of the module
// Function to read bluetooth
uint32_t bluetooth_read()
{
  if (Serial.available() > 0) //Check whether there is data in the serial port cache
  {
    data = Serial.read();  //Read data from the serial port cache
    Serial.print("DATA RECEIVED:");
    Serial.println(data);
    if (data == 'F') {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("led on");
    }
    if (data == 'B') 
    {
      digitalWrite(LED_PIN, LOW);
      Serial.println("led off");
    }
   }
   return data;
}

// Function to send data over bluetooth
void bluetooth_send()
{
  Serial.println(data);
}

// Check infrared and print if something is received
void ir_receive()
{
    //Infrared sensor
  if (IrReceiver.decode()) 
  {
    ir_recv_data = reverseBits(IrReceiver.decodedIRData.decodedRawData); // Decode received data and reverse it so the remote works
    //ir_data = reverseBits(ir_data); // Decode received data and reverse it so the remote works
    Serial.print("IR data received: ");
    Serial.print(ir_recv_data, HEX); // Print "old" raw data in HEX
    Serial.print(' '); // Print "old" raw data
    
    // Attach data to functionality/commands
    switch(ir_recv_data)
    {
      case 0xFF02FD : // Remote OK, the car goes forward
        Serial.print("OK");
        break;
      case 0xFF22DD : // Remote LEFT, the car goes left
        Serial.print("LEFT");
        break;
      case 0xFFC23D : // Remote RIGHT, the car goes right
        Serial.print("RIGHT");
        break;
      case 0xFF629D : // Remote UP, the car goes forward
        Serial.print("UP");
        break;
      case 0xFFA857 : // Remote DOWN, the car goes backwards
        Serial.print("DOWN");
        break;
      case 0xFF6897 : // Remote 1,
        Serial.print("1");
        break;
      case 0xFF9867 : // Remote 2,
        Serial.print("2");
        break;
      case 0xFFB04F : // Remote 3,
        Serial.print("3");
        break;
    }

    Serial.println();
    IrReceiver.resume(); // Enable receiving of the next value
    //IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    //IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data

    // if (ir_data == 0xFF02FD) Serial.println("LET'S GOOOOO");
    delay(500);
  }
}

void ir_senddata(unsigned long data)
{
  // IrReceiver.stop(); // Stop receiving IR, else sent IR echoes back to receiver. <-- Doesn't work
  irsend.sendNECMSB(data, 32);  // Replace with your own unique code
  
  #ifdef DEBUG // Print out sent data in serial when debug is active
  Serial.print("Data sent: ");
  Serial.println(data, HEX); // Convert string input to hex
  // Serial.print("\t Reversed: ");
  // Serial.println(reverseBits(data), HEX);
  #endif
  delay(15); // Force delay, without it IR echoes to receiver
  IrReceiver.resume(); // Resume receiving IR, else sent IR echoes back to receiver.
  return NULL;
}

// Function to reverse from MSB to LSB. Used for IR protocol NEC.
uint32_t reverseBits(uint32_t num) {
  uint32_t reversedNum = 0;

  for (int i = 0; i < 32; i++) {
    // Extract the rightmost bit of num and shift it to the leftmost position
    uint32_t bit = (num >> i) & 1;
    reversedNum |= (bit << (31 - i));
  }
  
  return reversedNum;
}

// Read the serial port
uint32_t readSerial()
{
  if (Serial.available() > 0) 
  {
    // Read the incoming data as a string
    String input = Serial.readStringUntil('\n');

    // Convert the string to an integer
    // Convert the string to an unsigned int (hexadecimal)
    data = strtoul(input.c_str(), NULL, 16);
    // ir_senddata(data); // Send data 
  }
  return data;
}

int readCompass()
{
	int heading = compass.readHeading();
	if(heading==0) {
		/* Still calibrating, so measure but don't print */
	} else {
		Serial.print(heading);
	}
  #ifdef DEBUG
  if (heading > 270) Serial.println("N");
  else if (heading > 180) Serial.println("E");
  else if (heading > 90) Serial.println("S");
  else Serial.println("W");
  #endif
  return heading;
}

void drive_10cm()
{
  int starting_distance, current_distance;
  starting_distance = measure_distance(trigPin, echoPin);
  current_distance = starting_distance;
  Serial.print("Starting 10cm. Distance: ");
  Serial.println(starting_distance);
  delay(1000);
  while (starting_distance - current_distance < 10)
  {
    // Serial.println(current_distance);
    drive_forward(5);
    current_distance = measure_distance(trigPin, echoPin);
  }
  drive_stop(1);
  Serial.print("ended 10cm. Distance: ");
  Serial.println(current_distance);
}




void loop() {
  // Read Bluetooth
  // readSerial();
  // ir_receive();
  // data = bluetooth_read(); 
  // bluetooth_send();
  //irsend.sendNECMSB(reverseBits(data), 32);  // Send reversed data
  // irrecv.enableIRIn();
  
  // Bluetooth testing
  // if (bt.available())	/* If data is available on serial port */
  // {
  //   // Send the data to the Bluetooth serial
  //   bt.write(data);
  //   // Optionally, print the data to the Serial Monitor
  //   // Serial.print("Sent via Bluetooth: ");
  //   // Serial.println(data);
  //   Serial.write(bt.read());	/* Print character received on to the serial monitor */
  // }

  // Distance sensor
  distance = measure_distance(trigPin, echoPin);
  Serial.print("distance = ");
  Serial.println(distance);

  // compass_data = readCompass();
  delay(250);

  // delay(2000);// delay in 2000ms
  
    
  //turn_distance_sensor();
  // digitalWrite(9, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(125);
  // // digitalWrite(9, LOW); // turn the LED off by making the voltage LOW
}
//***************************************************************************