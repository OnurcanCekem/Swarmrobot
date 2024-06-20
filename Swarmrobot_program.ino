//
// Author: Onurcan Cekem
// Version: 0.5
// Date: 20-06-2024
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
// #define BLUETOOTH
#define IR_RECEIVE_PIN 3 //define the pins of IR receiver as D3
#define IR_SEND_PIN A0
#define servoPin A3  //servo Pin
#define ML_Ctrl 2  //define the direction control pins of group B motor
#define ML_PWM 5   //define the PWM control pins of group B motor
#define MR_Ctrl 4  //define the direction control pins of group A motor
#define MR_PWM 6   //define the PWM control pins of group A motor
#define LED_PIN 9 //define the pin of LED as pin 9
#define DECODE_NEC

// Pins
int trigPin = 12;    // Ultrasonic Trigger
int echoPin = 13;    // Ultrasonic Echo
IRsend irsend; // IRremote class
IRrecv irrecv(IR_SEND_PIN);

// Variables
int pos; //the angle variable of servo
int pulsewidth; //pulse width variable of servo
int distance;
int compass_data; // Variable to store compass data
unsigned long data = 0xFF02FD; // Variable to store Serial.available data
long duration, cm, inches;
char ble_val;// An integer variable used to store the value received by Bluetooth
bool leader = 0; // A variable to keep track whether this robot is leader/slave (0 = slave, 1 = leader)
Servo servo_distance;  // create servo object to control a servo

// IR
unsigned long ir_recv_data; // Variable to store received infrared data
unsigned long ir_send_data; // Variable to store received infrared data
uint32_t senddata = 0xD00000;

// Bluetooth
#ifdef BLUETOOTH
SoftwareSerial bt(0,1); /* Rx,Tx for bluetooth */	
#endif BLUETOOTH

// Compass
QMC5883L compass; // Compass class
int calibrated_North = 135;
int calibrated_East = 225;
int calibrated_South = 315;
int calibrated_West = 45;

// Robot mapping
uint8_t grid_map[5][5] = {{0,0,0,0,0},
                          {0,0,0,0,0},
                          {0,0,1,0,0},
                          {0,0,0,0,0},
                          {0,0,0,0,0}};
const int numRows = 5;
const int numCols = 5;
int position_y = 2;
int position_x = 2; // x,y
uint8_t robot_ID = 0;
unsigned int stored_ids[5] = {0,0,0,0,0};
unsigned int MAC_ID = 0; // Generate MAC_ID, an ID that's randomly generated and won't change after


void setup() {
  //Serial Port begin
  Serial.begin (9600);

    // IR receiver
  servo_distance.attach(A3);  // attaches the servo on pin A3 to the servo object
  // servo_distance.write(110);  // Start distance sensor servo at exactly in the middle, because of offset
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // IrReceiver.disableIRIn(); IrReceiver.enableIRIn();

  // IR sender
  // irsend.begin(IR_SEND_PIN); // Call this function to send whenever I'm sending

  // HC-SR04 distance sensor
  pinMode(servoPin, OUTPUT);  //set the pins of servo to output
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LED_PIN,OUTPUT);//set pin 9 of LED to OUTPUT
  //procedure(0); //set the angle of servo to 0 degree


  // Motor
  pinMode(ML_Ctrl, OUTPUT);//set direction control pins of group B motor to output
  pinMode(ML_PWM, OUTPUT);//set PWM control pins of group B motor to output
  pinMode(MR_Ctrl, OUTPUT);//set direction control pins of group A motor to output
  pinMode(MR_PWM, OUTPUT);//set PWM control pins of group A motor to output

  // Compass
  Wire.begin();
	compass.init();
	compass.setSamplingRate(50);

  // Bluetooth
  #ifdef BLUETOOTH
  bt.begin(9600);	/* Define baud rate for software serial communication */
  #endif BLUETOOTH
  Serial.println("Damn homie, we chilling");
  
  // Mapping stuff
  // unsigned int MAC_ID = random(1, 254); // Generate MAC_ID, an ID that won't change
  randomSeed(analogRead(0));
  MAC_ID = random(1, 254); // Generate MAC_ID, an ID that won't change
  
  delay(1000);
  phase_0(); // Start program to determine leader
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

#ifdef BLUETOOTH
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
#endif BLUETOOTH

// Check infrared and print if something is received
void ir_receive()
{
  // irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
    //Infrared sensor
  if (irrecv.decode()) 
  {
    ir_recv_data = reverseBits(irrecv.decodedIRData.decodedRawData); // Decode received data and reverse it so the remote works
    //ir_data = reverseBits(ir_data); // Decode received data and reverse it so the remote works
    #ifdef DEBUG
    Serial.print("IR data received: ");
    Serial.print(ir_recv_data, HEX); // Print "old" raw data in HEX
    Serial.print(' '); // Print "old" raw data
    #endif
    unsigned long ir_temp = ir_recv_data >> 20; // Only grab first 4 bits
    if (ir_temp == 0xE) // Check first 4 bits, if it's 15 (E) enable map shennanigans protocol
                        // E----- map protocol
                        // E12345. 1 & 2 = ID, 3 = X coords, 4 = Y coords, 5 = empty
    {
      robot_ID = (ir_recv_data >> 12) & 0xFF; // Grab the second digit
      int destination_x = (ir_recv_data >> 8) & 0xF; // Grab the third digit
      int destination_y = (ir_recv_data >> 4 ) & 0xF; // Grab the fourth digit
      Serial.print("Destination X: ");
      Serial.println(destination_x);
      Serial.print("Destination Y: ");
      Serial.println(destination_y);

      // Send D12345, 1 = ID, 2&3 = MAC_ID, D = acknowledge
      long senddata = (0xD << 20) + (robot_ID << 16) + (MAC_ID << 8);
      ir_senddata(senddata);
    }
    else
    {
      #ifdef DEBUG
      Serial.println(ir_recv_data);
      #endif
    }
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
    irrecv.resume(); // Enable receiving of the next value
    //IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
    //IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data

    // if (ir_data == 0xFF02FD) Serial.println("LET'S GOOOOO");
    delay(500);
  }
}

// Function to send data through IR.
void ir_senddata(unsigned long data)
{
  irsend.begin(IR_SEND_PIN); // Enable sending IR.
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

// Read the serial port for sending IR data
uint32_t readSerial_IR()
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

// Function to read compass
// Returns degrees (1-360)
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

// Function to turn the car until given degrees of direction
void turn_until_degrees(int desired_direction)
{
  int current_direction = readCompass(); 
  int difference = abs(current_direction-desired_direction);
  int left, right; // counterclockwise
  Serial.println("Difference: ");
  Serial.println(difference);
  while(difference >= 4) // Keep in function until direction is achieved
  {
    // Calculate difference
    left = (current_direction - desired_direction + 360) % 360; // counterclockwise
    right = (desired_direction - current_direction + 360) % 360; // clockwise

  // Serial.print("Left: ");
  // Serial.print(left);
  // Serial.print("\t right: ");
  // Serial.println(right);

    // Determine shortest turn direction
    if (left <= right) // Go left
    {
      drive_left(5); // counterclockwise
    }
    else // Go right
    {
      drive_right(5); // clockwise
    }

    current_direction = readCompass();  // Update current direction
    // Serial.print("current direction in while: ");
    // Serial.println(current_direction);
    difference = abs(current_direction-desired_direction);
  }
  drive_stop(1);
  Serial.print("Achieved desired: ");
  Serial.println(desired_direction);
}

// Function to drive 10 cm forwards
void drive_10cm()
{
  int starting_distance, current_distance;
  starting_distance = measure_distance(trigPin, echoPin);
  current_distance = starting_distance;
  Serial.print("Starting 10cm. Distance: ");
  Serial.println(starting_distance);
  delay(500);
  while (starting_distance - current_distance < 10)
  {
    // Serial.println(current_distance);
    
    // Check if nothing is in front of car
    current_distance = measure_distance(trigPin, echoPin);
    if(current_distance <= 10)
    {
      Serial.println("BREAK, BELOW 10 CM");
      break;
    }
    else drive_forward(5); // Else drive forward
  }
  drive_stop(1);
  Serial.print("ended 10cm. Distance: ");
  Serial.println(current_distance);
}

// Function to print the map grid.
// Also updates position_y and y with wherever a 1 in the map is
void print_map()
{
  for(int row = 0; row < numRows; row++)
  {
    for(int col = 0; col < numCols; col++)
    {
      // Find x and y of current location
      if(grid_map[row][col] == 1)
      {
        position_y = row;
        position_x = col;
      }
      Serial.print(grid_map[row][col]); // Print map
      Serial.print("\t");
    }
    Serial.println();
  }
  // Print x and y coordinates
  Serial.print("X: ");
  Serial.print(position_x);
  Serial.print(" Y: ");
  Serial.print(position_y);
  Serial.println();
}

/* Function to select a coordinates and drive to it
  param desired_x: x-coordinates
  param desired_y: y-coordinates
*/
void goto_coordinates(int desired_x, int desired_y)
{
  int heading;
  // Drive to x
  for(;;)
  {
    heading = readCompass(); // Read compass
    if(position_x > desired_x) // Go left/West
    {
      turn_until_degrees(calibrated_West);
      drive_10cm();
      
      // Update map
      grid_map[position_y][position_x] = 0; // Remove old position
      position_x -= 1;
      grid_map[position_y][position_x] = 1; // Update current position
      Serial.println("Go West");
      // if (heading <= 90) drive_10_cm();
    }
    else if (position_x < desired_x) // Go right/East
    {
      turn_until_degrees(calibrated_East);
      drive_10cm();
      // Update map
      grid_map[position_y][position_x] = 0; // Remove old position
      position_x += 1;
      grid_map[position_y][position_x] = 1; // Update current position
      Serial.println("Go East");
    }
    else break; // position_x == desired_x // Desired x is found
    Serial.println("Next pos");
    print_map();
    delay(500);
  }

  // Drive to y
  for(;;)
  {
    heading = readCompass(); // Read compass
    if(position_y > desired_y) // Go left/West
    {
      turn_until_degrees(calibrated_North);
      drive_10cm();
      
      // Update map
      grid_map[position_y][position_x] = 0; // Remove old position
      position_y -= 1;
      grid_map[position_y][position_x] = 1; // Update current position
      Serial.println("Go North");
      // if (heading <= 90) drive_10_cm();
      // else ;
    }
    else if (position_y < desired_y) // Go right/East
    {
      turn_until_degrees(calibrated_South);
      drive_10cm();
      // Update map
      grid_map[position_y][position_x] = 0; // Remove old position
      position_y += 1;
      grid_map[position_y][position_x] = 1; // Update current position
      Serial.println("Go South");
    }
    else break; // position_y == desired_x // Desired x is found
    Serial.println("Next pos");
    print_map();
    delay(500);
  }

  Serial.println("Destination arrived.");
}

// Function for start of program, checks if leader exists, else assign one.
void phase_0()
{
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) // 5 second timer
  { 
    ir_receive();
    if(ir_recv_data == 0xDFFFFF) 
    {
      // If leader broadcast is found
      Serial.println("Leader is found, I'm a slave now");
      leader = 0;
      break;
    }
    leader = 1;
    delay(25);
  }
  // No leader is found, this robot becomes leader and starts broadcasting
  if (leader == 1)
  {
    Serial.print("I'm a leader ");
    senddata = 0xDFFFFF;
    Serial.print("LEADER Sending data: ");
    Serial.println(senddata, HEX);
    ir_receive();

    ir_senddata(senddata);
    if ((ir_recv_data >> 20) == 0xD)
    {
      uint8_t temp_ID = 0xFF;
      temp_ID &= (ir_recv_data >> 8); // --XX--
      Serial.print("Slave found on ID: ");
      Serial.println(temp_ID, HEX);

    }
    return NULL; // Break out of function
  }

  
  // leader is found, this robot is a slave and starts handshaking
  Serial.print("I'm a slave ");
  senddata = 0xD00000;
  senddata |= (uint32_t)MAC_ID << 8; // --XX--
  ir_senddata(senddata);
  Serial.print("SLAVE Sending data: ");
  Serial.println(senddata, HEX);



  ir_receive();
  ir_recv_data = reverseBits(IrReceiver.decodedIRData.decodedRawData); // Decode received data and reverse it so the remote works
  // ir_recv_data = 0xE12200;
  //ir_data = reverseBits(ir_data); // Decode received data and reverse it so the remote works
  Serial.print("IR data received: ");
  Serial.print(ir_recv_data, HEX); // Print "old" raw data in HEX
  Serial.print(' '); // Print "old" raw data
  unsigned long ir_temp = ir_recv_data >> 20; // Only grab first 4 bits
  robot_ID = (ir_recv_data >> 16) & 0xF; // -X---- Grab the second digit
  // robot_ID = 15;
  senddata = 0xD00000;
  int destination_x = (ir_recv_data >> 12) & 0xF; // --X---Grab the third digit
  int destination_y = (ir_recv_data >> 8 ) & 0xF; // ---X--Grab the fourth digit
  // ir_data[0] = 0xD;
  // ir_data[1] = robot_ID;
  // ir_data[2] = MAC_ID >> 4;
  // ir_data[3] = MAC_ID;
  // ir_data[4] = 0;
  // ir_data[5] = 0;
  senddata |= (uint32_t)robot_ID << 16; // -X----
  senddata |= (uint32_t)MAC_ID << 8; // --XX--
  Serial.print("ID: ");
  Serial.print(robot_ID, HEX);
  Serial.print("\t MAC_ID: ");
  Serial.print(MAC_ID, HEX);
  // Serial.print("\t Destination X: ");
  // Serial.print(destination_x);
  // Serial.print("\t Destination Y: ");
  // Serial.println(destination_y);
  Serial.print(" Senddata: "); // Senddata is D12345, 1 = ID, 2&3 = MAC_ID
  // Serial.println(senddata, BIN);
  
  // for (int i = 23; i >= 0; i--) { // Print all of senddata's bits
  //   Serial.print(bitRead(senddata, i));
  // }
  Serial.println("");
  Serial.print(senddata, HEX);
  Serial.print("\t ");
  Serial.print("0 - D ");
  Serial.print("\t ");
  Serial.print(" 1 - ID ");
  Serial.print("\t ");
  Serial.print(" 2+3 - MAC_ID ");
  if ((ir_temp & 0xF) == 0xE) // Check first 4 bits, if it's 15 (E) enable map shennanigans protocol
                    // E----- map protocol
                    // E12345. 1 = ID, 2 = X coords, 3 = Y coords, 4 = empty, 5 = empty
  {

  }
  else
  {
    // Switch case
    // Serial.println(ir_recv_data);
  }
}

int incomingByte; // for incoming serial data
// void loop()
// {
//   ir_receive();
// }
void loop() {
  // measure_distance(trigPin, echoPin);
  // Serial.println(ir_data, HEX);
  if(incomingByte == 1)
  {
    // Serial.println("Let's go");
    phase_0();
    delay(1000);
    // print_map();
  }
  if(incomingByte == 2)
  {
    // Serial.println("Let's go");
    ir_receive();
    delay(1000);
    // print_map();
  }

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
  // distance = measure_distance(trigPin, echoPin);
  // Serial.print("distance = ");
  // Serial.println(distance);

  if (Serial.available() > 0) {
    // read the incoming byte:
    String input = Serial.readStringUntil('\n');

    // Convert the string to an integer
    // Convert the string to an unsigned int (hexadecimal)
    
    incomingByte = strtoul(input.c_str(), NULL, 16);
    int data2 = input.toInt();
    Serial.print("Data2: ");
    Serial.println(data2);
    // incomingByte = Serial.read();// read the incoming data as string
    // // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte);
  }

  if(incomingByte == 255)
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(0,2);
    // print_map();

    delay(10000); // delay 
  }


  else if(incomingByte == 254)
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(2,4);
    // print_map();

    delay(10000); // delay 
  }

  else if(incomingByte == 253)
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(1,1);
    // print_map();

    delay(10000); // delay 
  }

  else if(incomingByte == 1) // Calibrate North
  {
    calibrated_North = readCompass(); // Read compass
    delay(50);// delay
  }

  else if(incomingByte == 2) // Calibrate East
  {
    calibrated_East = readCompass(); // Read compass
    delay(50);// delay 
  }

  else if(incomingByte == 3) // Calibrate South
  {
    calibrated_South = readCompass(); // Read compass
    delay(50);// delay 
  }

  else if(incomingByte == 4) // Calibrate West
  {
    calibrated_West = readCompass(); // Read compass
    delay(50);// delay 
  }

  else if(incomingByte == 5) // Reset map
  {
    grid_map[position_y][position_x] = 0;
    position_y = 2;
    position_x = 2;
    grid_map[position_y][position_x] = 1;
    delay(2000);// delay in 2000ms
  }

  else if(incomingByte == 6) // Reset map
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(4,2); // x y
    // print_map();

    delay(10000);// delay in 2000ms
  }
  // Serial.println("You donno");
  // find_current_location();
  // readCompass();
  // Serial.print("Amogus");
  // Serial.println(MAC_ID);
  // turn_until_degrees(90);
  // delay(1000);

  
    
  //turn_distance_sensor();
  // digitalWrite(9, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(125);
  // // digitalWrite(9, LOW); // turn the LED off by making the voltage LOW
}

// void find_current_location()
// {
//   for(int row = 0; row < numRows; row++)
//   {
//     for(int col = 0; col < numCols; col++)
//     {
//       if(grid_map[row][col] == 1)
//       {
//         position_y = row;
//         position_x = col;
//         Serial.print("X: ");
//         Serial.print(position_y);
//         Serial.print(" Y: ");
//         Serial.print(position_x);
//         Serial.println();
//       }

//     }
//   }
// }
//***************************************************************************