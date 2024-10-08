/* Author: Onurcan Cekem
 * Version: 0.7.6
 * Date: 04-10-2024
 * Used robot kit: https://docs.keyestudio.com/projects/KS0559/en/latest/Arduino/arduino.html
 * Description: This is part of the Bachelor's Thesis zwermgedrag (swarm behavior) where robots are designed for pattern formation.
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
#define IR_SEND_PIN A0 // Define IR-led as A0
#define servoPin A3  //servo Pin
#define ML_Ctrl 2  //define the direction control pins of group B motor
#define ML_PWM 5   //define the PWM control pins of group B motor
#define MR_Ctrl 4  //define the direction control pins of group A motor
#define MR_PWM 6   //define the PWM control pins of group A motor
#define LED_PIN 9 //define the pin of LED as pin 9
#define DECODE_NEC // Define which IR protocol is selected

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
uint32_t senddata = 0xFFFFFF; // Variable to store data to send
uint8_t received_ID = 0; // Variable to store received slave ID
unsigned long acknowledge_protocol = 0xDFFFF0; // Variable to determine protocol for own acknowledgement

// Bluetooth
#ifdef BLUETOOTH
SoftwareSerial bt(0,1); /* Rx,Tx for bluetooth */  
#endif BLUETOOTH

// Compass
QMC5883L compass; // Compass class
int calibrated_North = 135; // Variable to store direction North
int calibrated_East = 225;  // Variable to store direction East
int calibrated_South = 315; // Variable to store direction South
int calibrated_West = 45;   // Variable to store direction West

// Robot mapping
uint8_t grid_map[5][5] = {{0,0,0,0,0}, // grid_map[y][x]
                          {0,0,0,0,0},
                          {0,0,1,0,0},
                          {0,0,0,0,0},
                          {0,0,0,0,0}};
const int numRows = 5;
const int numCols = 5;
int position_y = 2; // Variable to store coordinate y
int position_x = 2; // Variable to store coordinate x 
uint8_t robot_ID = 0; // Variable to store own robot ID
unsigned int stored_id[5] = {0,0,0,0,0}; // Used for leader to keep track of slave ID's
unsigned int MAC_ID = 0; // Generate MAC_ID, an ID that's randomly generated in setup and won't change after


void setup() {
  //Serial Port begin
  Serial.begin (9600);

  // Servo code following Keyestudio's tutorial(unused)
  // servo_distance.attach(A3);  // attaches the servo on pin A3 to the servo object
  // servo_distance.write(110);  // Start distance sensor servo at exactly in the middle, because of offset
  // pinMode(servoPin, OUTPUT);  //set the pins of servo to output

  // IR receiver
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  irrecv.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Enable reading IR receiver

  // IR sender
  // irsend.begin(IR_SEND_PIN); // Call this function to send whenever I'm sending

  // HC-SR04 distance sensor  
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
  bt.begin(9600); /* Define baud rate for software serial communication */
  #endif BLUETOOTH
  Serial.println("Damn homie, we chilling");
  
  // Mapping stuff 
  randomSeed(analogRead(0)); // Random not random fix
  MAC_ID = random(1, 254); // Generate MAC_ID, an ID that won't change
  
  delay(1000);
  phase_2();
  // phase_0(); // Start program to determine leader
  formation_selection_5x5();
  print_map();
}

/* Function for distance sensor HC-SR04 to measure distance
 * param: trigger pin: TRIG D13
 * param: echo    pin: ECHO D12
 * Return: distance in centimeters
 */
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

/* function to control servo for distance sensor (unused)
 * param: myangle   1-180 Desired angle, aiming straight forward can range between 90-110 for each robot. 
 * KEEP IN MIND myangle can go out of bounds. If so, this could overheat the servo
 */
void servo_procedure(int myangle) {
  pulsewidth = myangle * 11 + 500;  //calculate the value of pulse width
  digitalWrite(servoPin,HIGH);
  delayMicroseconds(pulsewidth);   //The duration of high level is pulse width
  digitalWrite(servoPin,LOW);
  delay((20 - pulsewidth / 1000));  //the cycle is 20ms, the low level last for the rest of time
}

/* Function for distance sensor to turn and scan
 * It will automatically turn the servo to do a 180 degree spin
 */
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

/* Function set motor to drive forward
 * param: ms_time   drive for how long in ms
 */
void drive_forward(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH
  analogWrite(MR_PWM,55);// set the PWM control speed of group A motor to 55
  delay(ms_time);//delay in 2000ms)
}

/* Function set motor to drive backwards
 * param: ms_time   drive for how long in ms
 */
void drive_back(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);// set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,LOW);//set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

/* Function set motor to drive left
 * param: ms_time   drive for how long in ms
 */
void drive_left(int ms_time)
{
  digitalWrite(ML_Ctrl,LOW);//set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,200);//set the PWM control speed of group B motor to 200 
  digitalWrite(MR_Ctrl,HIGH);//set the direction control pins of group A motor to HIGH level
  analogWrite(MR_PWM,55);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

/* Function set motor to drive right
 * param: ms_time   drive for how long in ms
 */
void drive_right(int ms_time)
{
  digitalWrite(ML_Ctrl,HIGH);//set the direction control pins of group B motor to HIGH level
  analogWrite(ML_PWM,55);//set the PWM control speed of group B motor to 55 
  digitalWrite(MR_Ctrl,LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,200);//set the PWM control speed of group A motor to 200
  delay(ms_time);//delay in 2000ms)
}

/* Function set motor to stop driving
 * param: ms_time   stop for how long in ms
 */
void drive_stop(int ms_time)
{
  digitalWrite(ML_Ctrl, LOW);// set the direction control pins of group B motor to LOW level
  analogWrite(ML_PWM,0);//set the PWM control speed of group B motor to 0
  digitalWrite(MR_Ctrl, LOW);// set the direction control pins of group A motor to LOW level
  analogWrite(MR_PWM,0);//set the PWM control speed of group A motor to 0
  delay(ms_time);//delay in 2000ms)
}

/* Function to read bluetooth (unused, untested)
 * BT24 is the name of the module
 */
#ifdef BLUETOOTH
uint32_t bluetooth_read()
{
  if (Serial.available() > 0) //Check whether there is data in the serial port cache
  {
    data = Serial.read();  //Read data from the serial port cache
    Serial.print("DATA RECEIVED:");
    Serial.println(data);
    if (data == 'F') {
      digitalWrite(LED_PIN, HIGH); // Turn LED on
      Serial.println("led on");
    }
    if (data == 'B') 
    {
      digitalWrite(LED_PIN, LOW); // Turn LED off
      Serial.println("led off");
    }
   }
   return data;
}

/* Function to send data over bluetooth
 * Sends data through Serial monitor
 */
void bluetooth_send()
{
  Serial.println(data);
}
#endif BLUETOOTH

/* Function to check infrared receiver for messages
 * Print in Serial if something is received
 */
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

      // Send 0xD12345, D = acknowledge, 1&2 = MAC_ID 
      long senddata = (0xD << 20) + (MAC_ID << 12);
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

/* Function to send data with IR-led.
 * Sends data with NEC_MSB protocol
 * param: data    Data that is being sent (24 bit, 0x--345678)
 */
void ir_senddata(unsigned long data)
{
  irsend.begin(IR_SEND_PIN); // Enable sending IR.
  // IrReceiver.stop(); // Stop receiving IR, else sent IR echoes back to receiver. <-- Doesn't work
  irsend.sendNECMSB(data, 32);  // Send protocol NEC (same as remote)
  
  #ifdef DEBUG // Print out sent data in serial when debug is active
  // Serial.print("Data sent: ");
  // Serial.println(data, HEX); // Convert string input to hex
  // Serial.print("\t Reversed: ");
  // Serial.println(reverseBits(data), HEX);
  #endif
  delay(15); // Force delay, without sent signal echoes to receiver
  IrReceiver.resume(); // Resume receiving IR, else sent IR echoes back to receiver.
  return NULL;
}

/* Function to reverse from MSB to LSB. 
 * Used for IR protocol NEC.
 * param: num   a 32-bit value (input is generally IrReceiver.decodedIRData.decodedRawData)
 * Return: num but reversed 
 */
uint32_t reverseBits(uint32_t num) {
  uint32_t reversedNum = 0;

  for (int i = 0; i < 32; i++) {
    // Extract the rightmost bit of num and shift it to the leftmost position
    uint32_t bit = (num >> i) & 1;
    reversedNum |= (bit << (31 - i));
  }
  
  return reversedNum;
}

/* Function to Read the serial port for sending IR data
 * Return: data read from IR
 */
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

/* Function to read compass
 * Return: degrees (1-360)
 */
int readCompass()
{
  int heading = compass.readHeading();
  if(heading==0) {
    /* Still calibrating, so measure but don't print */
  } else {
    Serial.print(heading);
  }
  #ifdef DEBUG
  // if (heading > 270) Serial.println("N");
  // else if (heading > 180) Serial.println("E");
  // else if (heading > 90) Serial.println("S");
  // else Serial.println("W");
  #endif
  return heading;
}

/* Function to turn the car until desired degrees of direction
 * param: desired_direction   1-360
 */
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
  Serial.print("Achieved desired direction: ");
  Serial.println(desired_direction);
}

/* Function to drive 10 cm forwards
 */
void drive_10cm()
{
  int starting_distance, current_distance;
  starting_distance = measure_distance(trigPin, echoPin); // Measure starting distance
  current_distance = starting_distance;
  Serial.print("Starting 10cm. Distance: ");
  Serial.println(starting_distance);
  delay(500);
  while (starting_distance - current_distance < 10) // Continue until 10 cm was travelled
  {
    // Serial.println(current_distance);
    
    // Check if nothing is in front of car
    current_distance = measure_distance(trigPin, echoPin); // Measure distance
    if(current_distance <= 10) // If an object is below 10 cm threshold, interrupt
    {
      Serial.println("BREAK, BELOW 10 CM");
      break;
    }
    else drive_forward(5); // Else drive forward
  }
  drive_stop(1); // Stop wheels
  Serial.print("ended 10cm. Distance: ");
  Serial.println(current_distance);
}

/* Function to print the map grid.
 * Also updates position_y and y with wherever a 1 in the map is
 */
void print_map()
{
  for(int row = 0; row < numRows; row++)
  {
    for(int col = 0; col < numCols; col++)
    {
      // Find x and y of current location
      if(grid_map[row][col] == 1) // Find 1, which represents this robots coordinates: grid_map[y][x] == 1
      {
        position_y = row;
        position_x = col;
      }
      // if(row == 0 && col == 1) // Find 1, which represents this robots coordinates: grid_map[y][x] == 1
      // {
      //   grid_map[row][col] = 1;
      // }
      
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
 *  This is basically phase 3
 *  param desired_x: x-coordinates
 *  param desired_y: y-coordinates
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
    else break; // Desired x is found (position_y == desired_y) 
    Serial.println("Next pos");
    print_map();
    delay(500);
  }

  Serial.println("Destination arrived.");
}

/* A function for convenience to combine data byte-to-byte.
  param hex1: byte 1 (MSB)
  param hex2: byte 2
  param hex3: byte 3
  param hex4: byte 4
  param hex5: byte 5
  param hex6: byte 6 (LSB)
*/
uint32_t combine_irdata(int hex1, int hex2, int hex3, int hex4, int hex5, int hex6)
{
  uint32_t data;
  data |= (uint32_t) hex1 << 20;// 0x1----- MSB
  data |= (uint32_t) hex2 << 16;// 0x-1----
  data |= hex3 << 12;           // 0x--1---
  data |= hex4 << 8;            // 0x---1--
  data |= hex5 << 4;            // 0x----1-
  data |= hex6;                 // 0x-----1 LSB
  // Serial.print("data: ");
  // Serial.println(data, HEX);
  return data;
}

/* Function for start of program, checks if leader exists, else assign one.
 * Phase 0: 1. Determine leader, rest becomes slave.
 *          2. Slaves have to connect with leader
 */
void phase_0()
{
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) // 5 second timer
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
    Serial.println("I'm a leader ");
    Serial.print("LEADER broadcasting: ");
    Serial.print(senddata, HEX);
    Serial.print(" ");

    for(;;) // Infinite loop by leader until "OK is pressed"
    {
      ir_receive();
      delay(20);
      // Serial.println(senddata, HEX);
      // Serial.print("ir recv: ");
      // Serial.println(ir_recv_data, HEX);

      if ((ir_recv_data >> 20) == 0xD) // If a slave is found
      {
        received_ID = 0xFF; // Determine ID
        received_ID &= (ir_recv_data >> 8); // grab ID --XX--
        Serial.print("Slave found on ID (hex): ");
        Serial.println(received_ID, HEX);
        store_id(received_ID);

        senddata &= (received_ID << 8); // --XX--
        senddata &= 0xFFFFF0; // Protocol for acknowledge, it has to end with F0
        delay(1000);
        startTime = millis(); // refresh timer
        Serial.print("sending acknowledge:");
        Serial.println(senddata, HEX);
        while (millis() - startTime < 3000) // broadcast for 3 seconds
        { 
          ir_senddata(senddata); // Broadcast acknowledge
          delay(100);
        }
        ir_recv_data = 0;
        Serial.print("Back to broadcasting ");
      }

      else if (ir_recv_data == 0xFF02FD) // "OK" from remote
      {
        Serial.print("OK message received, continue");
        break; // Exit out of program and go to next phase
      }

      else
      {
        senddata = 0xDFFFFF; // Set to broadcast protocol
        ir_senddata(senddata); // Broadcast data
      }
      delay(200);
    } // End of for-loop
    return NULL; // Break out of function
  } // End of leader

  // leader is found, this robot is a slave and starts handshaking
  Serial.println("I'm a slave ");
  senddata = 0xD00000; // Determine protocol
  senddata |= (uint32_t)MAC_ID << 12; // -XX--- 
  acknowledge_protocol &= (MAC_ID << 12); // Acknowledge is 0xDFFFF0, turn into 0xD23F00 (1 and 2 are MAC_ID)
  Serial.print("Acknowledge protocol: ");
  Serial.println(acknowledge_protocol, HEX);
  delay(1000);

  for(;;) // Infinite loop as slave
  {

    ir_receive(); // Read for acknowledge from leader

    if ((ir_recv_data == acknowledge_protocol) || (ir_recv_data == acknowledge_protocol << 12)) // Acknowledge from leader
    {
      Serial.print("Acknowledged (HEX) ");
      Serial.println(MAC_ID, HEX);
      Serial.print("ID (DEC) ");
      Serial.println(MAC_ID, DEC);
      break; // Handshaking complete, exit
    }

    delay(20);
    // Serial.print("SLAVE Sending data: ");
    // Serial.println(senddata, HEX);
    ir_senddata(senddata); // first step of handshaking Send slave ID 
    delay(200);

  } // End of for-loop
  return NULL;

  // The code below is unused. It's purpose was to test the code above.
  ir_receive();
  ir_recv_data = reverseBits(IrReceiver.decodedIRData.decodedRawData); // Decode received data and reverse it so the remote works
  // ir_recv_data = 0xE12200;
  //ir_data = reverseBits(ir_data); // Decode received data and reverse it so the remote works
  Serial.print("IR data received: ");
  Serial.print(ir_recv_data, HEX); // Print "old" raw data in HEX
  Serial.print(' '); // Print "old" raw data
  unsigned long ir_temp = ir_recv_data >> 20; // Only grab first 4 bits
  senddata = 0xD00000;
  int destination_x = (ir_recv_data >> 12) & 0xF; // --X---Grab the third digit
  int destination_y = (ir_recv_data >> 8 ) & 0xF; // ---X--Grab the fourth digit
  // ir_data[0] = 0xD;
  // ir_data[1] = MAC_ID >> 4;
  // ir_data[2] = MAC_ID;
  // ir_data[3] = 0;
  // ir_data[4] = 0;
  // ir_data[5] = 0;
  senddata |= (uint32_t)MAC_ID << 12; // -XX---
  Serial.print("MAC_ID: ");
  Serial.print(MAC_ID, HEX);
  // Serial.print("\t Destination X: ");
  // Serial.print(destination_x);
  // Serial.print("\t Destination Y: ");
  // Serial.println(destination_y);
  Serial.print(" Senddata: "); // Senddata is D12345, 1&2 = MAC_ID
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
  if ((ir_recv_data >> 20)  == 0xD) // D----- handshaking protocol
  {                 // D12345, 1 = MAC_ID, 2 = MAC_ID, 3 = empty, 4 = empty, 5 = empty
    
  }
  if ((ir_temp & 0xF) == 0xE) // Check first 4 bits, if it's 15 (E) enable map shennanigans protocol
  {                 // E----- map protocol
                    // E12345. 1 = ID, 2 = X coords, 3 = Y coords, 4 = empty, 5 = empty
  

  }
  else
  {
    // Switch case
    // Serial.println(ir_recv_data);
  }
}

/* Function to separate phase 1
 * Phase 1: 1. Leader is sending IR, which slaves are trying to receive
 *          2. Slaves drive around until IR is received.
 *          3. Slaves try to drive towards leader.
 */
void phase_1()
{

}

/* Function to separate phase 2
 * Phase 2: 1. Slaves determine their offset with leader and leader receives coordinates from slaves
 *          2. Leader determines shortest route for each slave
 *          3. Leader sends slaves destination coordinates.
 */
void phase_2()
{
  // Leader receives coördinates from slaves and calculates shortest route
  // if(leader == 1)
  // {
    for(;;) // For-loop until coordinates have been shared and a formation has been selected.
    {
      break;
      // Receive coordinates
      ir_receive();
      ir_recv_data = 0xE1024F;
      if((ir_recv_data >> 20) == 0xE) // If it's the receive coordinate command, E12345. 12 = ID, 34 = XY
      {
        Serial.print("ID: ");
        Serial.print(ir_recv_data >> 12 & 0xFF);
        Serial.print(" x: ");
        Serial.print(ir_recv_data >> 8 & 0xF);
        Serial.print("\t y: ");
        Serial.println(ir_recv_data >> 4 & 0xF);
        grid_map[ir_recv_data >> 4 & 0xF][ir_recv_data >> 8 & 0xF] = ir_recv_data >> 12 & 0xFF; //grid_map[y][x] = MAC_ID | Remember coordinates
        // print_map();
      }      

      else if(ir_recv_data == 0xFF6897) // Remote 1,
      {
        Serial.print("1");
        break;
      }      
      else if(ir_recv_data == 0xFF9867) // Remote 2,
      {
        Serial.print("2");
        break;
      }      
      else if(ir_recv_data == 0xFFB04F) // Remote 3,
      {
        Serial.print("3");
        break;
      }
      delay(1000);
    } // End of for-loop on receiving coördinates from slaves

      // Determine end positions based on selected form
      // Calculate shortest route for each slave 
      // Leader has to decide which slave receives which destination
      
  // }

  // Slave calculates orientation with leader
  if(leader == 0)
  {
  // Determine all orientations
    int calibrated_NorthEast = (calibrated_North + calibrated_East) / 2;
    int calibrated_SouthEast = (calibrated_South + calibrated_East) / 2;
    int calibrated_SouthWest = (calibrated_South + calibrated_West) / 2;
    int calibrated_NorthWest = (calibrated_North + calibrated_West) / 2;
    int directions[8] = {calibrated_North, calibrated_NorthEast, // Store all orientations
                        calibrated_East, calibrated_SouthEast, 
                        calibrated_South, calibrated_SouthWest, 
                        calibrated_West, calibrated_NorthWest};
    int minDifference = 360; // Initialize to a large value
    int difference = 0;
    int index = 0;
    
    int current_direction = 130;
    // int current_direction = readCompass(); 
    for(int i = 0; i < 8; i++) // For-loop to determine closest direction amongst the 8 orientations
    {
      difference = abs(current_direction - directions[i]);
      if (difference > 180) {
        difference = 360 - difference; // Handle wrap-around case
      }

      if (difference < minDifference) {
        minDifference = difference;
        index = i;
      }

    }
    
    // int steps = (measure_distance(trigPin, echoPin)+5) / 10; // Round up to nearest 10. 15 would round up to 20.
    int steps = 2;
    // Determine what happens depending on oriëntation
    // Assuming that the leader is located at 2,2. Determine the coordinates of slave with leader.
    switch(index)
    {
      case 0: // North, y+1
        position_y = position_y + steps;
        break;
      case 1: // North-East, x-1, y+1
        position_y = position_y + steps;
        position_x = position_x - steps;
        break;
      case 2: // East, x-1
        position_x = position_x - steps;
        break;
      case 3: // South-East, x-1, y-1
        position_y = position_y - steps;
        position_x = position_x - steps;
        break;
      case 4: // South, y-1
        position_y = position_y - steps;
        break;
      case 5: // South-West, x+1, y-1
        position_x = position_x + steps;
        position_y = position_y - steps;
        break;
      case 6: // West, x+1
        position_x = position_x + steps;
        break;
      case 7: // North-West, x+1, y+1
        position_x = position_x + steps;
        position_y = position_y + steps;
        break;
    }

    Serial.print("New coordinates, X: ");
    Serial.print(position_x);
    Serial.print("\t Y: ");
    Serial.println(position_y);

    Serial.print("Closest direction is: ");
    Serial.print(directions[index]);
    Serial.print(" at index ");
    Serial.println(index);

    // Send coordinates to leader with E12345, 12 = ID, 34 = XY
    
    senddata = combine_irdata(0xE, MAC_ID>>4&0xF, MAC_ID&0xF, position_x, position_y, 0);
    Serial.print("MAC_ID: ");
    Serial.print(MAC_ID, HEX);
    Serial.print("\t Senddata: ");
    Serial.println(senddata, HEX);
    // ir_senddata(senddata);

    // Wait for end position
  } // End of slave
 
  return NULL;

  // int difference = abs(current_direction-desired_direction);
  // int left, right; // counterclockwise
  // Serial.println("Difference: ");
  // Serial.println(difference);
  // while(difference >= 4) // Keep in function until direction is achieved
  // {
  //   // Calculate difference
  //   left = (current_direction - desired_direction + 360) % 360; // counterclockwise
  //   right = (desired_direction - current_direction + 360) % 360; // clockwise

  // // Serial.print("Left: ");
  // // Serial.print(left);
  // // Serial.print("\t right: ");
  // // Serial.println(right);

  //   // Determine shortest turn direction
  //   if (left <= right) // Go left
  //   {
  //     drive_left(5); // counterclockwise
  //   }
  //   else // Go right
  //   {
  //     drive_right(5); // clockwise
  //   }

  //   // Serial.print("current direction in while: ");
  //   // Serial.println(current_direction);
  //   difference = abs(current_direction-desired_direction);
  // }

}

/* Function to separate phase 3
 * Phase 3: Slaves follow the shortest route algorithm.
 * 1. Receive coordinates
 * 2. call goto_coordinates() function
 */
void phase_3()
{

}

// 
/* Function to store ID
 * Find the first possible empty ID and store it
 * param: received_ID   1-5, indicates amount of slaves that can be part of the swarm. Amount is based on stored_id[] size.
 */
void store_id(int received_ID)
{
  Serial.print(" Current list of ID's: ");
  for(int i = 0; i < 5; i++)
  {
    if(stored_id[i] == received_ID) // If duplicate, don't store it
    {
      Serial.print(stored_id[i]);
      break;
    } 
    else if(stored_id[i] == 0) // If new, store it
    {
      stored_id[i] = received_ID;
      Serial.print(stored_id[i]);
      break;
    }
    else // Else print everything
    {
    Serial.print(stored_id[i]);
    Serial.print(" ");
    }
  }
  Serial.println(" ");
}

/* Function to calculate absolute distance
 * Used for routeplanning by the leader to determine which robots has the shortest route
 * param x1: x start position
 * param y1: y start position
 * param x2: x end position 
 * param y2: y end position 
 */
int manhattan_distance(int x1, int y1, int x2, int y2)
{
  return abs(x1 - x2) + abs(y1 - y2);
}
    

/* Function to create formations (line, v-form, square/triangle)
 * Retrieve data from ir and select a form
 * 
 */
void formation_selection_5x5()
{
  uint8_t robot2_start[2] = {1,2}, robot3_start[2] = {3,1};

  // Debug
  grid_map[1][2] = 2;
  grid_map[3][1] = 3;
  print_map();
  grid_map[1][2] = 0;
  grid_map[3][1] = 0;

  Serial.println("Above is map of current pos");
  uint8_t robot2_end[2], robot3_end[2]; // [y,x]
  // Grab starting positions
  // robot2_start;
  // robot3_start;
  if(0) // line
  {
  /*[ 0, 0, 0, 0, 0 ]
    [ 0, 0, 0, 0, 0 ]
    [ 0, 2, 1, 3, 0 ] 
    [ 0, 0, 0, 0, 0 ]
    [ 0, 0, 0, 0, 0 ]*/
    robot2_end[0] = 2, robot2_end[1] = 1; // Remember robot2 end position
    robot3_end[0] = 2, robot3_end[1] = 3; // Remember robot2 end position
    grid_map[2][1] = 2;
    grid_map[2][3] = 3;
    
  }
  else if(2) // v-form
  {
  /*[ 0, 0, 0, 0, 0 ]
    [ 0, 0, 0, 0, 0 ]
    [ 0, 0, 1, 0, 0 ] 
    [ 0, 2, 0, 3, 0 ]
    [ 0, 0, 0, 0, 0 ]*/
    robot2_end[0] = 3, robot2_end[1] = 1;
    robot3_end[0] = 3, robot3_end[1] = 3;
    grid_map[3][1] = 2;
    grid_map[3][3] = 3;
  }
  else if(3) // square/triangle
  {
  /*[ 0, 0, 0, 0, 0 ]
    [ 0, 0, 0, 0, 0 ]
    [ 0, 0, 1, 0, 3 ] 
    [ 0, 0, 0, 0, 0 ]
    [ 0, 0, 2, 0, 0 ]*/
    robot2_end[0] = 4, robot2_end[1] = 2;
    robot3_end[0] = 2, robot3_end[1] = 4;
    grid_map[4][2] = 2;
    grid_map[2][4] = 3;

  }
  else
  {
    Serial.println("No valid location was given in formation_selection");
    return NULL;
  }
  // End of formation selection
  
  // Start of calculating manhattan distance which is shortest route for each robot
  uint8_t distance_robot2_to_2_end = manhattan_distance(robot2_start[0], robot2_start[1], robot2_end[0], robot2_end[1]);
  uint8_t distance_robot2_to_3_end = manhattan_distance(robot2_start[0], robot2_start[1], robot3_end[0], robot3_end[1]);
  uint8_t distance_robot3_to_2_end = manhattan_distance(robot3_start[0], robot3_start[1], robot2_end[0], robot2_end[1]);
  uint8_t distance_robot3_to_3_end = manhattan_distance(robot3_start[0], robot3_start[1], robot3_end[0], robot3_end[1]);

  // Debug print
  // Serial.print("Current distance 2:2: ");
  // Serial.println(distance_robot2_to_2_end);
  // Serial.print("Current distance 3:3: ");
  // Serial.println(distance_robot3_to_3_end);

  // Serial.print("Alternative Distance 2:3: ");
  // Serial.println(distance_robot2_to_3_end);
  // Serial.print("Alternative Distance 3:2: ");
  // Serial.println(distance_robot3_to_2_end);

  // If distance from 2 to 2 is shorter than 2 to 3. AND if distance from 3 to 3 is shorter than 3 to 2.
  // Basically, check if it's worth to swap routes.
  if (distance_robot2_to_3_end <= distance_robot2_to_2_end &&
      distance_robot3_to_2_end <= distance_robot3_to_3_end) {
    
    // if-condition to check if it's worth swapping
    // If both routes have the same distance, do nothing
    if (distance_robot2_to_3_end == distance_robot2_to_2_end &&
        distance_robot3_to_2_end == distance_robot3_to_3_end){
      // No swap needed
      Serial.println("No swap needed");
    }

    else // Swap needed
    {
      Serial.println("Swap needed");
      uint8_t swap_positions[2]; // Storage for swapping both coordinates
      swap_positions[0] = robot2_end[0];
      swap_positions[1] = robot2_end[1];
      robot2_end[0] = robot3_end[0];
      robot2_end[1] = robot3_end[1];
      robot3_end[0] = swap_positions[0];
      robot3_end[1] = swap_positions[1];
      
      // Debug print
      Serial.print("robot2_end: ");
      Serial.print(robot2_end[0]); 
      Serial.println(robot2_end[1]);
      
      // Update map
      grid_map[robot2_end[0]][robot2_end[1]] = 2; 
      grid_map[robot3_end[0]][robot3_end[1]] = 3;
    }

  } 
  else // No swap needed
  { 
    Serial.println("No swap needed");
    // // No swap needed
  }

}

int incomingByte; // for incoming serial data
void loop() {
  // measure_distance(trigPin, echoPin);
  // Serial.println(ir_data, HEX);
  if(incomingByte == 1) // Start phase 0
  {
    // Serial.println("Let's go");
    phase_0();
    delay(1000);
    // print_map();
  }

  if(incomingByte == 2) // Enable IR Receiver
  {
    // Serial.println("Let's go");
    ir_receive();
    Serial.println("");
    delay(1000);
    // print_map();
  }
  
  if(incomingByte == 3) // Store ID test
  {
    // Serial.println("Let's go");
    store_id(0);
    delay(5000);
    // print_map();
  }

  if(incomingByte == 4) // IR-led send test
  {
    senddata = 0xF20FFF; // Random hex where ID is 32 (20 in hex)
    Serial.print("Sending data: ");
    Serial.println(senddata, HEX);
    
    
    while(incomingByte == 4) // IR-led test
    {
      ir_senddata(senddata);
      if (Serial.available() > 0) { // Check serial if something is received
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
    }
    delay(25);
  }

  // Distance sensor
  // distance = measure_distance(trigPin, echoPin);
  // Serial.print("distance = ");
  // Serial.println(distance);

  if (Serial.available() > 0) { // Read serial
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

  if(incomingByte == 255) // goto 0,2
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(0,2);
    // print_map();

    delay(10000); // delay 
  }


  else if(incomingByte == 254) // goto 2,4
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(2,4);
    // print_map();

    delay(10000); // delay 
  }

  else if(incomingByte == 253) // goto 1,1
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

  else if(incomingByte == 6) // Reset map, goto 4,2
  {
    // Serial.println("Let's go");
    print_map();
    delay(1000);
    goto_coordinates(4,2); // x y
    // print_map();

    delay(10000);// delay in 10000ms
  }
}
/*
 * The code beyond here is old, outdated or junk
 */
  // Serial.println("You donno");
  // find_current_location();
  // readCompass();
  // Serial.print("Yeet");
  // Serial.println(MAC_ID);
  // turn_until_degrees(90);
  // delay(1000);

  //turn_distance_sensor();
  // digitalWrite(9, HIGH); // turn the LED on (HIGH is the voltage level)
  // delay(125);
  // // digitalWrite(9, LOW); // turn the LED off by making the voltage LOW

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
  // Read Bluetooth
  // readSerial();
  // ir_receive();
  // data = bluetooth_read(); 
  // bluetooth_send();
  //irsend.sendNECMSB(reverseBits(data), 32);  // Send reversed data
  // irrecv.enableIRIn();
  
  // Bluetooth testing
  // if (bt.available())  /* If data is available on serial port */
  // {
  //   // Send the data to the Bluetooth serial
  //   bt.write(data);
  //   // Optionally, print the data to the Serial Monitor
  //   // Serial.print("Sent via Bluetooth: ");
  //   // Serial.println(data);
  //   Serial.write(bt.read()); /* Print character received on to the serial monitor */
  // }