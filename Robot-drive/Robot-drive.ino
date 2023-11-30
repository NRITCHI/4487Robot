// 
// MME 4487 Final Project Drive System
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Blake Nielsen, Nathan Ritchie, Peter Guatto, Hamza Faish
//  Date:     2023 11 23 
//

// #define SERIAL_STUDIO                                 // print formatted string, that can be captured and parsed by Serial-Studio
// #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
#define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Adafruit_TCS34725.h"

// Function declarations
void doHeartbeat();
void setMotor(int dir, int pwm, int in1, int in2);
void ARDUINO_ISR_ATTR encoderISR(void* arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
long degreesToDutyCycle(int deg);

// Control data packet structure
struct ControlDataPacket {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  int speed;                                          // motor speed
  int sorting;                                        // 1 to turn on sorting, 0 off
  int bucket;                                         // 1 to dump bucket
  unsigned long time;                                 // time packet sent
  int turn;                                           // motor direction (1 = right turn, 0 = straight, -1 = left turn)
};

// Drive data packet structure
struct DriveDataPacket {
  unsigned long time;                                 // time packet sent
  bool detected;                                      // desired colour detection
};

// Encoder structure
struct Encoder {
  const int chanA;                                    // GPIO pin for encoder channel A
  const int chanB;                                    // GPIO pin for encoder channel B
  long pos;                                           // current encoder position
};

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
///const int cStatusLED = 27;                            // GPIO pin of communication status LED
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cNumMotors = 2;                             // Number of DC motors
const int cIN1Pin[] = {17, 19};                       // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1};                        // PWM channel(s) for INT1
const int c2IN2Pin[] = {16, 18};                      // GPIO pin(s) for INT2
const int cIN2Chan[] = {2, 3};                        // PWM channel(s) for INT2
const int cPWMRes = 8;                                // bit resolution for PWM
const int cMinPWM = 0;                                // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;              // PWM value for maximum speed
const int cPWMFreq = 20000;                           // frequency of PWM signal
const int cCountsRev = 1096;                          // encoder pulses per motor revolution
const int cMaxSpeedInCounts = 1600;                   // maximum encoder counts/sec
const int cMaxChange = 14;                            // maximum increment in counts/cycle
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const float kp = 1.5;                                 // proportional gain for PID
const float ki = 0.2;                                 // integral gain for PID
const float kd = 0.8;                                 // derivative gain for PID
const int cTCSLED = 23;                               // GPIO pin for LED on TCS34725
const int diskIN1 = 13;
const int diskIN2 = 27;
//const int fanIN1 = 4;
//const int fanIN2 = 13;
const int ci_ServoPin1 = 4;                          // GPIO pin for servo 1
const int ci_Bucket = 12;                             // GPIO pin for servo 2
const int ci_ServoChannel1 = 4;                       // PWM channel for servo motor
const int ci_BucketChannel = 5;                       // PWM channel for servo motor 2


// Variables
unsigned long lastHeartbeat = 0;                      // time of last heartbeat state change
unsigned long lastTime = 0;                           // last time of motor control was updated
unsigned int commsLossCount = 0;                      // number of sequential sent packets have dropped
Encoder encoder[] = {{25, 26, 0},                     // encoder 0 on GPIO 25 and 26, 0 position
                     {32, 33, 0},};                    // encoder 1 on GPIO 32 and 33, 0 position
long target[] = {0, 0};                               // target encoder count for motor
long lastEncoder[] = {0, 0};                          // encoder count at last control cycle
float targetF[] = {0.0, 0.0};                         // target for motor as float
ControlDataPacket inData;                             // control data packet from controller
DriveDataPacket driveData;                            // data packet to send controller

int toggle = 1;
int curCheck;
int dumpDelay = millis();
int dumpWait;
int position = 0;
int senseDelay;
int pressed = 1;
int dumping = 0;

int test = 0;

// REPLACE WITH MAC ADDRESS OF YOUR CONTROLLER ESP32
uint8_t receiverMacAddress[] = {0xB0,0xA7,0x32,0xF2,0x92,0xD4};  // MAC address of controller 00:01:02:03:04:05 
esp_now_peer_info_t peerInfo = {};                    // ESP-NOW peer information

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor
  WiFi.mode(WIFI_STA);                                // Use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  WiFi.disconnect();                                  // disconnect from network
  
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat
  //pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO for control of LED on TCS34725

  ledcAttachPin(ci_ServoPin1, ci_ServoChannel1);        // assign servo pin to servo channel
  ledcAttachPin(ci_Bucket, ci_BucketChannel);        // assign servo pin to servo channel
  ledcSetup(ci_ServoChannel1, 50, 16);                // setup for channel for 50 Hz, 16-bit resolution
  ledcSetup(ci_BucketChannel, 50, 16);                // setup for channel for 50 Hz, 16-bit resolution

  // setup motors with encoders
  for (int k = 0; k < cNumMotors; k++) {
    ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);           // attach INT1 GPIO to PWM channel
    ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);        // configure PWM channel frequency and resolution
    ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);          // attach INT2 GPIO to PWM channel
    ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);        // configure PWM channel frequency and resolution
    pinMode(encoder[k].chanA, INPUT);                 // configure GPIO for encoder channel A input
    pinMode(encoder[k].chanB, INPUT);                 // configure GPIO for encoder channel B input
    // configure encoder to trigger interrupt with each rising edge on channel A
    attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

  pinMode(diskIN1, OUTPUT);
  pinMode(diskIN2, OUTPUT); 
  ledcWrite(ci_BucketChannel, degreesToDutyCycle(130));
  
  

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) 
  {
    Serial.printf("Error initializing ESP-NOW\n");
    return;
  }
  else
  {
    Serial.printf("Successfully initialized ESP-NOW\n");
  }
  esp_now_register_recv_cb(onDataRecv);               // register callback function for received data
  esp_now_register_send_cb(onDataSent);               // register callback function for data transmission
  
  // Set controller info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel
  peerInfo.encrypt = false;                           // no encryption of data
  
  // Add controller as ESP-NOW peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.printf("Failed to add peer\n");
    return;
  }
  else
  {
    Serial.printf("Successfully added peer %x:%x:%x:%x:%x:%x\n to list", receiverMacAddress[0], receiverMacAddress[1], 
                                                                         receiverMacAddress[2], receiverMacAddress[3], 
                                                                         receiverMacAddress[4], receiverMacAddress[5]);
  }

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
    digitalWrite(cTCSLED, 1);                         // turn on onboard LED 
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }
}


void loop() {
  float deltaT = 0;                                   // time interval
  long pos[] = {0, 0};                                // current motor positions
  float velEncoder[] = {0, 0};                        // motor velocity in counts/sec
  float velMotor[] = {0, 0};                          // motor shaft velocity in rpm
  float posChange[] = {0, 0};                         // change in position for set speed
  long e[] = {0, 0};                                  // position error
  float ePrev[] = {0, 0};                             // previous position error
  float dedt[] = {0, 0};                              // rate of change of position error (de/dt)
  float eIntegral[] = {0, 0};                         // integral of error 
  float u[] = {0, 0};                                 // PID control signal
  int pwm[] = {0, 0};                                 // motor speed(s), represented in bit resolution
  int dir[] = {1, 1};                                 // direction that motor should turn
  uint16_t r, g, b, c;                                // RGBC values from TCS34725

  //Serial.println("looped");
  // if too many sequential packets have dropped, assume loss of controller, restart as safety measure
  // if (commsLossCount > cMaxDroppedPackets) {
    //delay(1000);                                      // okay to block here as nothing else should be happening
    //ESP.restart();                                    // restart ESP32
  //}

  // store encoder positions to avoid conflicts with ISR updates
  noInterrupts();                                     // disable interrupts temporarily while reading
  for (int k = 0; k < cNumMotors; k++) {
    pos[k] = encoder[k].pos;                          // read and store current motor position
  }
  interrupts();                                       // turn interrupts back on

  //Object colour detection
   if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
    //Serial.println("colour");
   }
  
    if (r >= 10 && g >= 40 && b >= 10 && c >= 20){
      driveData.detected = true;
    } else{
      driveData.detected = false;
    }
  
  
  Serial.print("r ");
  Serial.print(r);
  Serial.print(" b ");
  Serial.print(b);
  Serial.print(" g ");
  Serial.print(g);
  Serial.print(" c ");
  Serial.println(c);

  //Serial.println(position);
  

  // changes servo position depending on colour
  
  if (position == 0) {                                          // if in position 0 (at home position, looking for colour)
    if (r <= 13 && b <= 10 && g <= 10 && c <= 25) {                    // check if black
      //Serial.println("black");
      senseDelay = millis();
    }else{
      if ((millis() - senseDelay) > 300){
        //if (r <= 30 && r >= 25 && b <= 25 && b >= 20  && g <= 38 && g >= 29 && c <= 92 && c >= 75) {            // check if green
        //if (r <= 32 && r >= 12 && b <= 25 && b >= 8  && g <= 37 && g >= 12 && c <= 90 && c >= 33) {             // check green upright
        //if ((g - b > 3) && (r/g < 1.5) && (g/b > 1.25)){
        if (r >= 12.52 && r <= 24.56 && b >= 6.95 && b <= 19.98 && g >= 11.10 && g <= 28.59 && c >= 31.44 && c <= 72.25) {
          curCheck = millis();                                      // update timer
          Serial.println("green");                                   //change position
          position = 1;                                             // change to position 1
        //} else if (r <= 45 && r >= 14 && b <= 42 && b >= 11  && g <= 52 && g >= 18 && c <= 137 && c >= 44) {             // check green upside down
          //Serial.println("green upside down");
          //curCheck = millis();                                      // update timer
          //position = 1;                                             // change to position 1
        }else {
          curCheck = millis();
          Serial.println("reject");                                   //change position
          position = 2;                                               // check if white
        }
      }
    }
  }

    
  
  if (position == 0) {
    ledcWrite(ci_ServoChannel1, degreesToDutyCycle(95));               // keep at home position if did not detect colour
  } else if (position == 1) {                                          
    //if ((millis() - curCheck) > 1000) {                             // if position 1 and after delay, change position
      ledcWrite(ci_ServoChannel1, degreesToDutyCycle(30));
      position = 4;                                                  // change to position 4 (waiting position)
      curCheck = millis();                                                 // reset
    //}
  } else if (position == 2) {
    //if ((millis() - curCheck) > 400) {                                 // if position 2 and after delay, change position
      //if(r <= 3 && b <= 5 && g <= 5 && c <= 1)
      ledcWrite(ci_ServoChannel1, degreesToDutyCycle(180));                 // change to proper position
      position = 4;                                                       // change to position 4 (waiting position)
      curCheck = millis();                                                      // reset
   // }
    
  }

  if (position == 4) {                                                    // if position 4 (item discharged)
    if ((millis() - curCheck) > 130 && (millis() - curCheck) < 400) {    // wait in position for 500 millisecond      
      ledcWrite(ci_ServoChannel1, degreesToDutyCycle(95));                                    // go to home position
      //position = 0;                                                        // return back to 0 (looking for colour)

    } else if ((millis() - curCheck) >= 500) {                             // after another 500 milliseconds
      position = 0;                                                        // return back to 0 (looking for colour)
      curCheck = millis();                                                 // reset
      Serial.println("We are here");

    }
  }
  
  // bucket code
  
  if (inData.bucket == 1 && toggle == 1 && dumping == 0) {  // if is not currently dumping and sorting system is running
    dumpDelay = millis();                                     // start timer for safety
    dumping = 1;                                             // toggle start of dump
  } else if (inData.bucket == 1 && toggle == 0 && dumping == 0) {    // if sorting isnt running
    dumping = 1;
  }                                                             // toggle start of dump
 
  if (dumping == 1 && (millis() - dumpDelay) > 1000){             // if is starting, wait half a second for anything to finish
    ledcWrite(ci_BucketChannel, degreesToDutyCycle(0));         // rotate bucket
    dumpWait = millis();                                         // start timer
    dumping = 2;                                                 // continue dump process
  } else if (dumping == 2 && (millis() - dumpWait) > 1000) {     // after time and is in second phase
    dumping = 0;                                                  // dump back to normal
    ledcWrite(ci_BucketChannel, degreesToDutyCycle(130));           // return to original position
  }
  
  /*
  if (test == 0) {
    ledcWrite(ci_BucketChannel, degreesToDutyCycle(95));
    test = 1;
  } else {
    ledcWrite(ci_BucketChannel, degreesToDutyCycle(95));
    test = 0;
  }
  */
  // control code
  unsigned long curTime = micros();                   // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    deltaT = ((float) (curTime - lastTime)) / 1.0e6;  // compute actual time interval in seconds
    lastTime = curTime;                               // update start time for next control cycle
    driveData.time = curTime;                         // update transmission time


    // sorting drive logic

    if (inData.sorting == 1 && pressed == 0) {           // if button pressed and is not currently pressed
      toggle = toggle * -1;                                // swap toggle state
      pressed = 1;                                         // button state as pressed
    } else if (inData.sorting == 0) {                    // if is not pressed
      pressed = 0;                                         // set state as not pressed
    }
 

     if (toggle == 1) {
      analogWrite(diskIN1, 78);
      analogWrite(diskIN2, 0);                    // if on go, if off stop the sorting system
    } else if (toggle == -1) {
      analogWrite(diskIN1, 0);
      analogWrite(diskIN2, 78);
    }

    for (int k = 0; k < cNumMotors; k++) {
      velEncoder[k] = ((float) pos[k] - (float) lastEncoder[k]) / deltaT; // calculate velocity in counts/sec
      lastEncoder[k] = pos[k];                        // store encoder count for next control cycle
      velMotor[k] = velEncoder[k] / cCountsRev * 60;  // calculate motor shaft velocity in rpm

      // update target for set direction

      if (k == 0 || k == 1) {
        if (inData.dir == 1 || inData.dir == -1){             // if forward or reverse button pressed
          posChange[k] = (float) (inData.dir * inData.speed); // update with controller speed value

          if (inData.turn == -1){                             // if left turn button pressed
            posChange[0] = 0;                                   // turn off right motor to turn left
          }
          else if (inData.turn == 1){                         // if right turn button pressed
            posChange[1] = 0;                                  // turn off left motor to turn right
          }
        
        } else{                                               // if forward or reverse button not pressed
          if (inData.turn == -1){                             // if left turn button pressed
            posChange[0] = (float) (-1 * inData.speed); // update with controller speed value
            posChange[1] = (float) (1 * inData.speed); // update with controller speed value
          }
          else if (inData.turn == 1){                         // if right turn button pressed
            posChange[0] = (float) (1 * inData.speed); // update with controller speed value
            posChange[1] = (float) (-1 * inData.speed); // update with controller speed value
          }
        }
          targetF[k] = targetF[k] + posChange[k];         // set new target position
      
        if (k == 0) {                                   // assume differential drive
          target[k] = (long) targetF[k];                // motor 1 spins one way
        }
        else {
          target[k] = (long) -targetF[k];               // motor 2 spins in opposite direction
        }
      }
      
      

      // use PID to calculate control signal to motor
      e[k] = target[k] - pos[k];                      // position error
      dedt[k] = ((float) e[k]- ePrev[k]) / deltaT;    // derivative of error
      eIntegral[k] = eIntegral[k] + e[k] * deltaT;    // integral of error (finite difference)
      u[k] = kp * e[k] + kd * dedt[k] + ki * eIntegral[k]; // compute PID-based control signal
      ePrev[k] = e[k];                                // store error for next control cycle
  
      // set direction based on computed control signal
      dir[k] = 1;                                     // default to forward directon
      if (u[k] < 0) {                                 // if control signal is negative
        dir[k] = -1;                                  // set direction to reverse
      }

      // set speed based on computed control signal
      u[k] = fabs(u[k]);                              // get magnitude of control signal
      if (u[k] > cMaxSpeedInCounts) {                 // if control signal will saturate motor
        u[k] = cMaxSpeedInCounts;                     // impose upper limit
      }
      pwm[k] = map(u[k], 0, cMaxSpeedInCounts, cMinPWM, cMaxPWM); // convert control signal to pwm
      
      if (commsLossCount < cMaxDroppedPackets / 4) {
        setMotor(dir[k], pwm[k], cIN1Chan[k], cIN2Chan[k]); // update motor speed and direction
      }
      else {
        setMotor(0, 0, cIN1Chan[k], cIN2Chan[k]);     // stop motor
      }
#ifdef SERIAL_STUDIO
      if (k == 0) {
        printf("/*");                                 // start of sequence for Serial Studio parsing
      }
      printf("%d,%d,%d,%0.4f", target[k], pos[k], e[k], velMotor[k]);  // target, actual, error, velocity
      if (k < cNumMotors - 1) {
        printf(",");                                  // data separator for Serial Studio parsing
      }
      if (k == cNumMotors -1) {
        printf("*/\r\n");                             // end of sequence for Serial Studio parsing
      }
#endif
    }
    // send data from drive to controller
    esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *) &driveData, sizeof(driveData)); 
    if (result == ESP_OK) {                           // if sent successfully
      //digitalWrite(cStatusLED, 0);                    // turn off communucation status LED
    }
    else {                                            // otherwise
      //digitalWrite(cStatusLED, 1);                    // turn on communication status LED
    }
  }
  doHeartbeat();                                      // update heartbeat LED
}

// blink heartbeat LED
void doHeartbeat() {
  unsigned long curMillis = millis();                 // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat toggle time for the next cycle
    digitalWrite(cHeartbeatLED, !digitalRead(cHeartbeatLED)); // toggle state of LED
  }
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
  if (dir == 1) {                                     // forward
    ledcWrite(in1, pwm);
    ledcWrite(in2, 0);
  }
  else if (dir == -1) {                               // reverse
    ledcWrite(in1, 0);
    ledcWrite(in2, pwm);
  }
  else {                                              // stop
    ledcWrite(in1, 0);
    ledcWrite(in2, 0);
  }
}

// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
  Encoder* s = static_cast<Encoder*>(arg);            // cast pointer to static structure
  
  int b = digitalRead(s->chanB);                      // read state of channel B
  if (b > 0) {                                        // high, leading channel A
    s->pos++;                                         // increase position
  }
  else {                                              // low, lagging channel A
    s->pos--;                                         // decrease position
  }
}

// callback function for when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)                                       // if empty packet
  {
    return;                                           // return
  }
  memcpy(&inData, incomingData, sizeof(inData));      // store drive data from controller
#ifdef PRINT_INCOMING
  Serial.printf("%d, %d, %d, %d\n", inData.dir, inData.sorting, inData.bucket, inData.time);
#endif
}

// callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef PRINT_SEND_STATUS
  Serial.printf("Last packet send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    //digitalWrite(cStatusLED, 1);                      // turn on communication status LED
    commsLossCount++;                                 // increase lost packet count
  }
  else {
    commsLossCount = 0;                               // reset communication loss counter
  }
}

long degreesToDutyCycle(int deg) {
  const long cl_MinDutyCycle = 1650;                 // duty cycle for 0 degrees
  const long cl_MaxDutyCycle = 8175;                 // duty cycle for 180 degrees

  long l_DutyCycle = map(deg, 0, 180, cl_MinDutyCycle, cl_MaxDutyCycle);  // convert to duty cycle
  return l_DutyCycle;
}