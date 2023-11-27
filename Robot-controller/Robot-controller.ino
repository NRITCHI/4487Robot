// 
// MME 4487 Final Project Control System
// 
//  Language: Arduino (C++)
//  Target:   ESP32
//  Author:   Blake Nielsen, Nathan Ritchie, Peter Guatto, Hamza Faish
//  Date:     2023 10 08 
//

 #define PRINT_SEND_STATUS                             // uncomment to turn on output packet send status
// #define PRINT_INCOMING                                // uncomment to turn on output of incoming data

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Function declarations
void doHeartbeat();
void ARDUINO_ISR_ATTR buttonISR(void* arg);
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

// Button structure
struct Button {
  const int pin;                                      // GPIO pin for button
  unsigned int numberPresses;                         // counter for number of button presses
  unsigned int lastPressTime;                         // time of last button press in ms
  bool pressed;                                       // flag for button press event
  bool state;                                         // current state of button; 0 = pressed; 1 = unpressed
  bool lastState;                                     // last state of button
};

// Control data packet structure
struct ControlDataPacket {
  int dir;                                            // drive direction: 1 = forward, -1 = reverse, 0 = stop
  int speed;                                          //motor speed
  int bucket;                                         // 1 = dump bucket
  unsigned long time;                                 // time packet sent
  int turn;                                           // turn direction: 1 = right, -1 = left, 0 = straight
};

// Drive data packet structure
struct DriveDataPacket {
  unsigned long time;                                 // time packet sent
  boolean detected;
};

// Constants
const int cHeartbeatLED = 2;                          // GPIO pin of built-in LED for heartbeat
const int cHeartbeatInterval = 500;                   // heartbeat blink interval, in milliseconds
const int cStatusLED = 26;                            // GPIO pin of communication status LED
const long cDebounceDelay = 20;                       // button debounce delay in milliseconds
const int cMaxDroppedPackets = 20;                    // maximum number of packets allowed to drop
const int cPotPin = 34;                               //potentiometer pin
const int cDetectedPin = 25;

// Variables
unsigned long lastHeartbeat = 0;                      // time of last heartbeat state change
unsigned long lastTime = 0;                           // last time of motor control was updated
unsigned int commsLossCount = 0;                      // number of sequential sent packets have dropped
Button buttonFwd = {14, 0, 0, false, true, true};     // forward, NO pushbutton on GPIO 14, low state when pressed
Button buttonRev = {12, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed
Button buttonLeft = {27, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed
Button buttonRight = {13, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed
Button buttonBucket = {33, 0, 0, false, true, true};     // reverse, NO pushbutton on GPIO 12, low state when pressed


// REPLACE WITH MAC ADDRESS OF YOUR DRIVE ESP32
uint8_t receiverMacAddress[] = {0xA8,0x42,0xE3,0xCA,0x77,0x58};  // MAC address of drive 00:01:02:03:04:05 
esp_now_peer_info_t peerInfo = {};                    // ESP-NOW peer information
ControlDataPacket controlData;                        // data packet to send to drive system
DriveDataPacket inData;                               // data packet from drive system

void setup() {
  Serial.begin(115200);                               // standard baud rate for ESP32 serial monitor
  WiFi.mode(WIFI_STA);                                // use WiFi in station mode
  Serial.print("MAC address ");
  Serial.println(WiFi.macAddress());                  // print MAC address of ESP32
  WiFi.disconnect();                                  // disconnect from network
  
  // Configure GPIO
  pinMode(cHeartbeatLED, OUTPUT);                     // configure built-in LED for heartbeat as output
  pinMode(cStatusLED, OUTPUT);                        // configure GPIO for communication status LED as output
  pinMode(buttonFwd.pin, INPUT_PULLUP);               // configure GPIO for forward button pin as an input with pullup resistor
  attachInterruptArg(buttonFwd.pin, buttonISR, &buttonFwd, CHANGE); // Configure forward pushbutton ISR to trigger on change
  pinMode(buttonRev.pin, INPUT_PULLUP);               // configure GPIO for reverse button pin as an input with pullup resistor
  attachInterruptArg(buttonRev.pin, buttonISR, &buttonRev, CHANGE); // Configure reverse pushbutton ISR to trigger on change
  pinMode(cPotPin, INPUT);
  pinMode(buttonLeft.pin, INPUT_PULLUP);               
  attachInterruptArg(buttonLeft.pin, buttonISR, &buttonLeft, CHANGE);
  pinMode(buttonRight.pin, INPUT_PULLUP);               
  attachInterruptArg(buttonRight.pin, buttonISR, &buttonRight, CHANGE);

  pinMode(buttonBucket.pin, INPUT_PULLUP);               
  attachInterruptArg(buttonBucket.pin, buttonISR, &buttonBucket, CHANGE);
  pinMode(cDetectedPin, OUTPUT);

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
  
  // Set drive info
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);  // set address of peer
  peerInfo.channel = 0;                               // set peer channel 
  peerInfo.encrypt = false;                           // no encryption of data
  
  // Add drive as ESP-NOW peer        
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
}

void loop() {
  esp_err_t result;
  unsigned long curTime = micros();                   // capture current time in microseconds
  if (curTime - lastTime > 10000) {                   // wait ~10 ms
    lastTime = curTime;
    controlData.time = curTime;                       // update transmission time

    controlData.speed = map(analogRead(cPotPin), 0, 4095, 0, 100);

    // drive 
    if (!buttonFwd.state) {                           // forward pushbutton pressed
      controlData.dir = 1;
    }
    else if (!buttonRev.state) {                      // reverse pushbutton pressed
      controlData.dir = -1;
    }
    else {                                            // no input, stop
      controlData.dir = 0;
    }

    if (!buttonLeft.state) {                           // forward pushbutton pressed
      controlData.turn = -1;
    }
    else if (!buttonRight.state) {                      // reverse pushbutton pressed
      controlData.turn = 1;
    }
    else {                                            // no input, stop
      controlData.turn = 0;
    }

    // bucket
    if (!buttonBucket.state) {                           // bucket dump when pressed
      controlData.bucket = 1;
    }
    else {                                            // no input, stop
      controlData.bucket = 0;
    }

    // if drive appears disconnected, update control signal to stop before sending
    if (commsLossCount > cMaxDroppedPackets) {
      controlData.dir = 0;
    }
    // send control signal to drive
    result = esp_now_send(receiverMacAddress, (uint8_t *) &controlData, sizeof(controlData));
    if (result == ESP_OK) {                           // if sent successfully
      digitalWrite(cStatusLED, 0);                    // turn off communucation status LED
    }
    else {                                            // otherwise
      digitalWrite(cStatusLED, 1);                    // turn on communication status LED
    }

    if (inData.detected == true){
      digitalWrite(cDetectedPin, 1);
    }else{
      digitalWrite(cDetectedPin, 0);
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

// button interrupt service routine
// argument is pointer to button structure, which is statically cast to a Button structure, allowing multiple
// instances of the buttonISR to be created (1 per button)
// implements software debounce
void ARDUINO_ISR_ATTR buttonISR(void* arg) {
  Button* s = static_cast<Button*>(arg);              // cast pointer to static structure

  unsigned int pressTime = millis();                  // capture current time
  s->state = digitalRead(s->pin);                     // capture state of button
  // if button has been pressed and sufficient time has elapsed
  if ((!s->state && s->lastState == 1) && (pressTime - s->lastPressTime > cDebounceDelay)) {
    s->numberPresses += 1;                            // increment button press counter
    s->pressed = true;                                // set flag for "valid" button press
  }
  s->lastPressTime = pressTime;                       // update time of last state change
  s->lastState = s->state;                            // save last state
}

// callback function for when data is received
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  if (len == 0)                                       // if empty packet
  {
    return;                                           // return
  }
  memcpy(&inData, incomingData, sizeof(inData));      // store data from drive
#ifdef PRINT_INCOMING
  Serial.printf("%d\n", inData.time);
#endif
}

// callback function for when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#ifdef PRINT_SEND_STATUS
  Serial.printf("Last packet send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "Message sent" : "Message failed");
#endif
  if (status != ESP_NOW_SEND_SUCCESS) {
    digitalWrite(cStatusLED, 1);                      // turn on communication status LED
    commsLossCount++;                                 // increase lost packet count
  }
  else {
    commsLossCount = 0;                               // reset communication loss counter
  }
}
