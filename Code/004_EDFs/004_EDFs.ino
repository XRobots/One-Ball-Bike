#include <Servo.h> 

Servo EDF1;
Servo EDF2;

#include <Arduino.h>
#include "ODriveCAN.h"

//PID
#include <PID_v1.h>

// PID1 - roll
double Pk1 = 2.5;
double Ik1 = 17;
double Dk1 = 0.07;

double Setpoint1, Input1, Output1, Output1a;    // PID variables
PID PID1(&Input1, &Output1, &Setpoint1, Pk1, Ik1 , Dk1, DIRECT);    // PID Setup

// PID2 - pitch
double Pk2 = 2.5;
double Ik2 = 17;
double Dk2 = 0.07;

double Setpoint2, Input2, Output2, Output2a;    // PID variables
PID PID2(&Input2, &Output2, &Setpoint2, Pk2, Ik2 , Dk2, DIRECT);    // PID Setup

// CAN bus baudrate. Make sure this matches for every device on the bus
#define CAN_BAUDRATE 250000

// ODrive node_id for odrvives
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.


/* Board-specific includes ---------------------------------------------------*/

#if defined(IS_TEENSY_BUILTIN) + defined(IS_ARDUINO_BUILTIN) + defined(IS_MCP2515) != 1
#warning "Select exactly one hardware option at the top of this file."

#if CAN_HOWMANY > 0 || CANFD_HOWMANY > 0
#define IS_ARDUINO_BUILTIN
#warning "guessing that this uses HardwareCAN"
#else
#error "cannot guess hardware version"
#endif

#endif

#ifdef IS_ARDUINO_BUILTIN
// See https://github.com/arduino/ArduinoCore-API/blob/master/api/HardwareCAN.h
// and https://github.com/arduino/ArduinoCore-renesas/tree/main/libraries/Arduino_CAN

#include <Arduino_CAN.h>
#include <ODriveHardwareCAN.hpp>
#endif // IS_ARDUINO_BUILTIN

#ifdef IS_MCP2515
// See https://github.com/sandeepmistry/arduino-CAN/
#include "MCP2515.h"
#include "ODriveMCPCAN.hpp"
#endif // IS_MCP2515

# ifdef IS_TEENSY_BUILTIN
// See https://github.com/tonton81/FlexCAN_T4
// clone https://github.com/tonton81/FlexCAN_T4.git into /src
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error
#endif // IS_TEENSY_BUILTIN

/* Board-specific settings ---------------------------------------------------*/

/* Teensy */

#ifdef IS_TEENSY_BUILTIN

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

#endif // IS_TEENSY_BUILTIN

/* MCP2515-based extension modules -*/

#ifdef IS_MCP2515

MCP2515Class& can_intf = CAN;

// chip select pin used for the MCP2515
#define MCP2515_CS 10

// interrupt pin used for the MCP2515
// NOTE: not all Arduino pins are interruptable, check the documentation for your board!
#define MCP2515_INT 2

// freqeuncy of the crystal oscillator on the MCP2515 breakout board. 
// common values are: 16 MHz, 12 MHz, 8 MHz
#define MCP2515_CLK_HZ 8000000


static inline void receiveCallback(int packet_size) {
  if (packet_size > 8) {
    return; // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(), .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onCanMessage(msg);
}

bool setupCan() {
  // configure and initialize the CAN bus interface
  CAN.setPins(MCP2515_CS, MCP2515_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);
  if (!CAN.begin(CAN_BAUDRATE)) {
    return false;
  }

  CAN.onReceive(receiveCallback);
  return true;
}

#endif // IS_MCP2515


/* Arduinos with built-in CAN */

#ifdef IS_ARDUINO_BUILTIN

HardwareCAN& can_intf = CAN;

bool setupCan() {
  return can_intf.begin((CanBitRate)CAN_BAUDRATE);
}

#endif

// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID); // Standard CAN message ID
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0, &odrv1, &odrv2}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};

// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;
ODriveUserData odrv4_user_data;

// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data_ptr) {
  ODriveUserData* odrv_user_data_ptr = static_cast<ODriveUserData*>(user_data_ptr);
  odrv_user_data_ptr->last_heartbeat = msg;
  odrv_user_data_ptr->received_heartbeat = true;
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data_ptr) {
  ODriveUserData* odrv_user_data_ptr = static_cast<ODriveUserData*>(user_data_ptr);
  odrv_user_data_ptr->last_feedback = msg;
  odrv_user_data_ptr->received_feedback = true;
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }

}

// Uncomment below the line that corresponds to your hardware.
// See also "Board-specific settings" to adapt the details for your hardware setup.

#define IS_TEENSY_BUILTIN // Teensy boards with built-in CAN interface (e.g. Teensy 4.1). See below to select which interface to use.
// #define IS_ARDUINO_BUILTIN // Arduino boards with built-in CAN interface (e.g. Arduino Uno R4 Minima)
// #define IS_MCP2515 // Any board with external MCP2515 based extension module. See below to configure the module.

#include <Wire.h>

// #include <FastGPIO.h>
// #define APA102_USE_FAST_GPIO

#include <APA102.h>
// Define which pins to use.
const uint8_t dataPin = 2;
const uint8_t clockPin = 3;

// ** DEFINE VARIABLES **

int LED0;
int LED1;
int LED2;
int LED3;
int LED4;
int LED5;
int LED6;

float roll;
float rollTrim;
float rollTrimmed;

float pitch;
float pitchTrim;
float pitchTrimmed;

int pot1;
int pot2;
int pot3;
float pot4;
float pot5;
float pot6;
float pot7;
int sw1;        // ODrive init button
int sw2;        // handlebar shift switch

float twist1;
float twist2;
float twist1Filtered;
float twist2Filtered;

int EDF1Output;
int EDF2Output;
int EDF1OutputConstrained;
int EDF2OutputConstrained;

float filterRoll;
float filterPitch;

float overallGain;

float wheel0;
float wheel1;
float wheel2;

int clFlag;  // ODrive init flag

int EMflag = 0;  // Emergency stop flag - if it falls over

unsigned long currentMillis;
unsigned long previousMillis = 0;         // set up timers - timed loop
unsigned long previousEMMillis = 0;       // set up timers - emergency motor disable
long interval = 10;                       // loop time

// Create an object for writing to the LED strip.
APA102<dataPin, clockPin> ledStrip;

// Set the number of LEDs to control.
const uint16_t ledCount = 7;

#include "SparkFun_BNO08x_Arduino_Library.h"  // CTRL+Click here to get the library: http://librarymanager/All#SparkFun_BNO08x
BNO08x myIMU;

#define BNO08X_INT  17
#define BNO08X_RST  16
#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B

void setup() {

  pinMode(A10, INPUT);    // ctrl panel top pot
  pinMode(A11, INPUT);    // ctrl panel bottom pot
  pinMode(A12, INPUT);    // external pot twist grip
  pinMode(A13, INPUT);    // external pot twist grip
  pinMode(A15, INPUT);    // pot
  pinMode(A16, INPUT);    // pot
  pinMode(A17, INPUT);    // pot

  pinMode(4, INPUT_PULLUP);   // ctrl panel switch - motor init
  pinMode(32, INPUT_PULLUP);  // external handlebar shift switch

  EDF1.attach(37);
  EDF2.attach(36);
  
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO08x Read Example");

  PID1.SetMode(AUTOMATIC);              
  PID1.SetOutputLimits(-65, 65);
  PID1.SetSampleTime(10);

  PID2.SetMode(AUTOMATIC);              
  PID2.SetOutputLimits(-65, 65);
  PID2.SetSampleTime(10);

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("BNO08x not detected at default I2C address. Check your jumpers and the hookup guide. Freezing...");
    ledStrip.startFrame();
    ledStrip.sendColor(LED0, LED0, LED0);
    ledStrip.sendColor(LED1, LED1, LED1);
    ledStrip.sendColor(LED2, LED2, LED2);  
    ledStrip.sendColor(50, LED3, LED3);       // error LED
    ledStrip.sendColor(LED4, LED4, LED4);   
    ledStrip.sendColor(LED5, LED5, LED5);   
    ledStrip.sendColor(LED6, LED6, LED6);  
    ledStrip.endFrame(7);
    while (1)
      ;
  }
  
  Serial.println("BNO08x found!");
  ledStrip.startFrame();
  ledStrip.sendColor(LED0, LED0, LED0);
  ledStrip.sendColor(LED1, LED1, LED1);
  ledStrip.sendColor(LED2, LED2, LED2);  
  ledStrip.sendColor(LED1, 50, LED3);       // no error LED
  ledStrip.sendColor(LED4, LED4, LED4);   
  ledStrip.sendColor(LED5, LED5, LED5);   
  ledStrip.sendColor(LED6, LED6, LED6);  
  ledStrip.endFrame(7);

  setReports();

  Serial.println("Reading events");
  delay(100);

  // Wait for up to 3 seconds for the serial port to be opened on the PC side.
  // If no PC connects, continue anyway.
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(100);
  }
  delay(200);

  Serial.println("Starting ODriveCAN");

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);
  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);
  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);

  // Configure and initialize the CAN bus interface. This function depends on
  // your hardware and the CAN stack that you're using.
  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true); // spin indefinitely
  }    
}

// Here is where you define the sensor outputs you want to receive
void setReports(void) {
  Serial.println("Setting desired reports");
  if (myIMU.enableRotationVector() == true) {
    Serial.println(F("Rotation vector enabled"));
    Serial.println(F("Output in form roll, pitch, yaw"));
  } else {
    Serial.println("Could not enable rotation vector");
  }
}

// motion filter to filter motions

float filter(float prevValue, float currentValue, int filter) {  
  float lengthFiltered =  (prevValue + (currentValue * filter)) / (filter + 1);  
  return lengthFiltered;  
}

void loop() {

      // ** READ IMU DATA **
      if (myIMU.wasReset()) {
        Serial.println("sensor was reset ");
        setReports();
      }    
      // Has a new event come in on the Sensor Hub Bus?
      if (myIMU.getSensorEvent() == true) {    
        // is it the correct sensor data we want?
        if (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {     
            roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
            pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
        }
      }
      // ** END OF READ IMU DATA **

      // ** CAN / ODRIVE STUFF **
      pumpEvents(can_intf); // This is required on some platforms to handle incoming feedback CAN messages


      // ** TIMED LOOP AT 10MS FOR EVERTHING ELSE ** 
      currentMillis = millis();
      if (currentMillis - previousMillis >= 10) {
          previousMillis = currentMillis;
    
          // ** READ SWITCHES **
          pot1 = analogRead(A10);       // left pot - roll trim
          pot2 = analogRead(A11);       // right pot - pitch trim
          pot3 = analogRead(A0);        // overall gain
          pot4 = analogRead(A14);       // panel top left
          pot5 = analogRead(A15);       // panel bottom left
          pot6 = analogRead(A16);       // panel top right
          pot7 = analogRead(A17);       // panel bottom left
          sw1 = digitalRead(4);         // ctrl panel switch - motor init
    
          twist1 = analogRead(A12);
          twist2 = analogRead(A13); 

          sw2 = digitalRead(32);
          //Serial.println(sw2);
    
          // *** RH Twist
    
          twist1 = twist1 - 523;  // centre at zero
          if (twist1 < 3 && twist1 > 0) {  // positive deadspot
              twist1 = 0;
          }
          else if (twist1 >= 3) {
            twist1 = twist1 - 3;
          }
          if (twist1 > -3 && twist1 < 0) {  // negative deadspot
              twist1 = 0;
          }
          else if (twist1 <= -3) {
            twist1 = twist1 + 3;
            twist1 = twist1 * 1.5;  // rescale to match positive magnitute
          }
    
          // *** LH Twist

          
    
          twist2 = twist2 - 515;  // centre at zero
           
          if (twist2 < 3 && twist2 > 0) {  // positive deadspot
              twist2 = 0;
          }
          else if (twist2 >= 3) {
            twist2 = twist2 - 3;
          }
          if (twist2 > -3 && twist2 < 0) {  // negative deadspot
              twist2 = 0;
          }
          else if (twist2 <= -3) {
            twist2 = twist2 + 3;
            twist2 = twist2 * 1.5;  // rescale to match positive magnitute
          }
    
          twist1 = constrain(twist1,-55,55);
          twist2 = constrain(twist2,-55,55);          
    
          // panel pots
    
          pot4 = pot4 / 1023;   // roll scaler
          pot5 = pot5;   // roll filter
          pot6 = pot6 / 1023;   // pitch scaler
          pot7 = pot7;   // pitch filter

          // scale for motion filters          
          filterRoll = map(pot5,0,1023,10,60);
          filterPitch = map(pot7,0,1023,10,60);

          // filter twist grip motions
          twist1Filtered = filter(twist1, twist1Filtered, filterPitch);
          twist2Filtered = filter(twist2, twist2Filtered, filterRoll);

          EDF1Output = map(twist2Filtered, -55,55, 1300,1700);
          EDF2Output = map(twist2Filtered, -55,55, 1700,1300);

          EDF1OutputConstrained = constrain(EDF1Output, 1500,1700);
          EDF2OutputConstrained = constrain(EDF2Output, 1500,1700);

          EDF1.writeMicroseconds(EDF1OutputConstrained);
          EDF2.writeMicroseconds(EDF2OutputConstrained);
        
          if (sw1 == 0) {          // Init Odrives
    
              // Check for Odrives
    
              Serial.println("Waiting for ODrive 0...");
              while (!odrv0_user_data.received_heartbeat) {
                pumpEvents(can_intf);
                delay(100);
              }
              Serial.println("found ODrive 0");  
    
              Serial.println("Waiting for ODrive 1...");
              while (!odrv1_user_data.received_heartbeat) {
                pumpEvents(can_intf);
                delay(100);
              }
              Serial.println("found ODrive 1");
             
              Serial.println("Waiting for ODrive 2...");
              while (!odrv2_user_data.received_heartbeat) {
                pumpEvents(can_intf);
                delay(100);
              }
              Serial.println("found ODrive 2");   
              
              odrv0.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
              odrv1.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
              odrv2.setControllerMode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH);
          
              for (int i = 0; i < 15; ++i) {
                delay(10);
                pumpEvents(can_intf);
              }
            
              Serial.println("Enabling closed loop control 0...");
                while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrv0.clearErrors();
                delay(1);
                odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
              }
    
              Serial.println("Enabling closed loop control 1...");
              while (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrv1.clearErrors();
                delay(1);
                odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
              }
            
              Serial.println("Enabling closed loop control 2...");
              while (odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
                odrv2.clearErrors();
                delay(1);
                odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
              }     
              
              clFlag = 1;          // reset flag
          }
    
          else if (sw1 == 1) {      // make sure we only power on ODrives once per button press
            clFlag = 0;
          }
    
          rollTrim = (float) (pot1 - 512)/100;
          rollTrimmed = (((roll*-1)-3) + (rollTrim*-1));      
    
          pitchTrim = (float) (pot2 - 512)/50;
          pitchTrimmed = (pitch*-1) + pitchTrim;
    
          // display IMU data on LEDs
      
          if (rollTrimmed > -1.5 && rollTrimmed < 1.5) {
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 50;
            LED4 = 0;
            LED5 = 0;
            LED6 = 0;      
          }
          else if (rollTrimmed > -3 && rollTrimmed < -1.5) {
            LED0 = 0;
            LED1 = 0;
            LED2 = 50;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            LED6 = 0;     
          }
          else if (rollTrimmed < 3 && rollTrimmed > 1.5) {
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 50;
            LED5 = 0;
            LED6 = 0;      
          }
          else if (rollTrimmed > -4.5 && rollTrimmed < -3) {
            LED0 = 0;
            LED1 = 50;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            LED6 = 0;      
          }
          else if (rollTrimmed < -4.5) {
            LED0 = 50;
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            LED6 = 0;      
          }
          else if (rollTrimmed < 4.5 && rollTrimmed > 3) {
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 50;
            LED6 = 0;      
          }
          else if (rollTrimmed > 4.5) {
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            LED6 = 50;      
          }
          else {
            LED0 = 0;
            LED1 = 0;
            LED2 = 0;
            LED3 = 0;
            LED4 = 0;
            LED5 = 0;
            LED6 = 0;
          }
        
          ledStrip.startFrame();                          // Write to LEDs
          ledStrip.sendColor(LED0, LED0, LED0);
          ledStrip.sendColor(LED1, LED1, LED1);
          ledStrip.sendColor(LED2, LED2, LED2);  
          ledStrip.sendColor(LED3, LED3, LED3);   
          ledStrip.sendColor(LED4, LED4, LED4);   
          ledStrip.sendColor(LED5, LED5, LED5);   
          ledStrip.sendColor(LED6, LED6, LED6);   
          ledStrip.endFrame(7);

          if (sw2 == 0) {
            Input1 = rollTrimmed + ((twist2Filtered/-2.5) * pot4);
          }
          else {
            Input1 = rollTrimmed;
          }   
          
          Setpoint1 = 0;
          PID1.Compute();
    
          Input2 = pitchTrimmed + ((twist1Filtered/2.5) * pot6);
          Setpoint2 = 0;
          PID2.Compute();
    
          overallGain = (float) pot3 / 1023; 
    
          wheel0 = (Output2) * overallGain;
          wheel1 = (((Output2 * 0.5) *-1) + Output1) * overallGain;
          wheel2 = (((Output2 * 0.5)) + Output1) * overallGain;
    
          if (rollTrimmed > -20 && rollTrimmed < 20 && pitchTrimmed > -20 && pitchTrimmed < 20) {      // E-stop condition if we tip over too far
              previousEMMillis = currentMillis;     // bookmark the time
          }

          if (currentMillis - previousEMMillis >= 1000) {     // EMergency stop if time is over 1 second
              EMflag = 1;
          }
    
          // *** drive ODrives ***      
          if (EMflag == 0) {      
            odrv0.setVelocity(wheel0);    // back
            odrv1.setVelocity(wheel1);    // front right
            odrv2.setVelocity(wheel2);    // front left
          }
          // *** emergency stop ***
          else if (EMflag == 1) {
            odrv0.setVelocity(0);    // back
            odrv1.setVelocity(0);    // front right
            odrv2.setVelocity(0);    // front left

            ledStrip.startFrame();
            ledStrip.sendColor(50, LED0, LED0);   // error LEDs
            ledStrip.sendColor(50, LED1, LED1);
            ledStrip.sendColor(50, LED2, LED2);  
            ledStrip.sendColor(50, LED3, LED3);
            ledStrip.sendColor(50, LED4, LED4);   
            ledStrip.sendColor(50, LED5, LED5);   
            ledStrip.sendColor(50, LED6, LED6);  
            ledStrip.endFrame(7);
            while (1);
          }

} // ** END OF TIMED LOOP **

} // end of main loop
