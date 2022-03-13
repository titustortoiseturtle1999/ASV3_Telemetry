//###################################################
//
//___.  ___.                         
//\_ |__\_ |__ _____    _________  __
// | __ \| __ \\__  \  /  ___/\  \/ /
// | \_\ \ \_\ \/ __ \_\___ \  \   / 
// |___  /___  (____  /____  >  \_/  
//     \/    \/     \/     \/        
//
// Telemetry for BBASV 3.0
//    Goals:
//    Drive Telemetry LCD Display using data received over CAN                          Done 
//    Recieve Thruster, actuated hydrophone, and actuated thruster controls from Frsky  Done (left actuated hydrophone and thruster)
//    Receive OCS control information from Controllink via serial.                      Not done -> thruster values? 
//    Relay RSSI of OCS and Frsky over CAN.                                             Frsky done OCS no
//    Determine state of control and send control signal to POSB accordingly.           Not done -> left OCS 
//
// Written by Titus Ng 
// Change log v1.3:
// Add Frsky library and functions 
//
//###################################################

// FOR DEBUG
// #define DEBUG
#ifdef DEBUG
#endif

#include <Wire.h>
#include "define.h"
#include <Arduino.h>
#include "telem_screen.h"       
#include "telem_can.h" 
#include "telem_frsky.h"         

// Create objects
LCD screen = LCD(SCREEN_CS, SCREEN_RESET); 
Frisky frsky = Frisky(RC_INT);
MCP_CAN CAN(CAN_Chip_Select);

// CAN 
uint32_t id = 0;
uint8_t len = 0; //length of CAN message, taken care by library
uint8_t buf[8];  //Buffer for CAN message

// Stats
uint32_t internalStats[INT_STAT_COUNT];
uint32_t powerStats[POWER_STAT_COUNT];
uint32_t heartbeat_timeout[HB_COUNT];

// loop times
static uint32_t screenloop;
static uint32_t currentTime;
static uint32_t hbloop;
static uint32_t controllinkloop;
static uint32_t thrusterloop;

// control 
int control_mode_frsky = AUTONOMOUS;
int control_mode = AUTONOMOUS;
int control_mode_ocs = AUTONOMOUS;
int16_t dir_forward = 0;
int16_t dir_side = 0;
int16_t dir_yaw = 0;
int16_t speed1 = 0;
int16_t speed2 = 0;
int16_t speed3 = 0;
int16_t speed4 = 0;

void setup() {
  pinMode(SCREEN_CS, OUTPUT);           //CS screen
  digitalWrite(SCREEN_CS, HIGH);
  pinMode(CAN_Chip_Select, OUTPUT);     //CS CAN
  digitalWrite(CAN_Chip_Select, HIGH);

  Serial.begin(115200);
  Serial.println("Hi, I'm ASV 3 Telemetry!");

  //Screen init
  screen.screen_init();
  Serial.println("Screen Ok");
  screen_prepare();

  // CAN init 
  CAN_init();
  CAN_mask();

  // frksy
  frsky.init();

  // DAC init
  Wire.begin();

  // initialise heartbeat 
  currentTime = screenloop = hbloop = millis();
  for (int i = 0; i < HB_COUNT; i++) {
    heartbeat_timeout[i] = millis();
  }
}

void loop() {
  screen_reset_stats();   // reset stats if past timeout
  CAN_read_msg();         // read incoming CAN messages

  if ((millis() - screenloop) > SCREEN_LOOP) {      // update screen
    screen_update_stats();
    screen_update_hb();
    screenloop = millis();
  }

  frsky_get_controlmode();        // get control mode from frsky 
  frsky_get_rssi();           
  frsky_send_batt_capacity();
//  get_controlmode();            // identify control mode based on control architecture 
  set_thruster_values();        // transmit thruster commands

  if ((millis() - hbloop) > HEARTBEAT_LOOP) {     
    CAN_publish_hb(TELEMETRY);                          // Send telemetry hb 
    if (internalStats[RSSI_FRSKY] > RSSI_THRESHOLD) {
      CAN_publish_hb(FRSKY);                            // send frsky hb if rssi is above threshold
    }
    hbloop = millis();
  }

  if ((millis() - controllinkloop) > CONTROLLINK_LOOP) {      
    CAN_publish_controllink();                          // Send control link message
    controllinkloop = millis();
  }

  if ((millis() - thrusterloop) > THRUSTER_LOOP) {
    CAN_publish_manualthruster();
    thrusterloop = millis();
  }
}

//==========================================
//          THRUSTER FUNCTIONS
//==========================================

// Map CPPM values to thruster values
// CPPM: 980 to 2020 (neutral: 1500) (with deadzone from -24 to 24)
// Thruster: -3200 to 3200 (neutral: 0)
int32_t map_cppm(uint32_t value) {
  value = remove_deadzone(value);
  if (value >= 1500) {
    value = map(value, 1524, 2020, 0, 3200);  // Map values
  }
  else {
    value = map(value, 980, 1476, -3200, 0);
  }
  return value;
}

// Remove deadzone around 1500 (from +24 to -24)
uint32_t remove_deadzone(uint32_t value) {
  if (value >= 1500) {
    return constrain(value, 1524, 2020);
  }
  else {
    return constrain(value, 980, 1476);
  }
}

void get_directions() {
  dir_forward = map_cppm(frsky.get_ch(FRISKY_FORWARD));
  dir_side = map_cppm(frsky.get_ch(FRISKY_SIDE));
  dir_yaw = map_cppm(frsky.get_ch(FRISKY_YAW));
}

void set_thruster_values() {
  if ((millis() - heartbeat_timeout[POKB]) > FAILSAFE_TIMEOUT) {
    // If no POKB heartbeat, stop all thrusters
    reset_thruster_values();
  }
  else  {
    switch (control_mode) {
    case AUTONOMOUS:
    case STATION_KEEP:
      reset_thruster_values();            // If not in manual mode, reset all thrusters
      break;
    case MANUAL_FRSKY:
      convert_thruster_values();
      break;
    }
  }
}
// Resolve vector thrust & Cap thrust at -3200 and 3200
void convert_thruster_values() {
  speed1 = constrain(-dir_forward - dir_side - dir_yaw, -3200, 3200);
  speed2 = constrain(-dir_forward + dir_side + dir_yaw, -3200, 3200);
  speed3 = constrain(dir_forward - dir_side + dir_yaw, -3200, 3200);
  speed4 = constrain(dir_forward + dir_side - dir_yaw, -3200, 3200);
}
void reset_thruster_values() {
  speed1 = 0;
  speed2 = 0;
  speed3 = 0;
  speed4 = 0;
}

//==========================================
//          CONTROL FUNCTIONS
//==========================================
void get_controlmode() {
   // If Frsky alive && it says manual, state = MANUAL_FRSKY
   
}

void CAN_publish_manualthruster()
{
  CAN.setupCANFrame(buf, 0, 2, (uint32_t)(speed1 + 3200));
  CAN.setupCANFrame(buf, 2, 2, (uint32_t)(speed2 + 3200));
  CAN.setupCANFrame(buf, 4, 2, (uint32_t)(speed3 + 3200));
  CAN.setupCANFrame(buf, 6, 2, (uint32_t)(speed4 + 3200));
  CAN.sendMsgBuf(CAN_MANUAL_THRUSTER, 0, 8, buf);
}
