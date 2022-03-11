//###################################################
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
// Telemetry Firmware for ASV 2.0
//    Drive Telemetry LCD Display using data received over CAN
//    Recieve Thruster, actuated hydrophone, and actuated thruster controls from Frsky. 
//    Receive OCS control information from Controllink via serial.
//    Relay RSSI of OCS and Frsky over CAN.
//    Determine state of control and send control signal to POSB accordingly. 
//
// Written by Titus Ng 
// Change log v0.0:
//
//###################################################
//###################################################

// FOR DEBUG
// #define DEBUG
#ifdef DEBUG
#endif

#include <Wire.h>
#include "Frisky_CPPM.h"
#include "define.h"
#include <Arduino.h>
#include "telem_screen.h"       // own telem_screen library

// Create objects
LCD screen = LCD(SCREEN_CS, SCREEN_RESET); 
Frisky rc = Frisky(RC_INT);
MCP_CAN CAN(CAN_Chip_Select);

// CAN 
uint32_t id = 0;
uint8_t len = 0; //length of CAN message, taken care by library
uint8_t buf[8];  //Buffer for CAN message

// Stats
uint8_t internalStats[INT_STAT_COUNT];
uint16_t powerStats[POWER_STAT_COUNT];
uint32_t heartbeat_timeout[HB_COUNT];

// Timeouts


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
    
}

void loop() {
}
