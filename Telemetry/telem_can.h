//===========================================
//
//    CAN HEADER FOR ASV 3 TElEMETRY
//
//===========================================

// FOR DEBUG
// #define DEBUG
#ifdef DEBUG
#endif

#ifndef TELEM_CAN
#define TELEM_CAN

#include "asv_3.0_can_def.h"
#include "define.h"
#include <SPI.h> 
#include <can.h>

extern MCP_CAN CAN;
extern uint32_t id;
extern uint8_t len;                     //length of CAN message, taken care by library
extern uint8_t buf[];                                 //Buffer for CAN message
extern uint16_t internalStats[INT_STAT_COUNT];                      // Array for internal stats
extern uint16_t powerStats[POWER_STAT_COUNT];         // Array for power stats
extern uint32_t heartbeat_timeout[HB_COUNT];          // Array for heartbeat timeout 



void CAN_init();
void CAN_mask();
void CAN_read_msg();
void read_heartbeat();
void read_batt_stats(int);
void read_posb_stats();



#endif
