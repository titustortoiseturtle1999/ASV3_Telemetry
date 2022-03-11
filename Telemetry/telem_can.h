//===========================================
//
//    CAN HEADER FOR ASV 3 TElEMETRY
//
//===========================================

#ifndef TELEM_CAN
#define TELEM_CAN

#include "asv_3.0_can_def.h"
#include "define.h"
#include <SPI.h> 
#include <can.h>

void CAN_init();
void CAN_mask();
void read_heartbeat();

#endif
