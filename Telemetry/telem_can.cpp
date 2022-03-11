//===========================================
//
//    CAN FUNCTIONS FOR ASV 3 TElEMETRY
//
//===========================================

#include "telem_can.h"

// Initialize CAN bus 
void CAN_init() {
	if (CAN_OK == CAN.begin(CAN_1000KBPS)) {                   // init can bus : baudrate = 1000k
		Serial.println("CAN init ok!");
	}
	else {
		Serial.println("CAN init fail");
		Serial.println("Init CAN again");
		delay(100);
        CAN_init();
	}
}

/* Receive these CAN ID
 *  4:  CAN_HEARTBEAT
 *  10: CAN_BATT1_STATS
 *  11: CAN_BATT2_STATS
 *  17: CAN_SBC_TEMP
 *  18: CAN_POSB_STATS
 */

// Initialize CAN mask using truth table 
/*
  Truth table
  mask  filter    id bit  reject
  0     X         X       no
  1     0         0       no
  1     0         1       yes
  1     1         0       yes
  1     1         1       no
  Mask 0 connects to filt 0,1
  Mask 1 connects to filt 2,3,4,5
  Mask decide which bit to check
  Filt decide which bit to accept
*/
void CAN_mask() {
}

// Receive CAN messages
void checkCAN msg() {
    if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBufID(&id, &len, buf);    // read data,  len: data length, buf: data buf
    switch (CAN.getCanId()) {
      case (CAN_HEARTBEAT): {
        read_can_heartbeat(CAN.parseCANFrame(buf,0,1);
        break;
      }
      case (CAN_BATT1_STATS): {
        read_batt_stats   
      }
      
    }
}

void read_heartbeat() {
}
}
