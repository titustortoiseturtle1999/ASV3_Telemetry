#ifndef TELEM_SCREEN
#define TELEM_SCREEN

#include <Adafruit_RA8875.h>
#include <Adafruit_GFX.h>
#include "LCD_Driver.h"   
#include "define.h"

extern LCD screen;
extern uint16_t internalStats[];
extern uint16_t powerStats[];
extern uint32_t heartbeat_timeout[];

void screen_prepare();
void screen_update_stats();
void screen_update_heartbeat();
void screen_update_hb();

#endif
