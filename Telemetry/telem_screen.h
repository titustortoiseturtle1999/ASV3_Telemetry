#ifndef TELEM_SCREEN
#define TELEM_SCREEN

#include <Adafruit_RA8875.h>
#include <Adafruit_GFX.h>
#include "LCD_Driver.h"   
#include "define.h"

extern LCD screen;
extern uint8_t internalStats[];
extern uint16_t powerStats[];

void screen_prepare();
void screen_update();

#endif
