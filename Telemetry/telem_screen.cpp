//===========================================
//
//    SCREEN FUNCTIONS FOR ASV TElEMETRY
//
//===========================================
#include "telem_screen.h"

void screen_prepare() {
  screen.set_cursor(0 + OFFSET, 0);
  screen.write_string("Int press:");
  screen.write_string("Humidity:");
  screen.write_string("CPU temp:");
  screen.write_string("POSB temp:");
  screen.write_string("RSSI OCS:");
  screen.write_string("RSSI RC:");
  screen.write_string("POSB OK:");
  screen.write_string("POPB OK:");
  screen.write_string("POKB OK:");
  screen.write_string("LARS OK:");
  screen.write_string("SBC OK:");
  screen.write_string("OCS OK:");
  screen.write_string("RC OK:");

  screen.set_cursor(400 + OFFSET, 0);
  screen.write_string("Batt1 capacity:");
  screen.write_string("Batt2 capacity:");
  screen.write_string("Batt1 current:");
  screen.write_string("Batt2 current:");
  screen.write_string("Batt1 voltage:");
  screen.write_string("Batt2 voltage:");
  screen.write_string("Batt1 OK:");
  screen.write_string("Batt2 OK:");
  screen.write_string("ESC1 OK:");
  screen.write_string("ESC2 OK:");
}

void screen_update() {
  screen.set_cursor(150 + OFFSET, 0);
  for (int i = 0; i < INT_STAT_COUNT; i++)
  {
    screen.write_value_int(internalStats[i]);
  }

  screen.set_cursor(645 + OFFSET, 0);
  for (int i = 0; i < POWER_STAT_COUNT; i++)
  {
    screen.write_value_int(powerStats[i]);
  }
}
