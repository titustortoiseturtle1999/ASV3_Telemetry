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
  screen.write_string("Hull leak:");
  screen.write_string("POSB OK:");
  screen.write_string("POPB OK:");
  screen.write_string("POKB OK:");
  screen.write_string("LARS OK:");
  screen.write_string("SBC OK:");
  screen.write_string("OCS OK:");
  screen.write_string("Frsky OK:");

  screen.set_cursor(400 + OFFSET, 0);
  screen.write_string("Batt1 capacity:");
  screen.write_string("Batt2 capacity:");
  screen.write_string("Batt1 current:");
  screen.write_string("Batt2 current:");
  screen.write_string("Batt1 voltage:");
  screen.write_string("Batt2 voltage:");
  screen.write_string("Logic Backplane OK:");
  screen.write_string("Ballshooter OK:");
  screen.write_string("Actuated Hydrophone OK:");
//  screen.write_string("Actuated Thrusters OK:");
  screen.write_string("Batt1 OK:");
  screen.write_string("Batt2 OK:");
  screen.write_string("ESC1 OK:");
  screen.write_string("ESC2 OK:");

}

void screen_update_stats() {
  screen.set_cursor(150 + OFFSET, 0);
  for (int i = 0; i < INT_STAT_COUNT; i++)
  {
    if (i = HULL_LEAK){ 
      internalStats[HULL_LEAK] ? screen.write_value_string("LEAK") : screen.write_value_string("NO LEAK");
    }
    else {
      screen.write_value_int(internalStats[i]);
    }
  }

  screen.set_cursor(645 + OFFSET, 0);
  for (int i = 0; i < POWER_STAT_COUNT; i++)
  {
    screen.write_value_int(powerStats[i]);
  }
}

void screen_update_hb() {
  int i; 
  screen.set_cursor(150 + OFFSET, 210);            // do right half
  for (i = 1; i < 9; i++) {
    if (i != TELEMETRY)                           // Skip Telemetry HB
    {
      if ((millis() - heartbeat_timeout[i]) > HB_TIMEOUT) {
        screen.write_value_string("NO");
      }
      else
        screen.write_value_string("YES");
    }
  }
  screen.set_cursor(550 + OFFSET, 210);           // do left half
  for (; i < HB_COUNT; i++) {                     
    if (i != ACTUATED_THRUSTERS) {                // skip actuated thrusters for now 
      if ((millis() - heartbeat_timeout[i]) > HB_TIMEOUT) {
        screen.write_value_string("NO");
      }
      else
        screen.write_value_string("YES");
    }
  }
}
