/** 
 *  @file    telemeter.ino
 *  @author  Marc Sibert
 *  @date    09/2017
 *  @version 1.0 
 *  
 *  @brief Main file for MKRFOX1200 telemeter application.
 *
 *  @section DESCRIPTION
 *  
 *  Cette application permet de surveiller un niveau et  
 *  de déclencher des alertes vers le réseau SigFox en
 *  cas de dépassement de seuil,
 *  soit de niveau maxi ;
 *  soit de variation (dérivée) de ce niveau.
 */

#define DEBUG true

#ifdef DEBUG
#define DEBUG_PRINT(...)    Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...)  Serial.println(__VA_ARGS__)
#define ASSERT(...)         if (!__VA_ARGS__){ Serial.print(F("__VA_ARGS__")); Serial.println(F(" est faux!")); }
#else
#define DEBUG_PRINT(...)    while (false) {}
#define DEBUG_PRINTLN(...)  while (false) {}
#define ASSERT(...)         while (false) {}
#endif

// #include "app.h"
// #include "telemeter.h"
#include <SigFox.h>

/**
 * @brief Telemeter hardware constants.
 */
enum {
  PIN_TEL_TRIG = 0,
  PIN_TEL_ECHO = 1
};

// const Telemeter tel(PIN_TEL_TRIG, PIN_TEL_ECHO);

// App app(tel);

struct message_t {
  byte cmd;
  union {
    unsigned level;
  } payload;
} __attribute__((__packed__));

union response_t {
  uint64_t epoch;
} __attribute__((__packed__));

void setup() { 
//  if (!app.setup()) exit(0);
  Serial.begin(115200);
  while(!Serial) {};

  if (!SigFox.begin()) {
    DEBUG_PRINTLN("Shield Error");
    return;
  }
  delay(100);

#ifdef DEBUG
  SigFox.debug();
#endif
  DEBUG_PRINTLN("Running in DEBUG mode.");
  DEBUG_PRINT("SigFox FW Version "); DEBUG_PRINTLN(SigFox.SigVersion());
  DEBUG_PRINT("Module ID. : ");      DEBUG_PRINTLN(SigFox.ID());
  DEBUG_PRINT("Module PAC. : ");     DEBUG_PRINTLN(SigFox.PAC());

  SigFox.end();

  message_t message;
  message.cmd = 0x01;

  SigFox.begin();
  delay(30);
  SigFox.status();
  delay(1);
  SigFox.beginPacket();
  SigFox.write((uint8_t*)(&message), 1);
  if (SigFox.endPacket(true)) {
    DEBUG_PRINTLN("No transmission");
    DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
    DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
    SigFox.end();
    return;
  }
  if (SigFox.parsePacket()) {
    const response_t response;
    uint8_t* p = (uint8_t*)(&response);
    while (SigFox.available()) {
      *p++ = SigFox.read();
    }
    DEBUG_PRINT("Returned Epoch : ");
    DEBUG_PRINTLN(response.epoch);

  } else {
    DEBUG_PRINTLN("Could not get any response from the server");
    DEBUG_PRINTLN("Check the SigFox coverage in your area");
    DEBUG_PRINTLN("If you are indoor, check the 20dB coverage or move near a window");
  }
  SigFox.end();
}

void loop() { 
//  if (!app.loop()) exit(0);
}

