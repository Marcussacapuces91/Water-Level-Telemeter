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
#define ASSERT(...)         if (!__VA_ARGS__){ Serial.print(F("__VA_ARGS__")); Serial.println(F(" is false!")); }
#else
#define DEBUG_PRINT(...)    while (false) {}
#define DEBUG_PRINTLN(...)  while (false) {}
#define ASSERT(...)         while (false) {}
#endif

#include "app.h"
#include "telemeter.h"

/**
 * @brief Telemeter hardware constants.
 */
enum {
  PIN_TEL_TRIG = 0,
  PIN_TEL_ECHO = 1
};

const Telemeter tel(PIN_TEL_TRIG, PIN_TEL_ECHO);

App app(tel);

void setup() { 
  if (!app.setup()) exit(0);
}

void loop() { 
  if (!app.loop()) exit(0);
}

