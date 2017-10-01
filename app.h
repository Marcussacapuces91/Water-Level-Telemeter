/** 
 *  @file    app.h 
 *  @author  Marc Sibert
 *  @date    09/2017  
 *  @version 1.0 
 *  
 *  @brief Class App - Main application setup & loop function.
 *
 *  @section DESCRIPTION
 *  
 *  
 *
 */

#pragma once

#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <utility>
#include <RTCZero.h>

#include "telemeter.h"

// Ordre 1, Longueur 15, derivee
const int16_t SAVGOL_O1_L15_DERIV[] = { 
  32768 * (7) / 280, 32768 * (6) / 280, 32768 * (5) / 280, 32768 * (4) / 280, 32768 * (3) / 280, 32768 * (2) / 280, 32768 * (1) / 280, 
  32768 * 0,
  32768 * (-1) / 280, 32768 * (-2) / 280, 32768 * (-3) / 280, 32768 * (-4) / 280, 32768 * (-5) / 280, 32768 * (-6) / 280, 32768 * (-7) / 280 
};

struct message_t {
  byte cmd;
  union {
    unsigned mediane;
  } payload;
} __attribute__((__packed__));

union response_t {
  struct {
    uint32_t pad1;
    uint32_t epoch;  
  };
  
} __attribute__((__packed__));

/**
 *  @brief Classe qui implémente toutes les méthodes de l'application.
 *  Après le constructeur, il faut appeler la méthode setup une fois, puis
 *  de manière répéter appeler la méthode loop.
 */  
class App {
private:
  const Telemeter& telemeter;
  unsigned long epoch;
  RTCZero rtc;

  static const size_t buffer_size = 21;
  unsigned mesures[buffer_size];
  unsigned buffer[buffer_size];

protected:

  size_t partition(unsigned list[], const size_t gauche, const size_t droite, const size_t pivot) {
    const unsigned valeurPivot = list[pivot];
    std::swap(list[pivot], list[droite]);
    size_t indiceStocage = gauche;
    for (size_t i = gauche; i < droite; ++i) {
      if (list[i] < valeurPivot) {
        std::swap(list[indiceStocage], list[i]);
        ++indiceStocage;
      }
    }
    std::swap(list[indiceStocage], list[droite]);
    return indiceStocage;
  }
  
  unsigned select(unsigned list[], size_t gauche, size_t droite, const size_t n) {
    if (gauche == droite) return list[gauche];
  
    while (true) {
      const size_t p = (gauche + droite) / 2;
      const size_t pivot = partition(list, gauche, droite, p);
      if (n == pivot) return list[n];
      else if (n < pivot) droite = pivot - 1;
      else gauche = pivot + 1;  
    }
  }

/**
 * @brief Start & configure SigFox module.
 * 
 * @return status of the module (true : init fine, false : something wrong)
 */
  bool initSF() {
    if (!SigFox.begin()) {
      DEBUG_PRINTLN("Shield Error");
      return false;
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
    return true;
  }

/**
 * @brief Send a message using SigFox network.
 * 
 * @param message Array of max 12 byte.
 * @param len Length of the message.
 * @return State of the transmission : true succes, false if not.
 */
  bool sendSF(const uint8_t message[], size_t len) {
    if (len == 0) return true;
    SigFox.begin();
    delay(30);
    SigFox.status();
    delay(1);
    SigFox.beginPacket();
    SigFox.write(message, len);
    if (SigFox.endPacket(false)) {  // no answer
      DEBUG_PRINTLN("No transmission");
      DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
      DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
      SigFox.end();
      return false;
    } else {
      SigFox.end();
      return true;
    }
  }

/**
 * @brief Send a message and wait for an answer using the SigFox network.
 * 
 * @param message Un pointeur vers le message à transmettre (maxi 12 octets).
 * @param lenMes La longueur du message en octets.
 * @param response Une référence où sera déposé la réponse (maxi 8 octets).
 * @param lenRes La longueur de la réponse en octets.
 */
  bool sendAckSF(const uint8_t message[], size_t lenMes, uint8_t response[], size_t& lenRes) const {
    ASSERT(message);
    ASSERT(response);
    if (lenMes == 0) return false;
    SigFox.begin();
    delay(30);
    SigFox.status();
    delay(1);
    SigFox.beginPacket();
    SigFox.write(message, lenMes);
    if (SigFox.endPacket(true)) {
      DEBUG_PRINTLN("No transmission");
      DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
      DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
      SigFox.end();
      return false;
    }
    if (SigFox.parsePacket()) {
      lenRes = 0;
      while (SigFox.available()) {
        response[lenRes++] = SigFox.read();
      }
      SigFox.end();
      return true;
    }
    DEBUG_PRINTLN("Could not get any response from the server");
    DEBUG_PRINTLN("Check the SigFox coverage in your area");
    DEBUG_PRINTLN("If you are indoor, check the 20dB coverage or move near a window");
    DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
    DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
    SigFox.end();
    return false;
  }

  
/**
 * @brief Return time as epoch value (seconds since January, the 1st, 1970).
 * 
 * The time is requested through the SigFox backend and a distant webservice.
 * 
 * @return 0 in case of erro, or the epoch value of the current time
 */
  uint32_t getTimeSF() const {
    const message_t message = { .cmd = 0x01 };
//    message.cmd = 0x01;

    response_t response;
    size_t len;
    
    if (sendAckSF(reinterpret_cast<const uint8_t*>(&message), 1, reinterpret_cast<uint8_t*>(&response), len)) {
      return response.epoch;      
    }
  
    return 0;
  }

public:
/**
 * @brief Constructeur
 *  
 * @param aTel une référence sur une instance de Telemeter.
 */  
  App(const Telemeter& aTel) : 
    telemeter(aTel) 
  {
  }


/**
 *   @brief  Cette méthode assure les réglages de tous les éléments de la carte.
 *  
 *   @return Retour un booléen qui indique le bon déroulement de la méthode, ou pas.
 */  
  bool setup() {
    Serial.begin(115200);
    while(!Serial) {};
  
    if (!initSF()) return false;
  
//    epoch = getTimeSF();
//    DEBUG_PRINT("Returned Epoch : "); DEBUG_PRINTLN(epoch);

    if (epoch == 0) epoch = 1506759463; // Saturday September 30 2017  08:17:43 UTC
    epoch -= 3 * 60;

    rtc.begin();
    rtc.setTime((epoch / 3600) % 24, (epoch / 60) % 60, epoch % 60);

    return true;
  }
  
/**
 * @brief Boucle répétée continuement et appelée par le framework Arduino.
 * 
 * @return L'état de l'exécussion d'une itération de la boucle.
 */
  bool loop() {
    if (millis() % 1000 > 0) return true;
    
    long deriv = 0;
    for (size_t i = 0; i < buffer_size; ++i) {
      mesures[i] = (i == buffer_size - 1) ? telemeter.mesure() : mesures[i + 1];   
      if (i < 15) deriv += mesures[i] * SAVGOL_O1_L15_DERIV[i];
    }
  
    DEBUG_PRINT((byte)rtc.getHours());
    DEBUG_PRINT(':');
    DEBUG_PRINT((byte)rtc.getMinutes());
    DEBUG_PRINT(':');
    DEBUG_PRINT((byte)rtc.getSeconds());
  
    DEBUG_PRINT(F(" - Mesure : "));
    DEBUG_PRINT(mesures[buffer_size - 1] / 10.0, 1);
  
    DEBUG_PRINT(F(" - Dérivée : "));
    DEBUG_PRINT(deriv / 327680.0, 2);
  
    unsigned buffer[buffer_size];  
    memcpy(buffer, mesures, sizeof(unsigned) * buffer_size);
    const unsigned mediane = select(buffer, 0, buffer_size - 1, buffer_size / 2);
  
    DEBUG_PRINT(F(" - Mediane : "));
    DEBUG_PRINTLN(mediane / 10.0, 1);
    
    if (rtc.getSeconds() == 0 && (rtc.getMinutes() % 15) == 0) {
      message_t message;
      message.cmd = 0x02;
      message.payload.mediane = mediane;      
      
      DEBUG_PRINT("Send to SigFox : ");
      DEBUG_PRINTLN(message.cmd, HEX);

      return sendSF((uint8_t*)(&message), 3);
    }

    return true;
  }
};

