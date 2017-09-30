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
// #include <stdlib.h>

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
 * @brief Return time as epoch value (seconds since January, the 1st, 1970).
 * 
 * The time is requested through the SigFox backend and a distant webservice.
 * 
 * @return 0 in case of erro, or the epoch value of the current time
 */
  uint32_t getTimeSF() const {
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
      return 0;
    }
    if (SigFox.parsePacket()) {
      response_t response;
      uint8_t* p = (uint8_t*)(&response);
      while (SigFox.available()) {
        *p++ = SigFox.read();
      }
      DEBUG_PRINT("Returned Epoch : "); DEBUG_PRINTLN(response.epoch);
      SigFox.end();
      return response.epoch;
    }
    DEBUG_PRINTLN("Could not get any response from the server");
    DEBUG_PRINTLN("Check the SigFox coverage in your area");
    DEBUG_PRINTLN("If you are indoor, check the 20dB coverage or move near a window");
    DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
    DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
    SigFox.end();
    return 0;
  }


/**
 * @brief Envoie un message sur le réseau SigFox sans attendre de retour.
 * 
 * @param mess Une référence sur le texte du message à transmettre.
 * @return L'état de la transmission, true en cas de succès, false sinon.
 */
  bool send(const String& mess) const {
    SigFox.begin();
    delay(100);
    SigFox.status();
    delay(1);

    SigFox.beginPacket();
    SigFox.print(mess);
    Serial.print(F("send to SigFox : "));
    Serial.println(mess);

    if (SigFox.endPacket(false) > 0) {
      Serial.println(F("No transmission"));
      Serial.println(SigFox.status(SIGFOX));
      Serial.println(SigFox.status(ATMEL));
      SigFox.end();
      delay(100);
      return false;
    }
    SigFox.end();
    delay(100);
    return true;
  }
  
/**
 * @brief Envoie un message sur le réseau SigFox et retourne la réponse.
 * 
 * @param mess Une référence sur le texte du message à transmettre.
 * @return La réponse ou une chaîne vide en cas d'échec de transmission ou de réception.
 */
  String sendAck(const String& mess) const {
    SigFox.begin();
    delay(100);
    SigFox.status();
    delay(1);
        
    Serial.print(F("sendAck to SigFox : "));
    Serial.print(mess);

    String rep;
    SigFox.beginPacket();
    SigFox.print(mess);
    if (SigFox.endPacket(true) > 0) {
      Serial.println(F("No transmission"));
      Serial.println(SigFox.status(SIGFOX));
      Serial.println(SigFox.status(ATMEL));
      SigFox.end();
      delay(100);
      return rep;
    }
    if (SigFox.parsePacket()) {
      while (SigFox.available()) {
        rep += static_cast<char>(SigFox.read());
      }
      Serial.print(F(" -> "));
      Serial.println(rep);
      SigFox.end();
      delay(100);
      return rep;
    }
    Serial.println(F("Could not get any response from the server"));
    Serial.println(F("Check the SigFox coverage in your area"));
    Serial.println(F("If you are indoor, check the 20dB coverage or move near a window"));
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
    SigFox.end();
    delay(100);
    return rep;
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
    if (epoch == 0) epoch = 1506759463; // Saturday September 30 2017  08:17:43 UTC

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
  
    Serial.print(F("Epoch : "));
    Serial.print(epoch);
  
    Serial.print(F(" - Mesure : "));
    Serial.print(mesures[buffer_size - 1] / 10.0, 1);
  
    Serial.print(F(" - Dérivée : "));
    Serial.print(deriv / 327680.0, 2);
  
    unsigned buffer[buffer_size];  
    memcpy(buffer, mesures, sizeof(unsigned) * buffer_size);
    const unsigned mediane = select(buffer, 0, buffer_size - 1, buffer_size / 2);
  
    Serial.print(F(" - Mediane : "));
    Serial.print(mediane / 10.0, 1);
    
    Serial.println();
  
    if (rtc.getSeconds() == 0) {
      message_t message;
      message.cmd = 0x02;
      message.payload.mediane = mediane;      
      
      SigFox.begin();
      delay(30);
      SigFox.status();
      delay(1);
      SigFox.beginPacket();
      SigFox.write((uint8_t*)(&message), 2);
      if (SigFox.endPacket(false)) {
        DEBUG_PRINTLN("No transmission");
        DEBUG_PRINT("SigFox Status : "); DEBUG_PRINTLN(SigFox.status(SIGFOX));
        DEBUG_PRINT("Atmel Status : ");  DEBUG_PRINTLN(SigFox.status(ATMEL));
        SigFox.end();
        return false;
      }
      SigFox.end();
    }

    return true;
  }
};

