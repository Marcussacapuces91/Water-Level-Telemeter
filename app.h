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

/**
 *  @brief Classe qui implémente toutes les méthodes de l'application.
 *  Après le constructeur, il faut appeler la méthode setup une fois, puis
 *  de manière répéter appeler la méthode loop.
 */  
class App {
private:
  const Telemeter& telemeter;
  long epoch;
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
 *   @brief Affiche les informations du module SigFox sur le port série par défaut.
 *   
 *   @return rien.
 */
  void displayInfo() const {
    Serial.println(F("MKRFox1200 Sigfox configuration"));
    Serial.print(F("SigFox FW version "));    Serial.println(SigFox.SigVersion());
    Serial.print(F("ID  = "));                Serial.println(SigFox.ID());
    Serial.print(F("PAC = "));                Serial.println(SigFox.PAC());
    Serial.print(F("Module temperature "));   Serial.println(SigFox.internalTemperature());
  
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
    SigFox.status();    
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
    Serial.begin(9600);
    while(!Serial) {}

/*
    if (!SigFox.begin()) {
      Serial.println(F("Shield error or not present!"));
      return false;
    }
    SigFox.debug();
    displayInfo();
    delay(100);
    SigFox.end();
    delay(100);
*/
    const String rep = sendAck(String("\0x01"));
    if (rep.length() > 0) {
      byte buffer[8];
      rep.getBytes(buffer, 8);
      epoch = 0;
      for(byte i = 0; i < 8; ++i) {
        epoch = (epoch << 8) + buffer[i];
      }
      rtc.setEpoch(epoch);
    }

    return true;
  }
  
/**
 * @brief Boucle répétée continuement et appelée par le framework Arduino.
 * 
 * @return L'état de l'exécussion d'une itération de la boucle.
 */
  bool loop() {
    if (millis() % 1000 > 0) return true;
    ++epoch;
    
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
  
    if (epoch % (60 * 1) == 0) {
      const String mess = "\0x02" + static_cast<char>((mediane / 256)) + static_cast<char>((mediane % 256));
      send(mess);
    }

    return true;
  }
};

 

