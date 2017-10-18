/** 
 *  @file    telemeter.h 
 *  @author  Marc Sibert
 *  @date    09/2017  
 *  @version 1.0 
 *  
 *  @brief Class Telemeter - Instancie les méthodes d'accès au télémètre.
 *
 *  @section DESCRIPTION
 *  
 *  
 *
 */
 
#pragma once

/**
  * @brief Classe qui instancie les méthodes de contrôle du télémètre 
  */  
class Telemeter {
private:
  const byte triggerPin, echoPin, enablePin;

public:
/**
 * @brief Contructeur qui permet de préciser les ports utilisés par le télémètre.
 * 
 * @param aTrigger Port [out] utilisé pour déclencher une mesure.
 * @param aEcho Port [in] qui donne la signal de la mesure (durée proportionnelle à la distance mesurée).
 * @param aEnable Port [inv out] Commande l'alimentation du convertisseur 3v -> 5v.
 */
  Telemeter(const byte aTrigger, const byte aEcho, const byte aEnable) : 
    triggerPin(aTrigger), 
    echoPin(aEcho),
    enablePin(aEnable) {
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);  
    pinMode(enablePin, OUTPUT);  
    digitalWrite(enablePin, LOW);  // start power converter
  }

/**
 * @brief Effectue une mesure et retourne la distance en mm.
 * 
 * @return La distance mesurée en mm ou 0 en cas de mesure erronnée.
 */
  unsigned mesure() const {
    for (byte i = 0; i < 3; ++i) {    // try 3 times
      digitalWrite(triggerPin, HIGH);
      delayMicroseconds(15);                // waits >10 µs
      digitalWrite(triggerPin, LOW); 
      const unsigned long duree = pulseIn(echoPin, HIGH, 60 * 1000); // Count | waits <60ms
      if (duree) return 17 * duree / 100;   // return first not null value
    }
    return 0;
  }
};


