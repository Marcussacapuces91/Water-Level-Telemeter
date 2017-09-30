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
  const byte triggerPin, echoPin;

public:
/**
 * @brief Contructeur qui permet de préciser les ports utilisés par le télémètre.
 * 
 * @param aTrigger Port [out] utilisé pour déclencher une mesure.
 * @param aEcho Port [in] qui donne la signal de la mesure (durée proportionnelle à la distance mesurée).
 */
  Telemeter(const byte aTrigger, const byte aEcho) : 
    triggerPin(aTrigger), 
    echoPin(aEcho) {
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);  
  }

/**
 * @brief Effectue une mesure et retourne la distance en mm.
 * 
 * @return La distance mesurée en mm ou 0 en cas de mesure erronnée.
 */
  unsigned mesure() const {
    digitalWrite(triggerPin, HIGH);   // sets the LED on
    delayMicroseconds(15);            // waits >10 µs
    digitalWrite(triggerPin, LOW);    // sets the LED off
    const unsigned long duree = pulseIn(echoPin, HIGH, 60 * 1000); // Count | waits <60ms
    return 17 * duree / 100;
  }
};


