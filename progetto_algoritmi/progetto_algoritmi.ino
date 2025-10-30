#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
// Librerie personalizzate
#include "LinAlg.h"
#include "Algo.h"
#include "Entity.h"

// ======= Oggetto globale ======
Entity* e;

// ======= Setup =======
void setup() {
    Serial.begin(31250);

    // Inizializzazione dell’entità con i parametri hardware
    e = new Entity(SLOT1, SLOT2, PORT_10, 0x69, 150, 1, 1);
}

// ======= Loop =======
void loop() {
  //Serial.println(e->get_Z());
  //e->actions();

  e->move_to(STRAIGHT,0,10);
  //e->turn_at(89);
  //e->delay(10);
  
  //e->move_to(STRAIGHT, 0, 3);
  //e->delay(5);
  //e->delay(5);
}

//150pwm - 3 sec
//80.5 cm / 3sec -> 26.6 cm/s
//da 12cm a 92cm
//da 12cm a 93cm
//da 12cm a 93cm
//da 12cm a 94cm
//da 12cm a 93.5
//da 12cm a 93cm
//da 12cm a 95cm
//da 12cm a 92.5cm
//da 12cm a 93.5cm
