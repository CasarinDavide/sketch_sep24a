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
    e = new Entity(SLOT1, SLOT2, PORT_10, 0x69, 180, 1);
    e->set_to_zeroZ();
}

// ======= Loop =======
void loop() {
  //Serial.println(e->get_Z());
  //e->turn_at(360-45);
  //e->delay(5);
    e->turn_at(360);
}
