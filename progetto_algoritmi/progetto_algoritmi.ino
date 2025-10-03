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
    e = new Entity(SLOT1, SLOT2, PORT_8, 0x69, 80, 1);
}

// ======= Loop =======
void loop() {
  e->actions();
}
