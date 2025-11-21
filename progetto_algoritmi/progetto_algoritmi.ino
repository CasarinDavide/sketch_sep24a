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
    e = new Entity(SLOT1, SLOT2, PORT_8, 0x69);
}

// ======= Loop =======
void loop() {
  //Serial.println(e->get_Z());
  e->actions();
  //e->follow(5, 10);
  
  //e->turn_at(90);
  //delay(10);

  //e->scan(16);
  //e->move_to(STRAIGHT,0,5);

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

//mBot Ranger: 38cm in 3sec, 60rpm
//mBot Raptor: 40 rpm

