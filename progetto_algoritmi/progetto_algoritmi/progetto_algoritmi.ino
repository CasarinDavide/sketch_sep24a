#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <MeAuriga.h>
#include <math.h>
#include "LinAlg.h"
#include "Algo.h"

using namespace std;
using namespace LinAlg;
using namespace Algo;



// ======= Enum =======
typedef enum EntityState {
    SCAN,
    MOVE,
    SLEEP,
    GAT,
    MASTER,
    SLAVE
} EntityState;

typedef enum Directions {
    STRAIGHT,
    RIGHT,
    LEFT,
    BACK,
    INVERT
} Directions;

typedef enum Wheel {
    RIGHT_WHEEL,
    LEFT_WHEEL
} WheelSide;

// ======= Classe Entity =======
class Entity {
public:
    MeEncoderOnBoard encoder_left;
    MeEncoderOnBoard encoder_right;
    MeUltrasonicSensor ultra;
    MeGyro gyro;

    uint16_t pwm;
    uint16_t encoder_left_port, encoder_right_port, ultra_port, gyro_port;

    EntityState internal_state;
    EntityState last_state;
    
    double tollerance;
    double K;
    double min_radius;
    vector<Vector2D> triangle;

    vector<pair<double, double>> internal_map;
    Vector2D center_gravity;

    // Costruttore
    Entity() {}

    Entity(uint16_t enc_l_port, uint16_t enc_r_port, uint16_t ultra_port, uint16_t gyro_p, uint16_t _pwm,uint16_t K)
        : encoder_left(enc_l_port),
          encoder_right(enc_r_port),
          ultra(ultra_port),
          gyro(0, gyro_p),
          pwm(_pwm),
          encoder_left_port(enc_l_port),
          encoder_right_port(enc_r_port),
          ultra_port(ultra_port),
          gyro_port(gyro_p),
          internal_map(),
          K(K) {
        
        // Configurazione timer PWM
        TCCR1A = _BV(WGM10);
        TCCR1B = _BV(CS11) | _BV(WGM12);
        TCCR2A = _BV(WGM21) | _BV(WGM20);
        TCCR2B = _BV(CS21);
        gyro.begin();
        internal_state = SCAN;
        min_radius = 20;
        tollerance = 2; // angle tollerance
    }



    void actions() {

        encoder_left.setMotorPwm(0);
        encoder_right.setMotorPwm(0);
        encoder_left.loop();
        encoder_right.loop();

        if (internal_state == SLEEP) return;
        if (internal_state == SCAN) 
        {    
            scan(8);
            this->internal_map.serial_print();
            
           
            // filtraggio algoritmo
            aggregate_cluster(true);

            // set min distance to keep 
            filter_cluster(80);


            Serial.println("-------- CLUSTER FILTRATO DOPO FUNZIONE --------");
            internal_map.serial_print();
            Serial.println("------------");
            
             // TODO: aggiungi qui se bluethooth mathing simmetrico
        
            // calcolo baricentro e retta

            // assuming in only robot in internal map

            if(internal_map.size()==2)
            {
                triangle = {};
                for(int i = 0; i < internal_map.size(); ++i)
                {
                    // passo norma + angolo al e creo il vettore associato

                    Serial.print("--- INTERNAL MAP ----");
                    Serial.println(i);
                    Serial.print("SECOND:");
                    Serial.println(internal_map[i].first);
                    Serial.print("FIRST:");
                    Serial.println(internal_map[i].second);
                    Serial.print("--- END ----");
                    
                    triangle.push_back(Vector2D(internal_map[i].second, internal_map[i].first, 0));
                }

                triangle.push_back(distance_between_vectors(triangle[0], triangle[1]));
                Serial.println("----- CENTER OF GRAVITY ------");
                Vector2D center_gravity = get_avg_center(triangle);
            }
            else
            {
                Serial.println("---- FOUUND MORE THAN 2 MOBILE ROBOTS ");
            }
            
            double distance_1 = triangle[0].get_vnorm();
            double distance_2 = triangle[2].get_vnorm();
            double distance_3 = triangle[3].get_vnorm();
                    
            if(close_to(distance_1,distance_2,10) && close_to(distance_2,distance_3,10))
            {
                // all distance are closed 
                // we have an equilateral triangle
                // we need to break the symmetry 
                // so we can take for a random walk
                    
            }

            if(last_state != GAT)
                set_state(GAT);
            else
            {
                // robots are already gathered

                // electing leader
                // if opposite distance is the lowest then im the farest than other robots
                if(distance_3 <= distance_2 && distance_3 <= distance_1)
                    set_state(MASTER);
                else
                    set_state(SLAVE);

                Serial.println("INTERNAL STATE");
                Serial.println(internal_state == SLAVE?"SCHIAVO":"MASTER");
                
            }

                        
         }

        if(internal_state == GAT)
        {
            turn_at(this->gyro.getAngleZ() + center_gravity.get_vdegree());
            //move_until(min_radius);
            
            set_state(SCAN);
        }

        if(internal_state == MASTER)
        {
            LineParam line(triangle[1],triangle[2]);
            //move_at_coord(new_x,line.evaluate(new_x));
        }
        else
        {

        }
    }

private:

    void set_state(EntityState state)
    {
        last_state = internal_state;
        internal_state = state;
    }
    
    void filter_cluster(double min_distance)
    {
        vector<pair<double,double>> filtered;

        Serial.println("-------- CLUSTER PRIMA --------");
        internal_map.serial_print();
        Serial.println("------------");

        for(size_t i = 0; i  < this->internal_map.size(); ++i)
        {
            if(this->internal_map[i].second <= min_distance)
            {
                filtered.push_back(this->internal_map[i]);
            }
        }

        Serial.println("-------- CLUSTER FILTRATO --------");
        filtered.serial_print();
        Serial.println("------------");

        this->internal_map = filtered;
    }

    void aggregate_cluster(bool include_last = false)
    {
        double eps_tol = 5; // tolleranza di distanza dai gruppi

        vector<pair<double,double>> clusters;

        int cluster_size = 0;
        pair<double,double> cluster(0.0, 0.0);

        if (internal_map.size() < 2) {
            this->internal_map = clusters;
            return;
        }

        for (size_t i = 0; i < internal_map.size()-1; i++)
        {
            // Se il cluster è vuoto oppure l'elemento corrente è "vicino" al precedente
            if (cluster_size == 0 ||
                (fabs(internal_map[i].second - internal_map[i+1].second) <= eps_tol))
            {
                cluster.first += internal_map[i].first;
                cluster.second += internal_map[i].second;
                ++cluster_size;
            }
            else
            {
                // salva cluster precedente
                cluster.first /= cluster_size;
                cluster.second /= cluster_size;
                clusters.push_back(cluster);

                // reset cluster
                cluster = { internal_map[i].first, internal_map[i].second };
                cluster_size = 1;
            }
        }

        // salva ultimo cluster
        if (cluster_size > 0 && include_last) {
            cluster.first /= cluster_size;
            cluster.second /= cluster_size;
            clusters.push_back(cluster);
        }

        this->internal_map = clusters;
    }


    void delay(double seconds) {
        unsigned long endTime = millis() + seconds * 1000;
        while (millis() < endTime);
    }

    void moveDuration(double seconds) {
        if (seconds < 0.0) seconds = 0.0;

        unsigned long endTime = millis() + seconds * 1000;
        while (millis() < endTime) {
            encoder_left.loop();
            encoder_right.loop();
        }
    }

    void scan(int sample_measurement) {
        gyro.update();

        Serial.read();
        int angle = 0;
        double delta_angle = 360/sample_measurement;

        for (int i = 0; i < sample_measurement; ++i) {
            
            angle = (int)(angle + delta_angle) % 360;

            turn_at(delta_angle);
            //move_to(STRAIGHT,0,3);
            delay(0.5);
            // Esempio di mappatura (commentato)
            double distance = get_distance_mean(1);
            internal_map.push_back({angle, distance});
        }
    }

    double get_distance_mean(int n_sample) {
        double mean_distance = 0.0;
        for (int i = 0; i < n_sample; ++i) {
            mean_distance += ultra.distanceCm();
        }
        return mean_distance / n_sample;
    }

    double normalizeAngle(double angle) {
        while (angle < -180) angle += 360;
        while (angle > 180)  angle -= 360;
        return angle;
    }

    double getPwmForWheel(Directions dir, WheelSide wheel) {
        double p = (double)this->pwm;

        switch (dir) {
            case STRAIGHT:
                return (wheel == LEFT_WHEEL) ? -p :  p;
            case BACK:
                return (wheel == LEFT_WHEEL) ? p :  -p;
            case RIGHT: // rotazione su posto a destra
                return (wheel ==  RIGHT_WHEEL) ? p :  p;
            case LEFT:  // rotazione su posto a sinistra
                return (wheel == RIGHT_WHEEL) ?  -p : -p;
            case INVERT:
                return p;
            default:
                return 0.0;
        }
    }


     double getPwmForWheel(double pwm,Directions dir, WheelSide wheel) {
        double p = pwm;

        switch (dir) {
            case STRAIGHT:
                return (wheel == LEFT_WHEEL) ? -p :  p;
            case BACK:
                return (wheel == LEFT_WHEEL) ? p :  -p;
            case RIGHT: // rotazione su posto a destra
                return (wheel ==  RIGHT_WHEEL) ? p :  p;
            case LEFT:  // rotazione su posto a sinistra
                return (wheel == RIGHT_WHEEL) ?  -p : -p;
            case INVERT:
                return p;
            default:
                return 0.0;
        }
    }


    // corrected_pwm differenziale: isLeft true se è la ruota sinistra
    // utilizzo back forward con errore simmetrico
    double corrected_pwm(double base_pwm, double error, double K, bool isLeft) {
        // correttore lineare

        double correction = K * error;
        // alla ruota sinistra sottrai, alla destra aggiungi (differenziale)
        
        return base_pwm + correction;
    }

    void move_to(Directions dir, double keep_angle, double seconds) {

        double base_left  = getPwmForWheel(dir, LEFT_WHEEL);
        double base_right = getPwmForWheel(dir, RIGHT_WHEEL);
        

        this->gyro.update();
        delay(1);
        double actual_angle = normalizeAngle(this->gyro.getAngleZ());
        double target_angle = actual_angle + keep_angle; 

        unsigned long endTime = millis() + seconds * 1000;
        double error = target_angle - actual_angle;

        while (millis() < endTime) {
            this->gyro.update();
            delay(0.2);
            actual_angle = this->gyro.getAngleZ();
            error = target_angle - actual_angle;

            // applica correzione differenziale
            double real_left  = corrected_pwm(base_left, error, this->K , true);
            double real_right = corrected_pwm(base_right, error, this->K, false);
            
            // bounding dei valori negli intervalli limite

            int pwm_left  = constrain(real_left,  -255.0f, 255.0f);
            int pwm_right = constrain(real_right, -255.0f, 255.0f);

            encoder_left.setMotorPwm(pwm_left);
            encoder_right.setMotorPwm(pwm_right);

            encoder_left.loop();
            encoder_right.loop(); 

        }
        
        //ferma i motori al termine
        encoder_left.setMotorPwm(0);
        encoder_right.setMotorPwm(0);
        encoder_left.loop();
        encoder_right.loop();

        delay(3);
    }


    void turn_at(double angle) {

        double base_left  = getPwmForWheel(this->pwm,INVERT, LEFT_WHEEL);
        double base_right = getPwmForWheel(this->pwm,INVERT, RIGHT_WHEEL);

        this->gyro.update();
        
        delay(0.5);
        double start_angle = normalizeAngle(this->gyro.getAngleZ());
        double read_angle;
        double actual_angle = start_angle;
        double target_angle = angle; 
        
        
        double error = angle;

        while(fabs(error) > this->tollerance)
        {
            this->gyro.update();
            //delay(0.2);
            
            read_angle = this->gyro.getAngleZ();

            if(read_angle < 0 && actual_angle > 0 && abs(read_angle) + actual_angle > 180)
            {
                read_angle = 360 + read_angle;
            }

            actual_angle = abs(read_angle - start_angle);

            //this->stop();
            //return;

            error = target_angle - actual_angle;


            // applica correzione differenziale
            double real_left  = corrected_pwm(base_left,  error, this->K, true);
            double real_right = corrected_pwm(base_right, error, this->K, false);

            if(error < 0)
            {
                real_left = -real_left;
                real_right = -real_right;
            }


            // bounding dei valori negli intervalli limite

            int pwm_left  = constrain(real_left,  -base_left - 40, base_left + 40);
            int pwm_right = constrain(real_right, -base_right - 40, base_right + 40);


            encoder_left.setMotorPwm(pwm_left);
            encoder_right.setMotorPwm(pwm_right);

            encoder_left.loop();
            encoder_right.loop();
        }
        
        //ferma i motori al termine
        encoder_left.setMotorPwm(0);
        encoder_right.setMotorPwm(0);
        encoder_left.loop();
        encoder_right.loop();

        delay(3);
            
    }
};

// ======= Oggetto globale ======

Entity* e;

// ======= Setup e Loop =======
void setup() {
  Serial.begin(31250);
  e = new Entity(SLOT1, SLOT2, PORT_8, 0x69, 80,1);
}

void loop() {
  e->actions();
}
