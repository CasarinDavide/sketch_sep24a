#pragma once
#include <math.h>
#include "LinAlg.h"
#include "Algo.h"
#include <MeAuriga.h>


using namespace LinAlg;
using namespace Algo; 

// ======= Enum =======
typedef enum EntityState {
    SCAN,
    MOVE,
    SLEEP,
    GAT,
    MASTER,
    SLAVE,
    FOLLOWING,
    LEADER,
    CALIBRATION,
    HEAD,
    TAIL
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



class Entity {
public:
    // Componenti hardware
    MeEncoderOnBoard encoder_left;
    MeEncoderOnBoard encoder_right;
    MeUltrasonicSensor ultra;
    MeGyro gyro;

    // Parametri configurazione
    uint16_t vel;
    uint16_t encoder_left_port;
    uint16_t encoder_right_port;
    uint16_t ultra_port;
    uint16_t gyro_port;
    uint16_t id;

    // Stato interno
    EntityState internal_state;
    EntityState last_state;

    static Entity* instance;

    // Rappresentazione geometrica
    vector<Vector2D> triangle;
    vector<pair<double, double>> internal_map;
    Vector2D center_gravity;
    Vector2D direction;

    // Costruttori
    Entity();
    Entity(uint16_t enc_l_port,
           uint16_t enc_r_port,
           uint16_t ultra_port,
           uint16_t gyro_p);

    // Metodi principali
    void actions();
    void move_to(Directions dir, double seconds);
    void move_at_coord(const Vector2D& v);
    
    template <typename Func>
    void move_until(Func stopping_criteria, Directions dir = STRAIGHT, double keep_angle = 0);


    
    
    
    void turn_at(double angle);
    // Utilità
    void delay(double seconds);
    static void isr_encoder_left();
    static void isr_encoder_right();

    double get_velocity(WheelSide);
    void follow(double min_dist,double seconds);
    
    void scan(int sample_measurement);

    void move_to_triangle(LineParam pt,double distance, EntityState state);
    void back_to_line(LineParam pt,double distance, EntityState state);
    double get_avg_distance(int n_sample);

private:
    // Gestione stati
    void set_state(EntityState state);
    // Algoritmi di clustering e filtraggio
    void filter_cluster(double min_distance);
    void aggregate_cluster();

    

    // PWM e controllo differenziale
    double getPwmForWheel(Directions dir, WheelSide wheel);
    double getPwmForWheel(double vel, Directions dir, WheelSide wheel);
    template <typename Func>
    double corrected_pwm(double base_pwm, double error, double K, bool isLeft,Func corrector);
    double normalizeAngle(double angle);
};
