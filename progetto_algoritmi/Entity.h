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

class Entity {
public:
    // Componenti hardware
    MeEncoderOnBoard encoder_left;
    MeEncoderOnBoard encoder_right;
    MeUltrasonicSensor ultra;
    MeGyro gyro;

    // Parametri configurazione
    uint16_t pwm;
    uint16_t encoder_left_port;
    uint16_t encoder_right_port;
    uint16_t ultra_port;
    uint16_t gyro_port;

    // Stato interno
    EntityState internal_state;
    EntityState last_state;

    // Parametri di controllo
    double tollerance;
    double K;
    double min_radius;

    // Rappresentazione geometrica
    vector<Vector2D> triangle;
    vector<pair<double, double>> internal_map;
    Vector2D center_gravity;

    // z_axis_bf_scan before scan
    double z_axis_bf_scan;

    // Costruttori
    Entity();
    Entity(uint16_t enc_l_port,
           uint16_t enc_r_port,
           uint16_t ultra_port,
           uint16_t gyro_p,
           uint16_t _pwm,
           uint16_t K);

    // Metodi principali
    void actions();
    void move_to(Directions dir, double keep_angle, double seconds);
    void turn_at(double angle);
    // Utilità
    void delay(double seconds);
    double get_Z();
    void set_to_zeroZ();

private:
    // Gestione stati
    void set_state(EntityState state);
    // Algoritmi di clustering e filtraggio
    void filter_cluster(double min_distance);
    void aggregate_cluster(bool include_last = false);

    
    void scan(int sample_measurement);
    double get_avg_distance(int n_sample);

    // PWM e controllo differenziale
    double getPwmForWheel(Directions dir, WheelSide wheel);
    double getPwmForWheel(double pwm, Directions dir, WheelSide wheel);
    double corrected_pwm(double base_pwm, double error, double K, bool isLeft);
    double Entity::normalizeAngle(double angle);
    template <typename Func>
    void move_until(Func stopping_criteria, Directions dir = STRAIGHT, double keep_angle = 0);
};
