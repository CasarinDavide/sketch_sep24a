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
    LEADER
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
    uint16_t id;

    // Stato interno
    EntityState internal_state;
    EntityState last_state;

    static Entity* instance;

    // Parametri di controllo
    double tollerance;
    double K;
    double min_radius;
    // Velocità sinistra
    double velocity_error_integral_L = 0;
    double last_error_velocity_L = 0;

    // Velocità destra
    double velocity_error_integral_R = 0;
    double last_error_velocity_R = 0;


    // Rappresentazione geometrica
    vector<Vector2D> triangle;
    vector<pair<double, double>> internal_map;
    Vector2D center_gravity;
    Vector2D direction;

    // z_axis_bf_scan before scan
    double z_axis_bf_scan;

    // Costruttori
    Entity();
    Entity(uint16_t enc_l_port,
           uint16_t enc_r_port,
           uint16_t ultra_port,
           uint16_t gyro_p,
           uint16_t _pwm,
           uint16_t K,
           uint16_t _id);

    // Metodi principali
    void actions();
    void move_to(Directions dir, double keep_angle, double seconds);
    void move_at_coord(const Vector2D& v);
    void turn_at(double angle);
    // Utilità
    void delay(double seconds);
    double get_Z();
    void set_to_zeroZ();
    static void isr_encoder_left_A();
    static void isr_encoder_left_B();
    static void isr_encoder_right_A();
    static void isr_encoder_right_B();

    double get_velocity(WheelSide);


private:
    // Gestione stati
    void set_state(EntityState state);
    // Algoritmi di clustering e filtraggio
    void filter_cluster(double min_distance);
    void aggregate_cluster();

    
    void scan(int sample_measurement);
    double get_avg_distance(int n_sample);

    // PWM e controllo differenziale
    double getPwmForWheel(Directions dir, WheelSide wheel);
    double getPwmForWheel(double pwm, Directions dir, WheelSide wheel);
    template <typename Func>
    double corrected_pwm(double base_pwm, double error, double K, bool isLeft,Func corrector);
    double normalizeAngle(double angle);
    template <typename Func>
    void move_until(Func stopping_criteria, Directions dir = STRAIGHT, double keep_angle = 0);
    //void correct_pwm_velocity_target(double& pwm_left, double& pwm_right,
    //                                      double dt,
    //                                      double target_velocity = 30.0,
    //                                      double Kp = 0.3, double Ki = 0.1, double Kd = 0.05);

    //void reset_velocity_params();
};
