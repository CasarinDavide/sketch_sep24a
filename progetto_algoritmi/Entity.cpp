#include "Entity.h"

Entity::Entity() {}

Entity::Entity(uint16_t enc_l_port, uint16_t enc_r_port, uint16_t ultra_port, uint16_t gyro_p, uint16_t _pwm, uint16_t K)
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
      K(K) 
{
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

void Entity::actions() {
    encoder_left.setMotorPwm(0);
    encoder_right.setMotorPwm(0);
    encoder_left.loop();
    encoder_right.loop();

    if (internal_state == SLEEP) return;

    if (internal_state == SCAN) {    
        scan(8);
        this->internal_map.serial_print();

        // Filtraggio algoritmo
        aggregate_cluster(true);

        // Set min distance to keep 
        filter_cluster(80);

        Serial.println("-------- CLUSTER FILTRATO DOPO FUNZIONE --------");
        internal_map.serial_print();
        Serial.println("------------");

        // Calcolo baricentro e retta
        if (internal_map.size() == 2) {
            triangle = {};
            for (int i = 0; i < internal_map.size(); ++i) {
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
        } else {
            Serial.println("---- FOUND MORE THAN 2 MOBILE ROBOTS ----");
        }

        double distance_1 = triangle[0].get_vnorm();
        double distance_2 = triangle[2].get_vnorm();
        double distance_3 = triangle[3].get_vnorm();

        if (close_to(distance_1, distance_2, 10) && close_to(distance_2, distance_3, 10)) {
            // Tutte le distanze sono simili → triangolo equilatero
            // Serve rompere la simmetria (random walk?)
        }

        if (last_state != GAT) {
            set_state(GAT);
        } else {
            // Robot già raccolti → elezione leader
            if (distance_3 <= distance_2 && distance_3 <= distance_1) {
                set_state(MASTER);
            } else {
                set_state(SLAVE);
            }

            Serial.println("INTERNAL STATE");
            Serial.println(internal_state == SLAVE ? "SCHIAVO" : "MASTER");
        }
    }

    if (internal_state == GAT) {
        turn_at(this->gyro.getAngleZ() + center_gravity.get_vdegree());
        // move_until(min_radius);
        set_state(SCAN);
    }

    if (internal_state == MASTER) {
        LineParam line(triangle[1], triangle[2]);
        // move_at_coord(new_x, line.evaluate(new_x));
    }
}

void Entity::move_to(Directions dir, double keep_angle, double seconds) {
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

        // Correzione differenziale
        double real_left  = corrected_pwm(base_left, error, this->K, true);
        double real_right = corrected_pwm(base_right, error, this->K, false);

        // Bounding PWM
        int pwm_left  = constrain(real_left,  -255.0f, 255.0f);
        int pwm_right = constrain(real_right, -255.0f, 255.0f);

        encoder_left.setMotorPwm(pwm_left);
        encoder_right.setMotorPwm(pwm_right);

        encoder_left.loop();
        encoder_right.loop(); 
    }
    
    // Stop motori
    encoder_left.setMotorPwm(0);
    encoder_right.setMotorPwm(0);
    encoder_left.loop();
    encoder_right.loop();

    delay(3);
}

void Entity::set_state(EntityState state) {
    last_state = internal_state;
    internal_state = state;
}

void Entity::filter_cluster(double min_distance) {
    vector<pair<double,double>> filtered;

    Serial.println("-------- CLUSTER PRIMA --------");
    internal_map.serial_print();
    Serial.println("------------");

    for (size_t i = 0; i < this->internal_map.size(); ++i) {
        if (this->internal_map[i].second <= min_distance) {
            filtered.push_back(this->internal_map[i]);
        }
    }

    Serial.println("-------- CLUSTER FILTRATO --------");
    filtered.serial_print();
    Serial.println("------------");

    this->internal_map = filtered;
}

void Entity::aggregate_cluster(bool include_last) {
    double eps_tol = 5; // tolleranza distanza cluster
    vector<pair<double,double>> clusters;

    int cluster_size = 0;
    pair<double,double> cluster(0.0, 0.0);

    if (internal_map.size() < 2) {
        this->internal_map = clusters;
        return;
    }

    for (size_t i = 0; i < internal_map.size() - 1; i++) {
        if (cluster_size == 0 || fabs(internal_map[i].second - internal_map[i+1].second) <= eps_tol) {
            cluster.first += internal_map[i].first;
            cluster.second += internal_map[i].second;
            ++cluster_size;
        } else {
            // Salva cluster
            cluster.first /= cluster_size;
            cluster.second /= cluster_size;
            clusters.push_back(cluster);

            // Reset cluster
            cluster = { internal_map[i].first, internal_map[i].second };
            cluster_size = 1;
        }
    }

    if (cluster_size > 0 && include_last) {
        cluster.first /= cluster_size;
        cluster.second /= cluster_size;
        clusters.push_back(cluster);
    }

    this->internal_map = clusters;
}

void Entity::delay(double seconds) {
    unsigned long endTime = millis() + seconds * 1000;
    while (millis() < endTime);
}

void Entity::scan(int sample_measurement) {
    gyro.update();

    Serial.read();
    int angle = 0;
    double delta_angle = 360 / sample_measurement;

    for (int i = 0; i < sample_measurement; ++i) {
        angle = (int)(angle + delta_angle) % 360;

        turn_at(delta_angle);
        // move_to(STRAIGHT,0,3);
        delay(0.5);

        double distance = get_avg_distance(1);
        internal_map.push_back({angle, distance});
    }
}

double Entity::get_avg_distance(int n_sample) {
    double mean_distance = 0.0;
    for (int i = 0; i < n_sample; ++i) {
        mean_distance += ultra.distanceCm();
    }
    return mean_distance / n_sample;
}

double Entity::getPwmForWheel(Directions dir, WheelSide wheel) {
    double p = (double)this->pwm;

    switch (dir) {
        case STRAIGHT: return (wheel == LEFT_WHEEL) ? -p : p;
        case BACK:     return (wheel == LEFT_WHEEL) ?  p : -p;
        case RIGHT:    return p;   // rotazione destra
        case LEFT:     return -p;  // rotazione sinistra
        case INVERT:   return p;
        default:       return 0.0;
    }
}

double Entity::getPwmForWheel(double pwm, Directions dir, WheelSide wheel) {
    double p = pwm;

    switch (dir) {
        case STRAIGHT: return (wheel == LEFT_WHEEL) ? -p : p;
        case BACK:     return (wheel == LEFT_WHEEL) ?  p : -p;
        case RIGHT:    return p;
        case LEFT:     return -p;
        case INVERT:   return p;
        default:       return 0.0;
    }
}

double Entity::corrected_pwm(double base_pwm, double error, double K, bool isLeft) {
    double correction = K * error;
    return base_pwm + correction;
}

void Entity::turn_at(double angle) {
    double base_left  = getPwmForWheel(this->pwm, INVERT, LEFT_WHEEL);
    double base_right = getPwmForWheel(this->pwm, INVERT, RIGHT_WHEEL);

    this->gyro.update();
    delay(0.5);

    double start_angle = normalizeAngle(this->gyro.getAngleZ());
    double read_angle;
    double actual_angle = start_angle;
    double target_angle = angle; 
    double error = angle;

    while (fabs(error) > this->tollerance) {
        this->gyro.update();
        read_angle = this->gyro.getAngleZ();

        if (read_angle < 0 && actual_angle > 0 && abs(read_angle) + actual_angle > 180) {
            read_angle = 360 + read_angle;
        }

        actual_angle = abs(read_angle - start_angle);
        error = target_angle - actual_angle;

        double real_left  = corrected_pwm(base_left,  error, this->K, true);
        double real_right = corrected_pwm(base_right, error, this->K, false);

        if (error < 0) {
            real_left  = -real_left;
            real_right = -real_right;
        }

        int pwm_left  = constrain(real_left,  -base_left - 40, base_left + 40);
        int pwm_right = constrain(real_right, -base_right - 40, base_right + 40);

        encoder_left.setMotorPwm(pwm_left);
        encoder_right.setMotorPwm(pwm_right);

        encoder_left.loop();
        encoder_right.loop();
    }
    
    encoder_left.setMotorPwm(0);
    encoder_right.setMotorPwm(0);
    encoder_left.loop();
    encoder_right.loop();

    delay(3);
}

double Entity::normalizeAngle(double angle) {
    while (angle < -180) angle += 360;
    while (angle > 180)  angle -= 360;
    return angle;
}

