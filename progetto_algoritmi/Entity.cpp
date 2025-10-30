#include "Entity.h"




#define MAX_SPEED 15.0  // giri/s — taralo in base al tuo robot


Entity* Entity::instance = nullptr;

inline double pwm_to_speed(double pwm_value) {
  return (pwm_value / 255.0) * MAX_SPEED;
}

#define SPEED(id) ((id) == 1 ? 25.5 : (id) == 2 ? 20.5 \
                                    : (id) == 3 ? 15.0 \
                                                : 10.0)

Entity::Entity() {}

Entity::Entity(uint16_t enc_l_port, uint16_t enc_r_port, uint16_t ultra_port,
               uint16_t gyro_p, uint16_t _vel, uint16_t _pwm, uint16_t K, uint16_t _id)
  : encoder_left(enc_l_port),
    encoder_right(enc_r_port),
    ultra(ultra_port),
    gyro(0, gyro_p),
    vel(_vel),
    pwm(_pwm),
    encoder_left_port(enc_l_port),
    encoder_right_port(enc_r_port),
    ultra_port(ultra_port),
    gyro_port(gyro_p),
    internal_map(),
    K(K),
    id(_id) {
  instance = this;
  // === Configurazione timer PWM ===
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  // === Inizializzazione sensori ===
  gyro.begin();
  
  encoder_left.setPosPid(1.8,0,1.2);
  encoder_right.setPosPid(1.8,0,1.2);
  encoder_left.setSpeedPid(0.18,0,0);
  encoder_right.setSpeedPid(0.18,0,0);

  // === Collegamento interrupt encoder ===

  attachInterrupt(encoder_left.getIntNum(), isr_encoder_left, RISING);
  attachInterrupt(encoder_right.getIntNum(), isr_encoder_right, RISING);

  // === Stati iniziali ===
  last_state = NULL;
  internal_state = SCAN;
  min_radius = 50;
  tollerance = 1.75;  // Tolleranza angolare
}


void Entity::move_at_coord(const Vector2D& v) {

  Serial.print("MI STO PROIETTANDO SULLA RETTA");
  double speed = SPEED(this->id);

  this->turn_at(v.get_vdegree());

  double seconds = v.get_vnorm() / speed;
  Serial.println(seconds);
  this->move_to(STRAIGHT, 0, seconds);
}


static void Entity::isr_encoder_left() {
  if(digitalRead(instance->encoder_left.getPortB()) == 0)
  {
    instance->encoder_left.pulsePosMinus();
  }
  else
  {
    instance->encoder_left.pulsePosPlus();
  }
}

static void Entity::isr_encoder_right() {
  if(digitalRead(instance->encoder_right.getPortB()) == 0)
  {
    instance->encoder_right.pulsePosMinus();
  }
  else
  {
    instance->encoder_right.pulsePosPlus();
  }

}

double Entity::get_velocity(WheelSide wheel) {
  encoder_left.updateSpeed();
  encoder_right.updateSpeed();
  
  return wheel == LEFT_WHEEL ? encoder_left.getCurrentSpeed() : encoder_right.getCurrentSpeed(); 
}

void Entity::actions() {
  encoder_left.runSpeed(0);
  encoder_right.runSpeed(0);
  encoder_left.loop();
  encoder_right.loop();

  if (internal_state == SLEEP) return;

  if (internal_state == SCAN) {
    scan(16);
    this->internal_map.serial_print();

    // Filtraggio algoritmo
    aggregate_cluster();

    // Set min distance to keep
    filter_cluster(110);

    Serial.println("-------- CLUSTER FILTRATO DOPO FUNZIONE --------");
    internal_map.serial_print();
    Serial.println("------------");

    // Calcolo baricentro e retta
    if (internal_map.size() == 2) {
      triangle = {};
      for (int i = 0; i < internal_map.size(); ++i) {
        Serial.print("--- INTERNAL MAP ----");
        Serial.println(i);
        Serial.print("FIRST:");
        Serial.println(internal_map[i].first);
        Serial.print("SECOND:");
        Serial.println(internal_map[i].second);
        Serial.print("--- END ----");
        triangle.push_back(Vector2D(internal_map[i].second, internal_map[i].first, 0));
      }

      triangle.push_back(Vector2D(0, 0));

      Serial.println("----- CENTER OF GRAVITY ------");
      center_gravity = get_avg_center(triangle);
    } else {
      Serial.println("---- FOUND MORE THAN 2 MOBILE ROBOTS ----");
    }

    Vector2D distance_1_vector = triangle[0];
    Vector2D distance_2_vector = triangle[1];
    Vector2D distance_3_vector = distance_between_vectors(triangle[0], triangle[1]);

    double distance_1 = distance_1_vector.get_vnorm();
    double distance_2 = distance_2_vector.get_vnorm();
    double distance_3 = distance_3_vector.get_vnorm();


    Serial.println("DISTANCE 1:");
    Serial.print(distance_1);
    Serial.println("DISTANCE 2:");
    Serial.print(distance_2);
    Serial.println("DISTANCE 3:");
    Serial.print(distance_3);

    if (close_to(distance_1, distance_2, 10) && close_to(distance_2, distance_3, 10)) {
      // Tutte le distanze sono simili → triangolo equilatero
      //random_walk() //TODO
    }

    if (last_state != GAT) {
      Serial.println("----- G ------");
      set_state(GAT);
    } else {
      // Robot già raccolti → elezione leader

      // l'abbiamo forzato ricordati !!
      //set_state(MASTER);

      if (distance_3 <= distance_2 && distance_3 <= distance_1) {

        set_state(MASTER);
        direction = distance_3_vector;

      } else {

        set_state(SLAVE);
        direction = distance_1 < distance_2 ? distance_1_vector : distance_2_vector;
      }

      Serial.println("INTERNAL STATE");
      Serial.println(internal_state == SLAVE ? "SCHIAVO" : "MASTER");
    }
  }

  if (internal_state == GAT) {
    

    double bari_angle = center_gravity.get_vdegree();

    Serial.print("POS VETTORE");
    Serial.println(center_gravity.get_x());
    Serial.println(center_gravity.get_y());
    Serial.print("-- -- POS VETTORE  -- - - ");

    Serial.print("ANGOLO BARICENTRO NON NORMALIZZATO ");
    Serial.println(bari_angle);

    Serial.print("ANGOLO BARICENTRO");
    Serial.println(bari_angle);


    turn_at(bari_angle);
    auto until_min_raius = [this]() {
      double current_dist = ultra.distanceCm();
      return current_dist <= min_radius;
    };

    move_until(until_min_raius);
    set_state(SCAN);
  }

  if (internal_state == MASTER) {

    Serial.print("SONO IL MASTER");
    Vector2D u = triangle[0];
    Vector2D v = triangle[1];
    Vector2D w = distance_between_vectors(v * 2.0, u);
    Vector2D h = distance_between_vectors(v, u);

    Serial.println("U");

    u.print_vector();
    Serial.println("V");
    v.print_vector();
    Serial.println("W");
    w.print_vector();
    Serial.println("H");
    h.print_vector();

    move_at_coord(w);


    // turn to side for starting to go straight

    LineParam st_line(direction);

    double opposite_angle_measure = st_line.evaluate(-1).get_y() > st_line.evaluate(1).get_y() ? direction.get_vdegree() + 180 : direction.get_vdegree();

    turn_at(opposite_angle_measure);

    Serial.println("OPPOSITE ANGLE");
    Serial.print(opposite_angle_measure);
    Serial.println("ST LINE");
    Serial.print(st_line.slope());

    set_state(FOLLOWING);
  }

  if (internal_state == SLAVE) {
    LineParam st_line(direction);

    double opposite_angle_measure = st_line.evaluate(-1).get_y() > st_line.evaluate(1).get_y() ? direction.get_vdegree() + 180 : direction.get_vdegree();

    turn_at(opposite_angle_measure);
    set_state(SLEEP);

    Serial.println("OPPOSITE ANGLE");
    Serial.print(opposite_angle_measure);
    set_state(FOLLOWING);
  }


  if (internal_state == FOLLOWING) {

    double first_current_dist = ultra.distanceCm();

    // SE è + inf la distanza misurata allora sarà quello piu in avanit
    if (!close_to(first_current_dist, direction.get_vnorm(), 10)) {
      move_to(STRAIGHT, 0, 5);
    } else {
      bool has_started = false;

      // busy-waiting
      while (!has_started) {
        has_started = first_current_dist < ultra.distanceCm() + 5;
      }

      /*
                2) fare funzione dove prendi in considerazione una distanza minima
                    e raggiunta la distanza minima setta i vel a 0, dopo conta quanti secondi sta in quella distanza
                    distanza da capire quale dare, possibile hint è norma di direction!!!!! :>
                     
            
            
            */
    }
  }
}

void Entity::set_to_zeroZ() {
  this->gyro.update();
  delay(1);
}

double Entity::get_Z() {

  delay(1);
  return this->gyro.getAngleZ();
}

void Entity::move_to(Directions dir, double keep_angle, double seconds) {
  double vel_left = getPwmForWheel(this->vel, dir, LEFT_WHEEL);
  double vel_right = getPwmForWheel(this->vel, dir, RIGHT_WHEEL);
  unsigned long endTime = millis() + (unsigned long)(seconds * 1000);

  while (millis() < endTime) {
    encoder_left.runSpeed(vel_right);
    encoder_right.runSpeed(vel_right);

    Serial.print("vL: ");
    Serial.println(get_velocity(LEFT_WHEEL));
    Serial.print("vR: ");
    Serial.println(get_velocity(RIGHT_WHEEL));

    encoder_left.loop();
    encoder_right.loop();
  }

  // Arresta i motori
  encoder_left.runSpeed(0);
  encoder_right.runSpeed(0);
  encoder_left.loop();
  encoder_right.loop();
}


void Entity::set_state(EntityState state) {
  last_state = internal_state;
  if (state == SCAN) internal_map = {};
  internal_state = state;
}

void Entity::filter_cluster(double min_distance) {
  vector<pair<double, double>> filtered;

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

void Entity::aggregate_cluster() {
  double eps_tol = 10;  // tolleranza distanza cluster
  vector<pair<double, double>> clusters;

  if (internal_map.size() == 0) {
    Serial.println("None Found");
    return;
  }

  int cluster_size = 1;
  pair<double, double> cluster(internal_map[0].first, internal_map[0].second);

  for (size_t i = 1; i < internal_map.size(); i++) {
    if (fabs(internal_map[i].second - internal_map[i - 1].second) <= eps_tol) {
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

  cluster.first /= cluster_size;
  cluster.second /= cluster_size;
  clusters.push_back(cluster);

  this->internal_map = clusters;
}

void Entity::delay(double seconds) {
  unsigned long endTime = millis() + seconds * 1000;
  while (millis() < endTime)
    ;
}

void Entity::scan(int sample_measurement) {
  gyro.update();

  z_axis_bf_scan = normalizeAngle(gyro.getAngleZ());

  if (z_axis_bf_scan < 0) z_axis_bf_scan += 360;

  double angle = 0;
  double delta_angle = 360 / sample_measurement;
  int i = 0;
  do {
    double distance = get_avg_distance(5);
    internal_map.push_back({ angle, distance });
    angle = angle + delta_angle;
    turn_at(delta_angle);
    // move_to(STRAIGHT,0,3);
    i++;

  } while (i < sample_measurement);
}

double Entity::get_avg_distance(int n_sample) {
  double mean_distance = 0.0;
  for (int i = 0; i < n_sample; ++i) {
    mean_distance += ultra.distanceCm();
  }
  return mean_distance / n_sample;
}

double Entity::getPwmForWheel(double vel, Directions dir, WheelSide wheel) {
  double p = vel;

  switch (dir) {
    case STRAIGHT: return (wheel == LEFT_WHEEL) ? -p : p;
    case BACK: return (wheel == LEFT_WHEEL) ? p : -p;
    case RIGHT: return p;
    case LEFT: return -p;
    case INVERT: return -p;
    default: return 0.0;
  }
}



void Entity::turn_at(double angle) {
  double base_left = getPwmForWheel(this->pwm, INVERT, LEFT_WHEEL);
  double base_right = getPwmForWheel(this->pwm, INVERT, RIGHT_WHEEL);

  this->gyro.update();
  delay(0.5);

  double start_angle = normalizeAngle((int)this->gyro.getAngleZ());
  double read_angle;
  double target_angle = angle;
  double actual_angle = start_angle;
  double error = angle;
  double mu = 0.0;                    // Media
  double sigma = sqrt(target_angle);  // Deviazione standard


  while (fabs(error) > this->tollerance) {
    this->gyro.update();

    read_angle = normalizeAngle(this->gyro.getAngleZ());

    actual_angle = fabs(read_angle - start_angle);

    error = target_angle - actual_angle;

    auto gaussian_error = [=](double K, double x) {
      double err = -K * (abs(base_left) - 100) * exp(-pow(mu - x, 2) / (2.0 * pow(sigma, 2)));
      return err;
    };

    double real_left = corrected_pwm(base_left, error, this->K, true, gaussian_error);
    double real_right = corrected_pwm(base_right, error, this->K, false, gaussian_error);

    if (abs(real_left) > 150) {
      real_left = real_left > 0 ? 150 : -150;
    }

    if (abs(real_right) > 150) {
      real_right = real_right > 0 ? 150 : -150;
    }


    if (abs(real_left) > 100) {
      real_left = real_left > 0 ? 100 : -100;
    }

    if (abs(real_right) < 100) {
      real_right = real_right > 0 ? 100 : -100;
    }

    int pwm_left = real_left;
    int pwm_right = real_right;

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



/*
void Entity::turn_at(double angle) {
  // Imposta PWM base (stesso modulo, segni opposti)
  double base_left = getPwmForWheel(this->pwm, INVERT, LEFT_WHEEL);
  double base_right = getPwmForWheel(this->pwm, INVERT, RIGHT_WHEEL);

  double pwm_left = base_left;
  double pwm_right = base_right;

  this->gyro.update();

  // PID parameters
  double Kp = 2.0;
  double Ki = 0.0;
  double Kd = 0.5;

  double integral = 0;
  double last_error = 0;

  unsigned long last_time = millis();
  
  double start_angle = normalizeAngle((int)this->gyro.getAngleZ());
  double read_angle;
  double target_angle = angle;
  double actual_angle = start_angle;
  double error = angle;
  

  while (true) {
    this->gyro.update();

    // Calcolo angolo attuale e differenza
    
    read_angle = normalizeAngle(this->gyro.getAngleZ());
    actual_angle = fabs(read_angle - start_angle);
    
    error = (target_angle - actual_angle);

    // Uscita condizione
    if (fabs(error) < this->tollerance) break;

    // Calcolo dt in secondi
    unsigned long now = millis();
    double dt = (now - last_time) / 1000.0;
    last_time = now;

    // PID sull'angolo
    integral += error * dt;
    double derivative = (error - last_error) / dt;
    last_error = error;

    double correction = Kp * error + Ki * integral + Kd * derivative;

    // Aggiorna PWM delle ruote (simmetrico)
    pwm_left = constrain(abs(base_left) + correction, 100, 150);
    pwm_right = constrain(abs(base_right) + correction, 100, 150);

    pwm_left *= sign(base_left);
    pwm_right *= sign(base_right);

    encoder_left.setMotorPwm(pwm_left);
    encoder_right.setMotorPwm(pwm_right);

    encoder_left.loop();
    encoder_right.loop();
  }

  // Stop finale

  encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();
  delay(2);
}*/


double Entity::normalizeAngle(double angle) {
  if (angle > 0) {
    angle = 360 - angle;
  } else {
    angle = fabs(angle);
  }

  return angle;
}


template<typename Func>
void Entity::move_until(Func stopping_criteria, Directions dir, double keep_angle) {
  // PWM di base per la direzione scelta
  double base_left = getPwmForWheel(this->vel, dir, LEFT_WHEEL);
  double base_right = getPwmForWheel(this->vel, dir, RIGHT_WHEEL);

  // Inizializza il giroscopio
  this->gyro.update();
  double start_angle = normalizeAngle(this->gyro.getAngleZ());
  double target_angle = normalizeAngle(start_angle + keep_angle);

  // Parametri PID per il controllo direzionale
  double Kp = 2.0;
  double Ki = 0.0;
  double Kd = 0.3;

  double integral = 0.0;
  double last_error = 0.0;
  unsigned long last_time = millis();
  double pwm_left = base_left;
  double pwm_right = base_right;
  
  while (!stopping_criteria()) {
    this->gyro.update();

    // Calcolo dt in secondi
    unsigned long now = millis();
    double dt = (now - last_time) / 1000.0;
    last_time = now;

    // Limiti PWM
    pwm_left = constrain(pwm_left, 0, 255);
    pwm_right = constrain(pwm_right, 0, 255);

    pwm_left  *= sign(base_left);
    pwm_right *= sign(base_right);
    
    // Applica PWM ai motori
    encoder_left.setMotorPwm(pwm_left);
    encoder_right.setMotorPwm(pwm_right);

    encoder_left.loop();
    encoder_right.loop();

  }

  // Arresta i motori
  encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();

  delay(3);
}


template<typename Func> double Entity::corrected_pwm(double base_pwm, double error, double K, bool isLeft, Func correction_func) 
{ double correction = correction_func(K, error); return base_pwm - correction; }
