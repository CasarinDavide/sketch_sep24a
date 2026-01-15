#include "Entity.h"
#define SPEED 13.0
#define ID 1
#define MIN_RADIUS 25
#define ANGULAR_TOL 0.5
#define CLUSTER_DIST 85
#define CLUSTER_AGGR_DIST 10
#define SCAN_NUMBER 25
#define CLOSE_TO_ERROR 100
#define DELAY_ERROR 1.5
#define MOVE_LINE_SECONDS 3
#define MOVE_TRIANGLE_SECONDS 3
#define DISTANCE_BETWEEN_ROBOTS 10
#define DISTANCE_BETWEEN_ROBOTS_ERROR 10
#define SCAN_SAMPLE_NUMBER 10
#define MAX_DELAY 10
#define DUMMY_FUNCTION_INTERVAL \
  { 15, target_angle }
#define INITIAL_STATE CALIBRATION


// Macro per configurazione
#if ID == 1
#define MAX_VEL 80
#define MIN_VEL 20
#elif ID == 2
#define MAX_VEL 40
#define MIN_VEL 15
#endif


double dummy_function(double error, double interval[], double a = 2, double max_velocity = MAX_VEL, double min_velocity = MIN_VEL) {
  if (error > interval[1]) {
    return max_velocity;
  } else if (error < interval[0]) {
    return min_velocity;
  } else {
    return constrain(a * error, min_velocity, max_velocity);
  }
}

/* ------------------- */

Entity* Entity::instance = nullptr;

Entity::Entity() {}

Entity::Entity(uint16_t enc_l_port, uint16_t enc_r_port, uint16_t ultra_port, uint16_t gyro_p)
  : encoder_left(enc_l_port),
    encoder_right(enc_r_port),
    ultra(ultra_port),
    gyro(0, gyro_p),
    vel(MAX_VEL),
    encoder_left_port(enc_l_port),
    encoder_right_port(enc_r_port),
    ultra_port(ultra_port),
    gyro_port(gyro_p),
    internal_map() {
  instance = this;
  // === Configurazione timer PWM ===
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  // === Inizializzazione sensori ===
  gyro.begin();

  encoder_left.setSpeedPid(0.18, 0, 0);
  encoder_right.setSpeedPid(0.18, 0, 0);

  encoder_left.runSpeed(0);
  encoder_right.runSpeed(0);
  encoder_left.loop();
  encoder_right.loop();

  // === Collegamento interrupt encoder ===

  attachInterrupt(encoder_left.getIntNum(), isr_encoder_left, RISING);
  attachInterrupt(encoder_right.getIntNum(), isr_encoder_right, RISING);

  // === Stati iniziali ===
  last_state = NULL;
  internal_state = INITIAL_STATE;
}




static void Entity::isr_encoder_left() {
  if (digitalRead(instance->encoder_left.getPortB()) == 0) {
    instance->encoder_left.pulsePosMinus();
  } else {
    instance->encoder_left.pulsePosPlus();
  }
}

static void Entity::isr_encoder_right() {
  if (digitalRead(instance->encoder_right.getPortB()) == 0) {
    instance->encoder_right.pulsePosMinus();
  } else {
    instance->encoder_right.pulsePosPlus();
  }
}

double Entity::get_velocity(WheelSide wheel) {
  encoder_left.updateSpeed();
  encoder_right.updateSpeed();
  return wheel == LEFT_WHEEL ? encoder_left.getCurrentSpeed() : encoder_right.getCurrentSpeed();
}



void Entity::move_at_coord(const Vector2D& v) {

  Serial.print("MI STO PROIETTANDO SULLA RETTA");

  this->turn_at(v.get_vdegree());
  Serial.print("turn at per retta: ");
  Serial.println(v.get_vdegree());


  double seconds = v.get_vnorm() / SPEED;
  Serial.println(seconds);
  this->move_to(STRAIGHT, seconds + 2);
}

void Entity::actions() {

  if (internal_state == SLEEP) return;

  if (internal_state == CALIBRATION) {

    Serial.println("STO FACENDO CALIBRAZIONE");

    // pid calibration
    //for(int i = 0; i< 4; i++)
    turn_at(357);
    set_state(SCAN);
    return;
  }



  if (internal_state == SCAN) {

    scan(SCAN_NUMBER);

    this->internal_map.serial_print();

    // Filtraggio algoritmo
    aggregate_cluster();

    // Set min distance to keep
    filter_cluster(CLUSTER_DIST);

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



    
    /*
    if (close_to(distance_1, distance_2, CLOSE_TO_ERROR) && close_to(distance_2, distance_3, CLOSE_TO_ERROR)) {
      // Tutte le distanze sono simili → triangolo equilatero
      //random_walk() //TODO
    }
    */

    set_state(GAT);
  }

  if (internal_state == GAT) {

    double bari_angle = center_gravity.get_vdegree();

    Serial.print("POS VETTORE");
    Serial.println(center_gravity.get_x());
    Serial.println(center_gravity.get_y());
    Serial.print("-- -- POS VETTORE  -- - - ");

    Serial.print("ANGOLO BARICENTRO NON NORMALIZZATO ");

    Serial.println(bari_angle);

    double norm_bari_angle = bari_angle < 0 ? 360 + bari_angle : bari_angle;

    turn_at(norm_bari_angle);

    //reset_motors_pid();
    encoder_left.loop();
    encoder_right.loop();

    Serial.print("MIN_RADIUS");
    float elapsed_millis = 0;

    auto until_min_radius = [this,&elapsed_millis](double millis) {
      double current_dist = ultra.distanceCm();

      elapsed_millis = millis;

      Serial.println("DISTANZA ATTUALE");
      Serial.println(current_dist);
      Serial.println("SEcondi rimasti");
      Serial.println(center_gravity.get_vnorm() / SPEED);

      return (current_dist <= MIN_RADIUS) || ((millis / 1000) >= ((center_gravity.get_vnorm() - 10) / SPEED));
    };

    // _---- quando passa da turn at a move until fa un po quello che vuole
    // da capire perchè


    move_until(until_min_radius);

    delay(1.3 * MAX_DELAY);

    move_to(BACK,elapsed_millis / 1000 );
    turn_at(360 - norm_bari_angle);

    Vector2D v1 = triangle[0];
    Vector2D v2 = triangle[1];
    Vector2D distance_3_vector = distance_between_vectors(triangle[0], triangle[1]);

    double distance_1 = v1.get_vnorm();
    double distance_2 = v2.get_vnorm();
    double distance_3 = distance_3_vector.get_vnorm();


    if (distance_3 <= distance_2 && distance_3 <= distance_1) {
      set_state(MASTER);
      direction = cross_product(v1, v2) > 0 ? distance_3_vector : distance_3_vector * -1;
    } else {
      set_state(SLAVE);
      direction = distance_1 < distance_2 ? v1 : v2;
    }

    Serial.println("INTERNAL STATE");
    Serial.println(internal_state == SLAVE ? "SCHIAVO" : "MASTER");
  }

  if (internal_state == MASTER) {

    Serial.print("SONO IL MASTER");
    Vector2D u = triangle[0];
    Vector2D v = triangle[1];

    // possiamo scegliere la direzione da andare, prendiamo sempre quella con la norma più bassa
    Vector2D w_1 = distance_between_vectors(u * 2.0, v);
    Vector2D w_2 = distance_between_vectors(v * 2.0, u);

    Vector2D w = dot_product(w_2, direction) > 0 ? w_2 : w_1;

    Vector2D h = distance_between_vectors(u, v);

    Serial.println("U");

    u.print_vector();
    Serial.println("V");
    v.print_vector();
    Serial.println("W");
    w.print_vector();
    Serial.println("H");
    h.print_vector();

    Serial.print("Angolo Retta");
    Serial.println(w.get_vdegree());

    move_at_coord(w);

    Serial.print("Torno nel punto di partenza");
    Serial.println(360 - w.get_vdegree());

    turn_at(360 - w.get_vdegree());

    float d1 = u.get_vnorm();
    float d2 = v.get_vnorm();

    Vector2D other = d1 < d2 ? u : v;
    float dt_prod = cross_product(direction, other);
    float opposite_angle_measure = dt_prod < 0 ? direction.get_vdegree() + 180 : direction.get_vdegree();
    turn_at(opposite_angle_measure >= 360 ? opposite_angle_measure - 360 : opposite_angle_measure);
    set_state(FOLLOWING);
  }

  if (internal_state == SLAVE) {

    Vector2D v1 = this->triangle[0];
    Vector2D v2 = this->triangle[1];
    float d1 = v1.get_vnorm();
    float d2 = v2.get_vnorm();

    Vector2D other = d1 >= d2 ? v1 : v2;

    float dt_prod = cross_product(other, this->direction);

    Serial.println("PRPODOTTO SCALARE");
    Serial.println(dt_prod);

    float opposite_angle_measure = dt_prod < 0 ? this->direction.get_vdegree() + 180 : this->direction.get_vdegree();

    Serial.println(opposite_angle_measure);

    turn_at(opposite_angle_measure >= 360?opposite_angle_measure - 360 : opposite_angle_measure);

    Vector2D distance_1_vector = this->triangle[0];
    Vector2D distance_2_vector = this->triangle[1];
    Vector2D distance_3_vector = distance_between_vectors(distance_1_vector, distance_2_vector);

    Vector2D w = distance_1_vector.get_vnorm() > distance_2_vector.get_vnorm() ? distance_1_vector : distance_2_vector;

    Vector2D w_1 = distance_between_vectors(w * 2, distance_3_vector);
    Vector2D w_2 = distance_between_vectors(distance_3_vector * 2, w);

    float seconds_to_wait = w_1.get_vnorm() > w_2.get_vnorm() ? w_2.get_vnorm() / SPEED : w_1.get_vnorm() / SPEED;
    Serial.println("ASPETTA I SECONDI");
    Serial.println(seconds_to_wait);
    float first_current_dist = this->get_avg_distance(SCAN_SAMPLE_NUMBER);
    bool has_started = false;
    // busy-waiting
    while (!has_started) {
      this->stop();
      float current_dist = this->get_avg_distance(SCAN_SAMPLE_NUMBER);
      has_started = current_dist < first_current_dist;
    }


    Serial.println("METTI COME FOLLOWING");
    
    delay(DELAY_ERROR);
    set_state(FOLLOWING);
  }


  if (internal_state == FOLLOWING) {


    LineParam st_line = LineParam(direction=direction);

    st_line.get_versor().print_vector();
    float first_current_dist = get_avg_distance(SCAN_SAMPLE_NUMBER);

    if (first_current_dist == 400)
    {
      
      Serial.println("SONO DENTRO RAMO IF ");
      while(true){
          
          Serial.println("STO FACENDO UN LOOP");
          move_to(STRAIGHT ,MOVE_LINE_SECONDS);
          move_to_triangle(st_line,direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS, HEAD);
          move_to(STRAIGHT ,MOVE_TRIANGLE_SECONDS);
          back_to_line(st_line,direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS, HEAD);
      }
    }
    else
    {
      Serial.println("SONO DENTRO RAMO ELSE");
      while(true)
      {
        Serial.println("STO FACENDO UN LOOP");

        bool has_started = false;
        while (not has_started)
        {
            stop();
            float current_dist = get_avg_distance(SCAN_SAMPLE_NUMBER);
            has_started = current_dist > (first_current_dist + DISTANCE_BETWEEN_ROBOTS_ERROR);
        }
          
        follow(DISTANCE_BETWEEN_ROBOTS, MOVE_LINE_SECONDS);
        move_to_triangle(st_line,direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS, TAIL);
        move_to(STRAIGHT,MOVE_TRIANGLE_SECONDS);
        back_to_line(st_line,  direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS, TAIL);
      }           
    }
  }
}

void Entity::move_to(Directions dir, double seconds) {

  resetPID(&encoder_left.getEncoderType().PID_speed);
  resetPID(&encoder_right.getEncoderType().PID_speed);



  double vel_left = getPwmForWheel((MAX_VEL + MIN_VEL) / 2, dir, LEFT_WHEEL);
  double vel_right = getPwmForWheel((MAX_VEL + MIN_VEL) / 2, dir, RIGHT_WHEEL);


  unsigned long endTime = millis() + (unsigned long)(1000);



  while (millis() < endTime) {

    encoder_left.runSpeed(vel_left);
    encoder_right.runSpeed(vel_right);
    encoder_left.loop();
    encoder_right.loop();
  }


  endTime = millis() + (unsigned long)(seconds * 1000);


  while (millis() < endTime) {
    encoder_left.runSpeed(vel_left);
    encoder_right.runSpeed(vel_right);
    encoder_left.loop();
    encoder_right.loop();
  }

  encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();

  //delay(5);
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
  double eps_tol = CLUSTER_AGGR_DIST;  // tolleranza distanza cluster
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
  double angle = 0;
  double delta_angle = 360.0 / (double)sample_measurement;
  int i = 0;
  do {
    double distance = get_avg_distance(SCAN_SAMPLE_NUMBER);

    internal_map.push_back({ angle, distance });

    Serial.println("--------------");
    Serial.print("SCANSIONE:");
    Serial.println(i);
    Serial.println(angle);
    Serial.println(distance);
    Serial.println("--------------");

    angle = angle + delta_angle;

    turn_at(delta_angle);
    i++;
  } while (i < sample_measurement);

  turn_at(delta_angle + (ANGULAR_TOL * sample_measurement));
}

double Entity::get_avg_distance(int n_sample) {

  // utilizzando la moda possiamo ottenere migliori risultati in caso di interferenze dovute all ultrasuoni
  // TODO

  if (n_sample < 1) return 0;
  if (n_sample > 50) n_sample = 50;  // evita array troppo grandi

  double samples[50];

  for (int i = 0; i < n_sample; ++i) {
    samples[i] = ultra.distanceCm();
  }

  for (int i = 0; i < n_sample - 1; ++i) {
    for (int j = 0; j < n_sample - i - 1; ++j) {
      if (samples[j] > samples[j + 1]) {
        double tmp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = tmp;
      }
    }
  }

  if (n_sample % 2 == 1) {
    return samples[n_sample / 2];  // dispari
  } else {
    int mid = n_sample / 2;
    return (samples[mid - 1] + samples[mid]) / 2.0;  // pari
  }
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

  resetPID(&encoder_left.getEncoderType().PID_speed);
  resetPID(&encoder_right.getEncoderType().PID_speed);

  double num_times = angle / 30.0;

  angle += ANGULAR_TOL;

  this->gyro.begin();
  delay(0.5);

  double start_angle = 0;
  double read_angle;
  double target_angle = angle;
  double actual_angle = start_angle;
  double error = angle;
  double interval[2] DUMMY_FUNCTION_INTERVAL;
  bool first_read = true;

  while (fabs(error) > ANGULAR_TOL) {
    this->gyro.update();

    double angle_z = first_read ? 0 : this->gyro.getAngleZ();
    first_read = false;


    //Serial.println(angle_z);

    read_angle = normalizeAngle(angle_z);

    //Serial.println("READ ANGLE");
    //Serial.println(read_angle);
    //Serial.println("ERROR");
    //Serial.println(error);

    //Serial.println("START AGNLE");
    //Serial.println(start_angle);


    actual_angle = fabs(read_angle - start_angle);


    //Serial.println("actual angle");
    //Serial.println(actual_angle);

    //Serial.println("TAGET AGNLE ");
    //Serial.println(target_angle);

    error = target_angle - actual_angle;

    //per ra
    //double speed = dummy_function(fabs(error),interval,2,this->vel/2,5);

    double speed = dummy_function(fabs(error), interval, 1, this->vel, MIN_VEL);

    //inizio operazione FONDAMENTALE
    //Serial.println();

    delay(0.01);
    //fine operazione FONDAMENTALE
    encoder_left.runSpeed(getPwmForWheel(speed, INVERT, LEFT_WHEEL));
    encoder_right.runSpeed(getPwmForWheel(speed, INVERT, RIGHT_WHEEL));

    encoder_left.loop();
    encoder_right.loop();
    this->gyro.update();

    read_angle = normalizeAngle(this->gyro.getAngleZ());
    actual_angle = fabs(read_angle - start_angle);
    error = target_angle - actual_angle;
  }

  encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();

}


double Entity::normalizeAngle(double angle) {

  if (angle > 1) {
    angle = 360 - angle;
  } else {
    angle = fabs(angle);
  }
  return angle;
}


template<typename Func>
void Entity::move_until(Func stopping_criteria, Directions dir, double keep_angle) {

  resetPID(&encoder_left.getEncoderType().PID_speed);
  resetPID(&encoder_right.getEncoderType().PID_speed);


  // PWM di base per la direzione scelta
  double vel_left = getPwmForWheel(this->vel, dir, LEFT_WHEEL);
  double vel_right = getPwmForWheel(this->vel, dir, RIGHT_WHEEL);

  double started_millis = millis();
  double elapsed_millis = started_millis;

  encoder_left.runSpeed(MIN_VEL);
  encoder_right.runSpeed(MIN_VEL);
  encoder_left.loop();
  encoder_right.loop();

  while (!stopping_criteria(elapsed_millis - started_millis)) {

    elapsed_millis = millis();

    encoder_left.runSpeed(vel_left);
    encoder_right.runSpeed(vel_right);

    Serial.print("vL: ");
    Serial.println(get_velocity(LEFT_WHEEL));
    Serial.print("vR: ");
    Serial.println(get_velocity(RIGHT_WHEEL));

    encoder_left.loop();
    encoder_right.loop();
  }

  // Arresta i motori
  encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();
}


template<typename Func> double Entity::corrected_pwm(double base_pwm, double error, double K, bool isLeft, Func correction_func) {
  double correction = correction_func(K, error);
  return base_pwm - correction;
}


void Entity::follow(double min_dist, double seconds) {
  
    resetPID(&encoder_left.getEncoderType().PID_speed);
    resetPID(&encoder_right.getEncoderType().PID_speed);
    float t0 = millis();

    while (millis() - t0 < seconds)
    {
      float current_dist = get_avg_distance(SCAN_SAMPLE_NUMBER);
      float error = current_dist - min_dist;
      if ( error < 0 ) stop();
      else
      {
        double vel_left = getPwmForWheel(this->vel, STRAIGHT, LEFT_WHEEL);
        double vel_right = getPwmForWheel(this->vel, STRAIGHT, RIGHT_WHEEL);

        encoder_left.runSpeed(vel_left);
        encoder_right.runSpeed(vel_right);
      }
      encoder_left.loop();
      encoder_right.loop();

    }
    
    stop();
  }


void Entity::move_to_triangle(LineParam pt, double distance, EntityState state) {
  Vector2D vector_1 = Vector2D(pt.get_versor() * distance);
  Vector2D ortogonal_vector = Vector2D(vector_1.y, -vector_1.x);
  Vector2D projection = vector_1 + ortogonal_vector;

  if (state == HEAD) {
  
    move_at_coord(projection);
    double projection_deg = 180 - projection.get_vdegree();
    double deg_to_turn = projection.get_vdegree() - 90;
    turn_at(360 - deg_to_turn);
  
  } else {
  
    turn_at(90);
    double time = projection.get_vnorm() / SPEED;
    delay(time + 1.5 + 2 * DELAY_ERROR);
  }
}

void Entity::back_to_line(LineParam pt, double distance, EntityState state) {
  Vector2D vector_1 = Vector2D(pt.get_versor() * -1 * distance);
  Vector2D ortogonal_vector = Vector2D(vector_1.y, -vector_1.x);
  Vector2D projection = vector_1 + ortogonal_vector;

  if (state == HEAD) {
    double projection_deg = 180 - projection.get_vdegree();
    double deg_to_turn = projection.get_vdegree() - 90;
    
    turn_at(270);

    Vector2D go_back_vec(projection.get_vnorm(), 180 - deg_to_turn, 0);
    Vector2D go_back = Vector2D(go_back_vec.x,go_back_vec.y) * -1;
    move_at_coord(go_back_vec);
    turn_at(deg_to_turn);
  } else {
    
    turn_at(270);
    bool has_started = false;
    float first_current_dist = get_avg_distance(SCAN_SAMPLE_NUMBER);
    while (!has_started)
    {
      stop();
      float current_dist = get_avg_distance(SCAN_SAMPLE_NUMBER);
      has_started = current_dist < first_current_dist;
    }
    
    delay(DELAY_ERROR);
  }
}


void Entity::resetPID(volatile PID_internal *pid)
{
  pid->Integral     = 0;
  pid->last_error   = 0;
  pid->differential = 0;
  pid->Output       = 0;
}


void Entity::stop()
{
   encoder_left.setMotorPwm(0);
  encoder_right.setMotorPwm(0);
  encoder_left.loop();
  encoder_right.loop();
}
