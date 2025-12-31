#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, GyroSensor
from pybricks.tools import StopWatch
from lin_alg import Vector2D, LineParam, get_avg_center, distance_between_vectors_vec, close_to, dot_product, cross_product
import umath


TIME_STEP = 64
DT = TIME_STEP / 1000.0
MAX_SPEED = 1
TURN_AT_FRACTION = 0.1
SLAVE_DELAY_VEL = TURN_AT_FRACTION * MAX_SPEED

SCAN_NUMBER = 64
SCAN_SAMPLE_NUMBER = 5
CLUSTER_DIST = 250
CLUSTER_AGGR_DIST = 5
MIN_RADIUS = 20
CLOSE_TO_ERROR = 10
DISTANCE_BETWEEN_ROBOTS = 20
DISTANCE_BETWEEN_ROBOTS_ERROR = 5
MOVE_TRIANGLE_SECONDS = 10
MOVE_LINE_SECONDS = 10
DELAY_ERROR = 2 * 3.14 / SLAVE_DELAY_VEL
WHEEL_RADIUS = 5

MAX_DELAY = 300

#STATI
SLEEP = 0
CALIBRATION = 1
SCAN = 2
GAT = 3
MASTER = 4
SLAVE = 5
FOLLOWING = 6

class Entity:
    def __init__(self):
        self._debugger = False
        self.robot = EV3Brick()
        self.left = Motor(1)
        self.right = Motor(2)
        self.stop()

        self.ultra = UltrasonicSensor(10)
        self.gyro = GyroSensor(11)
        self.watch = StopWatch()

        self.internal_state = SCAN
        self.last_state = SLEEP
        self.internal_map = []
        self.triangle = []
        self.center_gravity = Vector2D()
        self.direction = Vector2D()
        self.is_master = False
        

    def set_speed(self, left_speed: float, right_speed: float):
        self.left.run(left_speed)
        self.right.run(right_speed)
        
    def stop(self):
        self.left.stop()
        self.right.stop()

    def turn_at(self, angle_deg: float):
        target = umath.radians(angle_deg)
        rotated = 0.0
        self.stop()

        while abs(rotated) <= abs(target):
            gz = self.gyro.speed()
            rotated += gz * DT

            if angle_deg > 0:
                self.set_speed(-TURN_AT_FRACTION * MAX_SPEED, TURN_AT_FRACTION * MAX_SPEED)
            else:
                self.set_speed(TURN_AT_FRACTION * MAX_SPEED, -TURN_AT_FRACTION * MAX_SPEED)
        
        self.stop()

    def move_straight(self, seconds: float, back: bool= False):
        angle = 0.0
        last_angle = 0.0
        integral = 0.0

        Kp = 0.2
        Ki = 0.01
        Kd = 0.1

        base_speed = -MAX_SPEED if back else MAX_SPEED
        t0 = self.watch.time()

        while self.watch.time() - t0 < seconds:

            gyro_rate = self.gyro.speed()
            angle += gyro_rate * DT

            error = angle
            integral += error * DT
            derivative  =(error - last_angle) / DT
            last_angle = error

            correction = (Kp * error) + (Ki * integral) + (Kd * derivative)

            if not back:
                left_speed = base_speed + correction
                right_speed = base_speed - correction
            else:
                left_speed = base_speed - correction
                right_speed = base_speed + correction

            self.set_speed(left_speed, right_speed)

            if self._debugger:
                print(f"T: Ang: {angle:.4f} | Correction: {correction:.3f}")
                print(f"L: {left_speed:.2f} | R: {right_speed:.2f}")

    def get_avg_distance(self, n:int = 5):
        vals = []
        for _ in range(n):
            vals.append(self.ultra.distance())

        vals.sort()
        m = len(vals)
        if m == 0: return 0
        if m % 2 == 1:
            return vals[m//2]
        return (vals[m//2 - 1] + vals[m//2]) / 2.0

    def scan(self, n):
        step_angle = 360.0 / n
        for i in range(n):
            self.internal_map.append((i*step_angle, self.get_avg_distance(SCAN_SAMPLE_NUMBER)))
            self.turn_at(step_angle)

    def aggregate_cluster(self):
        if len(self.internal_map) == 0:
            return
        clusters = []
        c_angle, c_dist = self.internal_map[0]
        count = 1

        if abs(self.internal_map[0][1] - self.internal_map[-1][1]) <= CLUSTER_AGGR_DIST:
            self.internal_map.pop()

        for i in range(1, len(self.internal_map)):
            ang, dist = self.internal_map[i]
            prev_ang, prev_dist = self.internal_map[i-1]

            if abs(dist - prev_dist) <= CLUSTER_AGGR_DIST:
                c_angle += ang
                c_dist += dist
                count += 1
            else:
                clusters.append((c_angle/count, c_dist/count))
                c_angle, c_dist = ang, dist
                count = 1

        clusters.append((c_angle/count, c_dist/count))
        
        self.internal_map = clusters


    def filter_cluster(self, min_dist: float):
        self.internal_map = [(a, d+4) for a,d in self.internal_map if d <= min_dist]

    def move_at_coord(self, vec: Vector2D):
        ang = vec.get_vdegree()
        dist = vec.get_vnorm()
        self.turn_at(ang)
        seconds = dist / MAX_SPEED
        self.move_straight(seconds)

    def move_to_triangle(self, distance: float):
        vector_1 = Vector2D(1, 0) * -distance
        ortogonal_vector = Vector2D(vector_1.y, -vector_1.x)
        projection = vector_1 + ortogonal_vector

        if self.last_state == MASTER:
            self.move_at_coord(projection)
            deg_to_turn = projection.get_vdegree() - 90
            self.turn_at(360 - deg_to_turn)
        else:
            self.turn_at(90)
            time = projection.get_vnorm() / MAX_SPEED
            self.delay(time + 1.5 + 2 * DELAY_ERROR)
    
    def back_to_line(self, distance: float):
        vector_1 = Vector2D(1, 0) * -distance
        ortogonal_vector = Vector2D(vector_1.y, -vector_1.x)
        projection = vector_1 + ortogonal_vector

        if self.last_state == MASTER:
            deg_to_turn = projection.get_vdegree() - 90

            self.turn_at(270)

            go_back_vec = Vector2D(projection.get_vnorm(), 180 - deg_to_turn, 0)
            go_back = Vector2D(go_back_vec.x, go_back_vec.y) * -1

            self.move_at_coord(go_back)
            self.turn_at(deg_to_turn)

        else:
            self.turn_at(270)

            has_started = False
            first_current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
            while not has_started:
                self.stop()
                current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
                has_started = current_dist < first_current_dist
                
                if self._debugger:
                    print(f"DISTANZA ATTUALE {current_dist}")
                    print(f"QUI STO ASPETTANDO")

            self.delay(DELAY_ERROR)

    def follow(self, min_dist, seconds):
        t0 = self.watch.time()

        while self.watch.time() - t0 < seconds:
            current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
            error = current_dist - min_dist

            if error < 0:
                self.stop()
            else:
                self.set_speed(MAX_SPEED, MAX_SPEED)

        self.stop()

    def set_state(self, state):
        self.last_state = self.internal_state
        self.internal_state = state

    def evaluate_metrics(self):
        v1 = self.triangle[0]
        v2 = self.triangle[1]
        v3 = distance_between_vectors_vec(v1, v2)

        d1 = v1.get_vnorm()
        d2 = v2.get_vnorm()
        d3 = v3.get_vnorm()

        if close_to(d1, d2, CLOSE_TO_ERROR) and close_to(d2, d3, CLOSE_TO_ERROR):
            print("Equilateral Random Walk")

        if self.internal_state == SCAN:
            self.set_state(GAT)
        else:
            if d3 <= d2 and d3 <= d1:

                self.internal_state = MASTER

                if(cross_product(v1,v2) > 0):
                    self.direction = v3
                else:
                    self.direction = v3 * -1


                print("SONO MASTER")

            else:
                self.internal_state = SLAVE
                self.direction = v1 if d1 < d2 else v2
                print("SONO SLAVE")

    def move_until(self, stopping_criteria, forward=True):
        vel_l = MAX_SPEED if forward else -MAX_SPEED
        vel_r = MAX_SPEED if forward else -MAX_SPEED

        t0 = self.watch.time()
        elapsed = 0
        while not stopping_criteria(elapsed):
            elapsed = self.watch.time() - t0
            self.set_speed(vel_l, vel_r)

        self.stop()

    def delay(self, seconds):
        start = self.watch.time()
        while self.watch.time() - start < seconds:
            continue
    
    def actions(self):
        if self.internal_state == SLEEP:
            return

        if self.internal_state == SCAN:
            self.scan(SCAN_NUMBER)
            self.aggregate_cluster()

            self.filter_cluster(CLUSTER_DIST)

            # calcolo baricentro e retta
            if len(self.internal_map) == 2:
                self.triangle = []
                for a, d in self.internal_map:
                    self.triangle.append(Vector2D(d, a, 0))
                self.triangle.append(Vector2D(0,0))
            else:
                print(f"---- FOUND {len(self.internal_map)} ROBOTS ----")
                self.set_state(SLEEP)
                return

            self.center_gravity = get_avg_center(self.triangle)
            self.evaluate_metrics()
        
        if self.internal_state == GAT:
            bari_angle = self.center_gravity.get_vdegree()

            norm_bari_angle = 360 + bari_angle if bari_angle < 0 else bari_angle

            self.turn_at(norm_bari_angle)
            elapsed_seconds = 0

            def until_min_radius(seconds):
                nonlocal elapsed_seconds
                elapsed_seconds = seconds
                current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
                return (current_dist <= MIN_RADIUS) or ((seconds) >= ((self.center_gravity.get_vnorm() - 10) / (MAX_SPEED)))

            self.move_until(until_min_radius)
            self.delay(1.3 * MAX_DELAY)
            self.move_straight(elapsed_seconds, back=True)
            self.turn_at(360 - norm_bari_angle)
            self.evaluate_metrics()


        if self.internal_state == MASTER:
            if self._debugger:
                print("MASTER: moving along direction")
                print(self.triangle)
            
            u : Vector2D = self.triangle[0];
            v : Vector2D = self.triangle[1];

            # possiamo scegliere la direzione da andare, prendiamo sempre quella con la norma più bassa
            w_1 = distance_between_vectors_vec(u * 2.0, v)
            w_2 = distance_between_vectors_vec(v * 2.0, u)
            
            dot_prod2 = dot_product(w_2, self.direction)
            w = w_2 if dot_prod2 > 0 else w_1

            h = distance_between_vectors_vec(u, v)
            
            if self._debugger:
                print("U")
                u.print_vector()
                print("V")
                v.print_vector()
                print("W")
                w.print_vector()
                print("H")
                h.print_vector()

                print("Angolo Retta")
                print(w.get_vdegree())

            self.move_at_coord(w)

            if self._debugger:
                print("Torno nel punto di partenza")
                print(360 - w.get_vdegree())

            self.turn_at(360 - w.get_vdegree())

            st_line = LineParam(self.direction)

            if self._debugger:
                print("DIRECTION GET V DEGREE")
                print(self.direction.get_vdegree())

                # metodo deprecato
                print("STO GIRANDO VERSO LA CRESCENZA");
                print("-1: ")
                print(st_line.evaluate(-1).get_y())
                print("1: ")
                print(st_line.evaluate(1).get_y())
            
            v1 = self.triangle[0]
            v2 = self.triangle[1]
            d1 = v1.get_vnorm()
            d2 = v2.get_vnorm()

            other = v1 if d1 < d2 else v2
            dt_prod = cross_product(self.direction,other)
            opposite_angle_measure = self.direction.get_vdegree() + 180 if dt_prod < 0 else self.direction.get_vdegree()
            self.turn_at(opposite_angle_measure - 360 if opposite_angle_measure >= 360 else opposite_angle_measure)

            self.set_state(FOLLOWING)

            return

        if self.internal_state == SLAVE:
            v1 = self.triangle[0]
            v2 = self.triangle[1]
            d1 = v1.get_vnorm()
            d2 = v2.get_vnorm()


            other = v1 if d1 >= d2 else v2

            dt_prod = cross_product(other, self.direction)

            opposite_angle_measure = self.direction.get_vdegree() + 180 if dt_prod < 0 else self.direction.get_vdegree()

            self.turn_at(opposite_angle_measure - 360 if opposite_angle_measure >= 360 else opposite_angle_measure)
            
            distance_1_vector = self.triangle[0]
            distance_2_vector = self.triangle[1]
            distance_3_vector = distance_between_vectors_vec(distance_1_vector, distance_2_vector)

            w = distance_1_vector if distance_1_vector.get_vnorm() > distance_2_vector.get_vnorm() else distance_2_vector

            w_1 = distance_between_vectors_vec(w*2, distance_3_vector)
            w_2 = distance_between_vectors_vec(distance_3_vector*2, w)

            first_current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
            has_started = False
            while not has_started:
                self.stop()
                current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
                has_started = current_dist < first_current_dist

            self.delay(DELAY_ERROR)
            self.set_state(FOLLOWING)

            return

        if self.internal_state == FOLLOWING:
            if self.last_state == MASTER:
                for _ in range(2):
                    self.move_straight(MOVE_LINE_SECONDS)
                    self.move_to_triangle(self.direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS)
                    self.move_straight(MOVE_TRIANGLE_SECONDS)
                    self.back_to_line(self.direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS)

            else:
                for _ in range(2):
                    has_started = False
                    first_current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)

                    while not has_started:
                        self.stop()
                        current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
                        has_started = current_dist > (first_current_dist + DISTANCE_BETWEEN_ROBOTS_ERROR)

                    self.follow(DISTANCE_BETWEEN_ROBOTS, MOVE_LINE_SECONDS)
                    self.move_to_triangle(self.direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS)

                    self.move_straight(MOVE_TRIANGLE_SECONDS)
                    self.back_to_line(self.direction.get_vnorm() + DISTANCE_BETWEEN_ROBOTS)

            return

# Main Loop
robot = Entity()

robot.actions()
