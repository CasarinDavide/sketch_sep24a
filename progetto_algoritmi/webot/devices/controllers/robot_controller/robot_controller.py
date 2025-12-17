from controller import Robot, Motor, DistanceSensor, Gyro

# entity.py
import math
from controller import Robot
import random 
from lin_alg import Vector2D, LineParam, get_avg_center, distance_between_vectors_vec, close_to
from lin_alg import to_radians, to_degree,dot_product,cross_product

TIME_STEP = 32
DT = TIME_STEP / 1000.0
MAX_SPEED = 4
SPEED = 0.15
TURN_AT_FRACTION = 0.1
SLAVE_DELAY_VEL = TURN_AT_FRACTION * MAX_SPEED

# costanti (adatta se necessario)
SCAN_NUMBER = 64

SCAN_SAMPLE_NUMBER = 5
CLUSTER_AGGR_DIST = 5
CLUSTER_DIST = 250
MIN_RADIUS = 20
CLOSE_TO_ERROR = 10
DISTANCE_BETWEEN_ROBOTS = 20
DISTANCE_BETWEEN_ROBOTS_ERROR = 5
MOVE_TRIANGLE_SECONDS = 2
MOVE_LINE_SECONDS = 2
DELAY_ERROR =  2 * 3.14 / SLAVE_DELAY_VEL
WHEEL_RADIUS = 0.01

# stati
SLEEP = 0
CALIBRATION = 1
SCAN = 2
GAT = 3
MASTER = 4
SLAVE = 5
FOLLOWING = 6
HEAD = 7
TAIL = 8

class Entity:
    def __init__(self):
        self.robot = Robot()
        # motors

        self._debugger_ = False
        self.left = self.robot.getDevice("left wheel motor")
        self.right = self.robot.getDevice("right wheel motor")
        self.left.setPosition(float("inf"))
        self.right.setPosition(float("inf"))
        self.left.setVelocity(0)
        self.right.setVelocity(0)
        # sensors
        self.ultra = self.robot.getDevice("distance sensor")
        self.ultra.enable(TIME_STEP)
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(TIME_STEP)
        # state
        self.internal_state = SCAN
        self.last_state = SLEEP
        self.internal_map = []
        self.triangle = []
        self.center_gravity = Vector2D()
        self.direction = Vector2D()

    def step(self):
        return self.robot.step(TIME_STEP) != -1

    def set_velocity(self, l, r):
        self.left.setVelocity(l)
        self.right.setVelocity(r)

    def stop(self):
        self.set_velocity(0, 0)

    def turn_at(self, angle_deg):
        target = math.radians(angle_deg)
        rotated = 0.0
        self.stop()
        while self.step():
            gz = self.gyro.getValues()[2]
            rotated += gz * DT
            
            if angle_deg > 0:
                self.set_velocity(-TURN_AT_FRACTION * MAX_SPEED, TURN_AT_FRACTION *MAX_SPEED)
            else:
                self.set_velocity(TURN_AT_FRACTION * MAX_SPEED, -TURN_AT_FRACTION * MAX_SPEED)
                
            if abs(rotated) >= abs(target):
                break
        
        self.stop()
        
    def move_straight(self, seconds):
        t0 = self.robot.getTime()
        while self.robot.getTime() - t0 < seconds:
            self.set_velocity(MAX_SPEED, MAX_SPEED)
            if not self.step(): break
        self.stop()

    def get_avg_distance(self, n=5):
        vals = []
        for _ in range(n):
            vals.append(self.ultra.getValue())
            if not self.step(): break
        vals.sort()
        m = len(vals)
        if m == 0: return 0.0
        if m % 2 == 1:
            return vals[m//2]
        return (vals[m//2 - 1] + vals[m//2]) / 2.0

    def scan(self, n):
        self.internal_map = []
        step_angle = 360.0 / n
        for i in range(n):
            self.internal_map.append((i*step_angle,self.get_avg_distance(SCAN_SAMPLE_NUMBER)))
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
            prev_dist = self.internal_map[i-1][1]
            if abs(dist - prev_dist) <= CLUSTER_AGGR_DIST:
                c_angle += ang
                c_dist += dist
                count += 1
            else:
                clusters.append((c_angle/count, c_dist/count))
                c_angle, c_dist = ang, dist
                count = 1
        clusters.append((c_angle/count, c_dist/count))

        print(clusters)
        self.internal_map = clusters

    def filter_cluster(self, min_d):
        self.internal_map = [(a, d + 4) for a,d in self.internal_map if d <= min_d]

    def move_at_coord(self, vec: Vector2D):
        ang = vec.get_vdegree()
        dist = vec.get_vnorm()
        self.turn_at(ang)
        seconds = dist / MAX_SPEED
        self.move_straight(seconds)

    # ---- MOVIMENTO TRIANGOLO E RITORNO (TRADUZIONE DIRETTA) ----
    def move_to_triangle(self, pt: LineParam, distance: float, state_flag):
        vector_1 = pt.get_versor() * distance
        ortogonal_vector = Vector2D(vector_1.y, -vector_1.x)
        # projection = vector_1 + ortogonal_vector;
        projection = vector_1 + ortogonal_vector

        if state_flag == HEAD:
            # esegui move_at_coord sul vettore di proiezione
            self.move_at_coord(projection)
            projection_deg = 180 - projection.get_vdegree()
            deg_to_turn = projection.get_vdegree() - 90
            # turn_at(360-deg_to_turn);
            self.turn_at(360 - deg_to_turn)
        else:
            # lo slave usa un semplice turn + delay (approssimazione)
            self.turn_at(90)
            time = projection.get_vnorm() / MAX_SPEED
            # delay simulata con step loop
            t0 = self.robot.getTime()
            while self.robot.getTime() - t0 < time + 1.5:
                if not self.step(): break

    def back_to_line(self, pt: LineParam, distance: float, state_flag):
        vector_1 = pt.get_versor() * distance
        ortogonal_vector = Vector2D(vector_1.y, -vector_1.x)
        projection = vector_1 + ortogonal_vector

        if state_flag == HEAD:
            projection_deg = 180 - projection.get_vdegree()
            deg_to_turn = projection.get_vdegree() - 90
            self.turn_at(180 - deg_to_turn)
            go_back_vec = Vector2D(projection.get_vnorm(), 180 - deg_to_turn, 0)
            # go_back_vec è costruito con costruttore polare (radius, alpha, int)
            # ma la nostra Vector2D prende (x,y) salvo flag; quindi ricostruiamo
            go_back = Vector2D(go_back_vec.x, go_back_vec.y)
            self.move_at_coord(go_back)
        else:
            self.turn_at(270)
            time = projection.get_vnorm() / MAX_SPEED
            t0 = self.robot.getTime()
            while self.robot.getTime() - t0 < time + 1.5:
                if not self.step(): break

    # follow semplificato
    def follow(self, min_dist, seconds):
        t0 = self.robot.getTime()

        while self.robot.getTime() - t0 < seconds:
            current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)

            error = current_dist - min_dist

            if error < 0:
                self.stop()
            else:
                self.set_velocity(MAX_SPEED, MAX_SPEED)

        self.stop()
                
    
    def set_state(self,state):
        self.last_state = self.internal_state
        self.internal_state = state

    def actions(self):
        
        self.last_state = GAT

        if self.internal_state == SLEEP:
            return 
        
        if self.internal_state == CALIBRATION:
            self.turn_at(360)
            self.set_state(SCAN)
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
                self.triangle.append(Vector2D(0, 0))
            else:
                print("---- FOUND MORE THAN 2 MOBILE ROBOTS ----")
                self.set_state(SLEEP)
                return

            self.center_gravity = get_avg_center(self.triangle)

            v1 = self.triangle[0]
            v2 = self.triangle[1]
            v3 = distance_between_vectors_vec(v1, v2)

            d1 = v1.get_vnorm()
            d2 = v2.get_vnorm()
            d3 = v3.get_vnorm()

            if close_to(d1, d2, CLOSE_TO_ERROR) and close_to(d2, d3, CLOSE_TO_ERROR):
                print("Equilateral (TODO random_walk)")

            if self.last_state != GAT:
                self.set_state(GAT)
            else:
                if d3 <= d2 and d3 <= d1:
                    self.internal_state = MASTER
                    self.direction = v3
                    print("SONO MASTER")

                else:
                    self.internal_state = SLAVE
                    self.direction = v1 if d1 < d2 else v2
                    print("SONO SLAVE")
                    
        if self.internal_state == GAT:
            bari_angle = self.center_gravity.get_vdegree();

            norm_bari_angle =  360 + bari_angle if bari_angle < 0 else bari_angle;

            self.turn_at(norm_bari_angle);
            
            def until_min_radius(seconds):
                current_dist = self.get_avg_distance(10)
                return (current_dist <= MIN_RADIUS) or ((seconds) >= ((self.center_gravity.get_vnorm() - 10) / (MAX_SPEED)))

            self.move_until(until_min_radius)    
            self.set_state(SLEEP)
            

        if self.internal_state == MASTER:

            print("MASTER: moving along direction")
            print(self.triangle)
            u : Vector2D = self.triangle[0];
            v : Vector2D = self.triangle[1];
            
            u.print_vector()
            v.print_vector()
            
            # possiamo scegliere la direzione da andare, prendiamo sempre quella con la norma più bassa
            w_1 = distance_between_vectors_vec(u * 2.0, v)
            w_2 = distance_between_vectors_vec(v * 2.0, u)
            
            w = w_2 if w_1.get_vnorm() > w_2.get_vnorm() else w_1

            h = distance_between_vectors_vec(u, v)

            if self._debugger_:
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

            if self._debugger_:            
                print("Torno nel punto di partenza")
                print(360-w.get_vdegree())

            self.turn_at(360-w.get_vdegree())

            
            self.delay(1)

            st_line = LineParam(self.direction)

            if self._debugger_:
                print("DIRECTION GET V DEGREE")
                print(self.direction.get_vdegree())
                
                print("STO GIRANDO VERSO LA CRESCENZA");
                print("-1: ")
                print(st_line.evaluate(-1).get_y())
                print("1: ")
                print(st_line.evaluate(1).get_y())            
            
            opposite_angle_measure = self.direction.get_vdegree() + 180   if st_line.evaluate(-1).get_y() > st_line.evaluate(1).get_y() else self.direction.get_vdegree();
            self.turn_at(opposite_angle_measure)

            opposite_angle_measure = opposite_angle_measure - 360  if opposite_angle_measure > 360 else opposite_angle_measure;

            self.set_state(FOLLOWING)

            return

        if self.internal_state == SLAVE:
            print("SLAVE: waiting then following")
            # delay approssimato

            print("DIREZIONE")
            self.direction.print_vector()
            
            v1 = self.triangle[0]
            v2 = self.triangle[1]
            d1 = v1.get_vnorm()
            d2 = v2.get_vnorm()

            other = v1 if d1 >= d2 else v2

            dt_prod = cross_product(other, self.direction)
            # dt_prod = dot_product(Vector2D(1,0),self.direction)

           
            opposite_angle_measure = self.direction.get_vdegree() + 180 if dt_prod < 0 else self.direction.get_vdegree()

            self.turn_at(opposite_angle_measure - 360 if opposite_angle_measure >= 360 else opposite_angle_measure)

            # devo aspettare per quando il master ci mette ad arrivare in posizione 

            distance_1_vector = self.triangle[0]
            distance_2_vector = self.triangle[1]
            distance_3_vector = distance_between_vectors_vec(distance_1_vector, distance_2_vector)

            w = distance_1_vector if distance_1_vector.get_vnorm() > distance_2_vector.get_vnorm() else distance_2_vector

            w_1 = distance_between_vectors_vec(w*2,distance_3_vector )
            w_2 = distance_between_vectors_vec(distance_3_vector*2, w)
            
            seconds_to_wait = w_2.get_vnorm() / MAX_SPEED if w_1.get_vnorm() > w_2.get_vnorm() else w_1.get_vnorm() / MAX_SPEED
            
            self.delay(seconds_to_wait + DELAY_ERROR)

            self.set_state(FOLLOWING)

            return

        if self.internal_state == FOLLOWING:
            print("FOLLOWING")
            self.follow(DISTANCE_BETWEEN_ROBOTS, MOVE_LINE_SECONDS)

            st_line = LineParam(direction=self.direction)
            first_current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
            print(f"PRIMA DISTANZA LETTA {first_current_dist}")

            if not close_to(first_current_dist, self.direction.get_vnorm(), CLOSE_TO_ERROR):
                print("SONO QUELLO DAVANTI")
                self.move_straight(MOVE_LINE_SECONDS)
                self.move_to_triangle(st_line, DISTANCE_BETWEEN_ROBOTS, HEAD)
                self.move_straight(MOVE_TRIANGLE_SECONDS)
                self.back_to_line(st_line, DISTANCE_BETWEEN_ROBOTS, HEAD)
            else:
                has_started = False

                # busy-waiting
                while not has_started:
                    self.stop()
                    current_dist = self.get_avg_distance(SCAN_SAMPLE_NUMBER)
                    print(f"DISTANZA ATTUALE {current_dist}")
                    has_started = first_current_dist < (current_dist + DISTANCE_BETWEEN_ROBOTS_ERROR)
                    print("QUI STO ASPETTANDO")
                
                print("NON STO PIU ASPETTANDO")
                print("FOLLOW")
                self.follow(DISTANCE_BETWEEN_ROBOTS, MOVE_LINE_SECONDS)
                self.move_straight(MOVE_TRIANGLE_SECONDS)
                self.back_to_line(st_line, DISTANCE_BETWEEN_ROBOTS, TAIL)
                
            return
    

    def move_until(self, stopping_criteria, forward=True):
        """
        stopping_criteria: funzione che accetta elapsed_time (in sec) e ritorna True quando fermarsi
        forward: True = avanti, False = indietro
        keep_angle: angolo in gradi da mantenere usando il gyro (opzionale)
        """

        # velocità di base
        base = MAX_SPEED
        vel_l = base if forward else -base
        vel_r = base if forward else -base

        t0 = self.robot.getTime()

        while self.step():
            elapsed = self.robot.getTime() - t0

            if stopping_criteria(elapsed):
                break
            
            self.set_velocity(vel_l, vel_r)

        self.stop()


    def delay(self, seconds):
        """
        Delay non bloccante per Webots.
        Usa robot.getTime() e step() per non congelare la simulazione.
        """
        start = self.robot.getTime()
        while self.robot.getTime() - start < seconds:
            if not self.step():
                break
            

    


# --- MAIN LOOP ---
robot = Entity()

robot.delay(random.uniform(0, 2.0))

while robot.step():
    robot.actions()

    
