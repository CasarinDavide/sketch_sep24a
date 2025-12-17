# lin_alg.py
import math
import numbers
PI = math.pi

def to_radians(degree):
    return degree * (PI / 180.0)

def to_degree(angle_rad):
    return angle_rad * (180.0 / PI)

def sign(value):
    return 1.0 if value > 0 else -1.0

def close_to(a, c, e_tol):
    return abs(a - c) < e_tol

# Vector2D equivalente al tuo C++
class Vector2D:
    def __init__(self, x=0.0, y=0.0, radius_alpha_flag=None):
        """
        - new Vector2D(x, y) -> cartesiane
        - new Vector2D(radius, alpha, 0) -> polari (alpha gradi)
        """
        if radius_alpha_flag is not None:
            # costruttore polar: (radius, alpha, int)
            radius = x
            alpha = y
            ang = to_radians(alpha)
            self.x = math.cos(ang) * radius
            self.y = math.sin(ang) * radius
            self.v_norm = radius
            self.v_degree = alpha
        else:
            self.x = x
            self.y = y
            self.v_norm = self.evaluate_vnorm(self.x, self.y)
            # atan2 -> radianti, poi gradi
            self.v_degree = to_degree(math.atan2(self.y, self.x))

    def evaluate_vnorm(self, x, y):
        return math.sqrt(x*x + y*y)

    def get_vnorm(self):
        return self.v_norm

    def get_vdegree(self):
        # normalizza come nel tuo C++
        deg = self.v_degree
        return deg if deg >= 0 else 360.0 + deg

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def __sub__(self, other):
        return Vector2D(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Vector2D(self.x + other.x, self.y + other.y)

    def __mul__(self, k):
        if isinstance(k,numbers.Real):
            return Vector2D(self.x * k, self.y * k)
        elif isinstance(k,Vector2D):
            return self.get_x() * k.get_x() + self.get_y() * k.get_y() 
    
    

    def print_vector(self):
        print("PRINT VECTOR")
        print("X:", self.x)
        print("Y:", self.y)
        print("NORM:", self.v_norm)
        print("ANGLE:", self.v_degree)
        print("END PRINT VECTOR")

# distance_between_vectors (vector version)
def distance_between_vectors_vec(a: Vector2D, b: Vector2D):

    a.print_vector()
    b.print_vector
    
    return a - b

def multi_vector(v : Vector2D,u : Vector2D):
    return v.get_x() * u.get_x() 
    
def get_avg_center(vectors):
    if len(vectors) == 0:
        return Vector2D(0, 0)
    avg_x = sum(v.x for v in vectors) / len(vectors)
    avg_y = sum(v.y for v in vectors) / len(vectors)
    return Vector2D(avg_x, avg_y)

def get_all_angles(a, b, c):
    # calcola angoli opposti con legge dei coseni (restituisce in radianti)
    def opposite_angle(adj1, adj2, opp):
        return math.acos(((adj1*adj1) + (adj2*adj2) - (opp*opp)) / (2*adj1*adj2))
    return [opposite_angle(a, b, c), opposite_angle(b, c, a), opposite_angle(c, a, b)]


def dot_product(v : Vector2D,u: Vector2D):
    return  (v * u)  /  (v.get_vnorm() * u.get_vnorm()) 

def cross_product(v: Vector2D, u: Vector2D):
    return v.x * u.y - v.y * u.x

# LineParam come nel C++
class LineParam:
    def __init__(self, point_on_line=None, direction=None):
        if direction is None and point_on_line is not None:
            # costruttore LineParam(direction) -> origin = (0,0)
            self.origin = Vector2D(0, 0)
            self.dir = self.make_unique_direction(point_on_line)
        else:
            self.origin = point_on_line if point_on_line is not None else Vector2D(0, 0)
            self.dir = self.make_unique_direction(direction if direction is not None else Vector2D(1, 0))

    def evaluate(self, t):
        return self.origin + Vector2D(self.dir.x * t, self.dir.y * t)

    def get_versor(self):
        return self.dir

    def slope(self):
        if abs(self.dir.x) < 1e-9:
            return 0.0
        return self.dir.y / self.dir.x

    def make_unique_direction(self, v: Vector2D):
        norm = v.get_vnorm()
        if norm < 1e-12:
            return Vector2D(1, 0)
        u = Vector2D(v.x / norm, v.y / norm)
        x_zero = abs(u.x) < 1e-9
        if u.x < 0 or (x_zero and u.y < 0):
            u.x = -u.x
            u.y = -u.y
        return u
