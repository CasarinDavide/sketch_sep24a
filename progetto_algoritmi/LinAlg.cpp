#include "LinAlg.h"
namespace LinAlg {

// --- Funzioni matematiche ---
double to_radians(double degree) {
    return degree * (PI / 180.0f);
}

double sign(double value)
{ 
    return value > 0?1:-1; 
}

double to_degree(double angle) {
    return angle * (180.0f / PI);
}

double distance_between_vectors(double a, double b, double alpha) {
    return sqrt((a * a) + (b * b) - (2 * a * b * cos(to_radians(alpha))));
}

Vector2D distance_between_vectors(Vector2D a, Vector2D b) {
    return a - b;
}

Vector2D to_vector(double d, double alpha) {
    return Vector2D(d * cos(to_radians(alpha)), d * sin(to_radians(alpha)));
}

double opposite_angle(double adj_1, double adj_2, double opposite) {
    return acos(((adj_1 * adj_1) + (adj_2 * adj_2) - (opposite * opposite)) / (2 * adj_1 * adj_2));
}


double opposite_angle_vector(Vector2D adj_1, Vector2D adj_2, Vector2D opposite) {
    return acos(((adj_1.get_vnorm()) + (adj_2.get_vnorm()) - (opposite.get_vnorm())) / (2 * sqrt(adj_1.get_vnorm()) * sqrt(adj_2.get_vnorm()) ));
}



vector<double> get_all_angles(double a, double b, double c) {
    vector<double> r;
    r.push_back(opposite_angle(a, b, c));
    r.push_back(opposite_angle(b, c, a));
    r.push_back(opposite_angle(c, a, b));
    return r;
}

Vector2D get_avg_center(vector<Vector2D> vectors) {
    double avg_x = 0, avg_y = 0;
    int size = vectors.size();
    for (size_t i = 0; i < size; ++i) {
        avg_x += vectors[i].x;
        avg_y += vectors[i].y;
    }
    return Vector2D(avg_x / size, avg_y / size);
}

bool close_to(double a, double c, double e_tol) {
    return abs(a - c) < e_tol;
}

// --- Vector2D ---
Vector2D::Vector2D() : x(0), y(0), v_norm(0), v_degree(0) {}

Vector2D::Vector2D(double x, double y)
    : x(x), y(y), v_norm(evaluate_vnorm(x, y)), v_degree(to_degree(atan2(y, x))) {}

Vector2D::Vector2D(double radius, double alpha, int)
    : x(cos(to_radians(alpha)) * radius),
      y(sin(to_radians(alpha)) * radius),
      v_norm(radius), v_degree(alpha) {}

Vector2D& Vector2D::operator=(const Vector2D& v) {
    if (this != &v) {
        this->x = v.get_x();
        this->y = v.get_y();
        this->v_degree = v.v_degree;
        this->v_norm = v.v_norm;
    }
    return *this;
}

double Vector2D::get_vnorm() const { return v_norm; }
double Vector2D::get_vdegree() const {return v_degree < 0 ? 360+v_degree:v_degree; }
double Vector2D::get_x() const { return x; }
double Vector2D::get_y() const { return y; }

double Vector2D::evaluate_vnorm(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
}

Vector2D Vector2D::operator-(const Vector2D& vector) const {
    return Vector2D(x - vector.x, y - vector.y);
}

Vector2D Vector2D::operator+(const Vector2D& vector) const {
    return Vector2D(x + vector.x, y + vector.y);
}

Vector2D Vector2D::operator*(const double& x) {
    return Vector2D(this->x * x, this->y * x);
}

void Vector2D::print_vector()
{
    Serial.println("PRINT VECTOR");

    Serial.print("X:");
    Serial.print(this->x);

    Serial.println("Y:");
    Serial.print(this->y);

    Serial.println("NORM:");
    Serial.print(this->v_norm);

    Serial.println("ANGLE:");
    Serial.print(this->v_degree);

    Serial.println(" END PRINT VECTOR");
}

LineParam::LineParam(const Vector2D& point_on_line, const Vector2D& direction)
{
    origin = point_on_line;
    dir    = make_unique_direction(direction);
}

LineParam::LineParam(const Vector2D& direction)
        : LineParam(Vector2D(0,0), direction){}

Vector2D LineParam::evaluate(double t) const { return origin + Vector2D(dir.x * t, dir.y * t);}

Vector2D LineParam::get_versor() const {return dir;}

double LineParam::slope() const {
    if (fabs(dir.x) < 1e-9)
        return 0; 
    return dir.y / dir.x;
}

Vector2D LineParam::make_unique_direction(const Vector2D& v) const
{
    double norm = v.get_vnorm();

    // caso direzione nulla → default verso (1,0)
    if (norm < 1e-12)
        return Vector2D(1, 0);

    // versore normale
    Vector2D u(v.x / norm, v.y / norm);

    // unificazione del verso
    // 1) se x < 0, inverti
    // 2) se x == 0 e y < 0, inverti
    bool x_zero = fabs(u.x) < 1e-9;

    if (u.x < 0 || (x_zero && u.y < 0)) {
        u.x = -u.x;
        u.y = -u.y;
    }

    return u;
}


} // namespace LinAlg
