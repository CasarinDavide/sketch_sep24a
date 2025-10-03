#include "LinAlg.h"
namespace LinAlg {

// --- Funzioni matematiche ---
double to_radians(double degree) {
    return degree * (PI / 180.0f);
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
    : x(x), y(y), v_norm(evaluate_vnorm(x, y)), v_degree(to_degree(acos(x / v_norm))) {}

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
double Vector2D::get_vdegree() const { return v_degree; }
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

// --- LineParam ---
LineParam::LineParam(const Vector2D& origin, const Vector2D& direction)
    : origin(origin), dir(direction) {}

Vector2D LineParam::evaluate(double t) const {
    return origin + Vector2D(dir.x * t, dir.y * t);
}

double LineParam::slope() const {
    return dir.y / dir.x;
}

double LineParam::intercept() const {
    return origin.y - slope() * origin.x;
}

} // namespace LinAlg
