#ifndef LINALG_H
#define LINALG_H

#include "Algo.h"
#include <math.h>
#include <Arduino.h>


#ifndef PI
#define PI 3.14159265358979323846
#endif

using namespace Algo;

namespace LinAlg {

// === DICHIARAZIONI FUNZIONI ===
double to_degree(double angle);
double to_radians(double degree);
double distance_between_vectors(double a, double b, double alpha);
class Vector2D distance_between_vectors(class Vector2D a, class Vector2D b);
class Vector2D to_vector(double d, double alpha);
double opposite_angle(double adj_1, double adj_2, double opposite);
vector<double> get_all_angles(double a, double b, double c);
Vector2D get_avg_center(class vector<class Vector2D> vectors);
bool close_to(double a, double c, double e_tol);

// === Classe Vector2D ===
class Vector2D {
public:
    double x;
    double y;
    double v_norm;
    double v_degree;

    Vector2D();
    Vector2D(double x, double y);
    Vector2D(double radius, double alpha, int); // polare

    Vector2D& operator=(const Vector2D& v);

    double get_vnorm() const;
    double get_vdegree() const;
    double get_x() const;
    double get_y() const;
    static double evaluate_vnorm(double x, double y);
    Vector2D operator-(const Vector2D& vector) const;
    Vector2D operator+(const Vector2D& vector) const;
};

// === Classe LineParam ===
class LineParam {
    Vector2D origin;
    Vector2D dir;

public:
    LineParam(const Vector2D& origin, const Vector2D& direction);
    Vector2D evaluate(double t) const;
    double slope() const;
    double intercept() const;
};

}


#endif  // LINALG_H
