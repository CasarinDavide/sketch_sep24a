
#ifndef LINALG_H
#define LINALG_H

#include "Algo.h"

using namespace Algo;

#ifndef PI
#define PI 3.14159265358979323846
#endif

namespace LinAlg {

double to_degree(double angle);

double to_radians(double degree);
// ======= Classe Vector2D =======
class Vector2D {
public:
  double x;
  double y;
  double v_norm;
  double v_degree;  // degree from 0-axis to x,y

  Vector2D() {}

  // Costruttore da coordinate cartesiane
  Vector2D(double x, double y)
    : x(x), y(y), v_norm(evaluate_vnorm(x, y)), v_degree(to_degree(acos(x / v_norm) )) {}

  Vector2D& operator=(const Vector2D& v) {
    if (this != &v) {
      this->x = v.get_x();
      this->y = v.get_y();
      this->v_degree = v.v_degree;
      this->v_norm = v.v_norm;
    }

    return *this;
  }

  // Costruttore da coordinate polari
  Vector2D(double radius, double alpha, int)
    : x(cos(to_radians(alpha)) * radius),
      y(sin(to_radians(alpha)) * radius),
      v_norm(radius), v_degree(alpha) {
        
        Serial.println("INCOERENTE");
        Serial.println(alpha);
        Serial.println(to_radians(alpha));
        Serial.println(this->x);
        Serial.println(this->y);
        
      }

  double get_vnorm() const {
    return v_norm;
  }

  double get_vdegree() const {
    return v_degree;
  }

  double get_x() const {
    return x;
  }
  double get_y() const {
    return y;
  }

  static double evaluate_vnorm(double x, double y) {
    return sqrt(pow(x, 2) + pow(y, 2));
  }

  Vector2D operator-(const Vector2D& vector) const {
    return Vector2D(x - vector.x, y - vector.y);
  }

  Vector2D operator+(const Vector2D& vector) const {
    return Vector2D(x + vector.x, y + vector.y);
  }
};

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

vector<double> all_angles(double a, double b, double c) {
  vector<double> r;

  r.push_back(opposite_angle(a, b, c));
  r.push_back(opposite_angle(b, c, a));
  r.push_back(opposite_angle(c, a, b));

  return r;
}


  Vector2D get_avg_center(vector<Vector2D> vectors) {
    float avg_x, avg_y;
    int size = vectors.size();
    for (size_t i = 0; i < size; ++i) {
      avg_x += vectors[i].x;
      avg_y += vectors[i].y;

       Serial.print("avg_x");
      Serial.print(vectors[i].x);
      Serial.print("avg_y");
      Serial.print(vectors[i].y);
      Serial.print(size);
    }

   
    
    return Vector2D(avg_x / size, avg_y / size);
  }
}

#endif  // LINALG_H
