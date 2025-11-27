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
double opposite_angle_vector(Vector2D adj_1, Vector2D adj_2, Vector2D opposite);
vector<double> get_all_angles(double a, double b, double c);
Vector2D get_avg_center(class vector<class Vector2D> vectors);
bool close_to(double a, double c, double e_tol);
double sign(double value);

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
    Vector2D operator*(const double& x);

    double get_vnorm() const;
    double get_vdegree() const;
    double get_x() const;
    double get_y() const;
    static double evaluate_vnorm(double x, double y);
    void print_vector();
    Vector2D operator-(const Vector2D& vector) const;
    Vector2D operator+(const Vector2D& vector) const;

};

class LineParam {
    public:
        Vector2D origin;   // un punto sulla retta
        Vector2D dir;      // direzione UNIVOCAMENTE normalizzata
    // =======================================
    //   COSTRUTTORE GENERALE
    // =======================================
    LineParam(const Vector2D& point_on_line, const Vector2D& direction);

    // =======================================
    //   COSTRUTTORE: orig = (0,0)
    // =======================================
    LineParam(const Vector2D& direction);

    // =======================================
    //   Calcola punto sulla retta
    // =======================================
    Vector2D evaluate(double t) const;

    // =======================================
    //   Restituisce il versore (già unico)
    // =======================================
    Vector2D get_versor() const;

    // =======================================
    //   Coefficiente angolare (gestisce verticali)
    // =======================================
    double slope() const;

    private:

    // ==============================================================
    //   NORMALIZZAZIONE UNIVOCA DEL VETTORE DIREZIONE
    // ==============================================================

    /**
     *   make_unique_direction(v)
     *
     *   1. normalizza v → versore u
     *   2. applica convenzione di univocità:
     *
     *      - u.x deve essere >= 0
     *      - se u.x == 0, allora u.y >= 0
     *
     *   Inverti u se non rispetta le condizioni.
     *
     *   In questo modo tutti i vettori paralleli → stessa forma canonica.
     */
    Vector2D make_unique_direction(const Vector2D& v) const;



};


#endif  // LINALG_H
