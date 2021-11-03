#include "curves.h"

#include <math.h>

double (*curveFunction)(double va, double vg, int n, double *p);

void setDefaultParameterConstraints(mp_par *constraint) {
    constraint->fixed = false;
    constraint->limited[0] = false;
    constraint->limited[1] = false;
    constraint->step = 0.0;
    constraint->relstep = 0.0;
    constraint->side = 0;
    constraint->deriv_debug = false;
    constraint->deriv_abstol = 0.0;
    constraint->deriv_reltol = 0.0;
}

int fitCurve(int m, int n, double *p, double *deviates, double **derivs, void *dataPtr)
{
  int i;
  double *x, *y, *y_error;
  double *g;

  struct curveData *data = (struct curveData *) dataPtr;

  /* Retrieve values of x, y and y_error from private structure */
  x = data->x;
  y = data->y;
  y_error = data->y_error;
  g = data->g;

  /* Compute function deviates */
  for (i=0; i<m; i++) {    
    deviates[i] = (y[i] - curveFunction(x[i], g[i], n, p)) / y_error[i];
  }

  return 0;
}

double polynomial(double x, int n, double *p) {
    double value = p[0];

    for (int i=1; i < n; i++) {
        value += p[i] * pow(x, i);
    }

    return value;
}

double simpleTriodeModel(double va, double vg, int n, double *p) {
    double ig = p[0]; // Grid current
    double g = p[1]; // G (Perveance)
    double alpha = p[2]; // Power law constant - 3/2 for ideal spherical geometry
    double mu = p[3];

    double et = (va / mu) + vg;
    if (et < 0.0) { // if we're below the threshold voltage, no current flows
        return 0.0;
    }

    double ia = ig + g * pow(et, alpha);

    return ia;
}

double korenCurrent(double va, double vg, double kp, double alpha, double mu, double kvb) {
    double x1 = sqrt(kvb + va * va);
    double x2 = 1 / mu + vg / x1;
    double x3 = exp(kp * x2);
    double x4 = log (1.0 + x3);
    double et = (va / kp) * x4;
    // double et = (va / kp) * log(1 + exp(kp * (1 / mu + vg / sqrt(kvb + va * va))));

    if (et < 0.0) {
        et = 0.0;
    }

    return pow(et, alpha);
}

double korenTriodeModel(double va, double vg, int n, double *p) {
    double kvb = p[0]; // Remote cutoff term
    double kp = p[1]; // Replaces simple measure of perveance for scaling
    double alpha = p[2]; // Power law constant - 3/2 for ideal spherical geometry
    double mu = p[3];
    double kg = p[4];

    double ik = korenCurrent(va, vg, kp, alpha, mu, kvb) / kg;

    return ik;
}

double pentodeModel(double x, double vg, int n, double *p) {
    double value = polynomial(x, n - 3, p); // start with a polynomial, reserving three terms

    value += p[n - 1] * pow(p[n - 2], x + p[n - 3]); // Additional term is a . b ^ (x + c)

    return value;
}

double diodeModel(double x, double vg, int n, double *p) {
    double value = polynomial(x, n - 3, p); // start with a polynomial, reserving three terms

    value += p[n - 1] * pow(p[n - 2], x + p[n - 3]); // Additional term is a . b ^ (x + c)

    return value;
}
