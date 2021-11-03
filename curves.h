#ifndef CURVES_H
#define CURVES_H

#include "cmpfit/mpfit.h"

#define CRV_MAX_PARAMETERS 12

struct curveData {  /* EXAMPLE: fitting y(x) */
  double *x;         /* x - independent variable of model */
  double *y;         /* y - measured "y" values */
  double *y_error;   /* y_error - measurement uncertainty in y */
  double *g;         /* grid voltage parameter */
};

void setDefaultParameterConstraints(mp_par *constraint);

int fitCurve(int m, int n, double *p, double *deviates, double **derivs, void *data);

double polynomial(double x, int n, double *p);

double simpleTriodeModel(double x, double vg, int n, double *p);

double korenCurrent(double va, double vg, double kp, double alpha, double mu, double kvb);

double korenTriodeModel(double x, double vg, int n, double *p);

double pentodeModel(double x, double vg, int n, double *p);

double diodeModel(double x, double vg, int n, double *p);

#endif // CURVES_H
