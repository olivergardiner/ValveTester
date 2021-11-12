#ifndef DEVICEMODEL_H
#define DEVICEMODEL_H

#include <QLabel>
#include <QLineEdit>

#include "ceres/ceres.h"
#include "glog/logging.h"

#include "template.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

enum eTriodeModel{
    SIMPLE_TRIODE,
    KOREN_TRIODE,
    IMPROVED_KOREN_TRIODE
};

enum ePentodeModel{
    KOREN_PENTODE,
    DERK_PENTODE,
    DERKE_PENTODE
};

enum eModelType{
    MODEL_TRIODE,
    MODEL_PENTODE
};

class DeviceModel
{
public:
    DeviceModel(int _modelType, int _model, Template tpl);

    void addTriodeSample(double va, double vg1, double ia);
    void addPentodeSample(double va, double vg1, double vg2, double ia);
    void solve();

    double anodeCurrent(double va, double vg1);
    double anodeCurrent(double va, double vg1, double vg2);

    double getCKg() const;
    double getCKp() const;
    double getCAlpha() const;
    double getCVct() const;
    double getCKvb() const;
    double getCKvb2() const;
    double getCMu() const;

    void updateUI(QLabel *labels[], QLineEdit *values[]);

private:
    Problem problem;
    int modelType = MODEL_TRIODE;
    int model = IMPROVED_KOREN_TRIODE;

    // Model variables
    double cKg;
    double cKp;
    double cAlpha;
    double cVct;
    double cKvb;
    double cKvb2;
    double cMu;

    double korenCurrent(double va, double vg, double kp, double kvb, double a, double mu);
    double improvedKorenCurrent(double va, double vg, double kp, double kvb, double kvb2, double vct, double a, double mu);

    void updateParameter(QLabel *label, QLineEdit *value, QString name, double x);
};

#endif // DEVICEMODEL_H
