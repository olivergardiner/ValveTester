#include "devicemodel.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
struct SimpleTriodeResidual {
    SimpleTriodeResidual(double va, double vg, double ia) : va_(va), vg_(vg), ia_(ia) {}

    template <typename T>
    bool operator()(const T* const kg, const T* const vct, const T* const a, const T* const mu, T* residual) const {
        T e1t = va_ / mu[0] + vg_ + vct[0];
        T ia = (1 + sgn(e1t)) * 0.5 * pow(e1t, a[0]) / kg[0];
        residual[0] = ia_ - ia;
        return true;
    }

private:
    const double va_;
    const double vg_;
    const double ia_;
};

struct KorenTriodeResidual {
    KorenTriodeResidual(double va, double vg, double ia) : va_(va), vg_(vg), ia_(ia) {}

    template <typename T>
    bool operator()(const T* const kg, const T* const kp, const T* const kvb, const T* const a, const T* const mu, T* residual) const {
        T e1t = log(1.0 + exp(kp[0] * (1.0 / mu[0] + vg_ / sqrt(kvb[0] + va_ * va_))));
        T ia = pow((va_ / kp[0]) * e1t, a[0]) / kg[0];
        residual[0] = ia_ - ia;
        return true;
    }

private:
    const double va_;
    const double vg_;
    const double ia_;
};

struct ImprovedKorenTriodeResidual {
    ImprovedKorenTriodeResidual(double va, double vg, double ia) : va_(va), vg_(vg), ia_(ia) {}

    template <typename T>
    bool operator()(const T* const kg, const T* const kp, const T* const kvb, const T* const kvb2, const T* const vct, const T* const a, const T* const mu, T* residual) const {
        T e2t = log(1.0 + exp(kp[0] * (1.0 / mu[0] + (vg_ + vct[0])/ sqrt(kvb[0] + va_ * va_ + kvb2[0] * va_))));
        T ia = pow((va_ / kp[0]) * e2t, a[0]) / kg[0];
        residual[0] = ia_ - ia;
        return true;
    }

private:
    const double va_;
    const double vg_;
    const double ia_;
};

DeviceModel::DeviceModel(int _modelType, int _model, Template tpl) : modelType(_modelType), model(_model)
{
    // Set default model parameter values - only mu taken from the template currently
    cKg = 0.7;
    cKp = 500.0;
    cKvb = 300.0;
    cKvb2 = 30.0;
    cVct = 0.01;
    cAlpha = 1.5;
    cMu = tpl.getMu();
}

void DeviceModel::addTriodeSample(double va, double vg1, double ia)
{
    if (model == SIMPLE_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<SimpleTriodeResidual, 1, 1, 1, 1, 1>(
                new SimpleTriodeResidual(va, vg1, ia)),
            NULL,
            &cKg, &cVct, &cAlpha, &cMu);
    } else if (model == KOREN_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<KorenTriodeResidual, 1, 1, 1, 1, 1, 1>(
                new KorenTriodeResidual(va, vg1, ia)),
            NULL,
            &cKg, &cKp, &cKvb, &cAlpha, &cMu);
    } else if (model == IMPROVED_KOREN_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<ImprovedKorenTriodeResidual, 1, 1, 1, 1, 1, 1, 1, 1>(
                new ImprovedKorenTriodeResidual(va, vg1, ia)),
            NULL,
            &cKg, &cKp, &cKvb, &cKvb2, &cVct, &cAlpha, &cMu);      
    }
}

void DeviceModel::solve()
{
    Solver::Options options;

    if (model == SIMPLE_TRIODE) {
        problem.SetParameterLowerBound(&cKg, 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&cAlpha, 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&cAlpha, 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&cMu, 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&cMu, 0, 1000.0); // mu <= 1000
        problem.SetParameterLowerBound(&cVct, 0, -2.0); // Vct >= -2.0
        problem.SetParameterUpperBound(&cVct, 0, 2.0); // Vct <= 2.0
    } else if (model == KOREN_TRIODE) {
        problem.SetParameterLowerBound(&cKg, 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&cKp, 0, 0.0000001); // Kp > 0
        problem.SetParameterLowerBound(&cKvb, 0, 0.0); // Kvb >= 0
        problem.SetParameterUpperBound(&cKvb, 0, 10000.0); // Kvb <= 10000
        problem.SetParameterLowerBound(&cAlpha, 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&cAlpha, 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&cMu, 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&cMu, 0, 1000.0); // mu <= 1000
    } else if (model == IMPROVED_KOREN_TRIODE) {
        problem.SetParameterLowerBound(&cKg, 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&cKp, 0, 0.0000001); // Kp > 0
        problem.SetParameterLowerBound(&cKvb, 0, 0.0); // Kvb >= 0
        problem.SetParameterUpperBound(&cKvb, 0, 10000.0); // Kvb <= 10000
        problem.SetParameterLowerBound(&cAlpha, 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&cAlpha, 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&cMu, 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&cMu, 0, 1000.0); // mu <= 1000
        problem.SetParameterLowerBound(&cKvb2, 0, 0.0); // Kvb2 >= 0
        problem.SetParameterUpperBound(&cKvb2, 0, 1000.0); // Kvb2 <= 1000
        problem.SetParameterLowerBound(&cVct, 0, -2.0); // Vct >= -2.0
        problem.SetParameterUpperBound(&cVct, 0, 2.0); // Vct <= 2.0
    }

    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::CGNR;
    options.preconditioner_type = ceres::JACOBI;
    options.minimizer_progress_to_stdout = true;
    Solver::Summary summary;
    Solve(options, &problem, &summary);

    qInfo(summary.BriefReport().c_str());
}

double DeviceModel::anodeCurrent(double va, double vg1)
{
    // Only va and vg1 supplied so necessarily a triode
    double ia = 0.0;

    if (model == SIMPLE_TRIODE) {
        double e1t = va / cMu + vg1 + cVct;
        if (e1t > 0) {
            ia = pow(e1t, cAlpha) / cKg;
        }
    } else if (model == KOREN_TRIODE) {
        ia = korenCurrent(va, vg1, cKp, cKvb, cAlpha, cMu) / cKg;
    } else if (model == IMPROVED_KOREN_TRIODE) {
        ia = improvedKorenCurrent(va, vg1, cKp, cKvb, cKvb2, cVct, cAlpha, cMu) / cKg;
    }

    return ia;
}

double DeviceModel::anodeCurrent(double va, double vg1, double vg2)
{
    // Vg2 also provided so the pentode model should be being used
    double ia = 0.0;

    return ia;
}

double DeviceModel::korenCurrent(double va, double vg, double kp, double kvb, double a, double mu)
{
    double x1 = std::sqrt(kvb + va * va);
    double x2 = kp * (1 / mu + vg / x1);
    double x3 = std::log(1.0 + std::exp(x2));
    double et = (va / kp) * x3;

    if (et < 0.0) {
        et = 0.0;
    }

    return pow(et, a);
}

double DeviceModel::improvedKorenCurrent(double va, double vg, double kp, double kvb, double kvb2, double vct, double a, double mu)
{
    double x1 = std::sqrt(kvb + va * va + va * kvb2);
    double x2 = kp * (1 / mu + (vg + vct) / x1);
    double x3 = std::log(1.0 + std::exp(x2));
    double et = (va / kp) * x3;

    if (et < 0.0) {
        et = 0.0;
    }

    return pow(et, a);
}

double DeviceModel::getCKg() const
{
    return cKg;
}

double DeviceModel::getCKp() const
{
    return cKp;
}

double DeviceModel::getCAlpha() const
{
    return cAlpha;
}

double DeviceModel::getCVct() const
{
    return cVct;
}

double DeviceModel::getCKvb() const
{
    return cKvb;
}

double DeviceModel::getCKvb2() const
{
    return cKvb2;
}

double DeviceModel::getCMu() const
{
    return cMu;
}

void DeviceModel::updateUI(QLabel *labels[], QLineEdit *values[])
{
    if (model == SIMPLE_TRIODE) {
        updateParameter(labels[0], values[0], "Mu:", cMu);
        updateParameter(labels[1], values[1], "Kg:", cKg);
        updateParameter(labels[2], values[2], "Alpha:", cAlpha);
        updateParameter(labels[3], values[3], "Vct:", cVct);
    } else if (model == KOREN_TRIODE) {
        updateParameter(labels[0], values[0], "Mu:", cMu);
        updateParameter(labels[1], values[1], "Kg:", cKg);
        updateParameter(labels[2], values[2], "Alpha:", cAlpha);
        updateParameter(labels[3], values[3], "Vct:", cVct);
        updateParameter(labels[4], values[4], "Kp:", cKp);
    } else if (model == IMPROVED_KOREN_TRIODE) {
        updateParameter(labels[0], values[0], "Mu:", cMu);
        updateParameter(labels[1], values[1], "Kg:", cKg);
        updateParameter(labels[2], values[2], "Alpha:", cAlpha);
        updateParameter(labels[3], values[3], "Vct:", cVct);
        updateParameter(labels[4], values[4], "Kp:", cKp);
        updateParameter(labels[5], values[5], "Kvb:", cKvb);
        updateParameter(labels[6], values[6], "Kvb2:", cKvb2);
    }
}

void DeviceModel::updateParameter(QLabel *label, QLineEdit *value, QString name, double x)
{
    label->setText(name);
    label->setVisible(true);

    char number[32];

    sprintf(number, "%.3f", x);

    int length = strlen(number);
    for (int i=length-1;i >= 0; i--) {
        char test = number[i];
        if (test == '0' || test == '.') {
            number[i] = 0;
        }

        if (test != '0') {
            break;
        }
    }

    value->setText(number);
    value->setVisible(true);
}
