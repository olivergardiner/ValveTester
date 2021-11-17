#include "devicemodel.h"

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
struct SimpleTriodeResidual {
    SimpleTriodeResidual(double va, double vg, double ia) : va_(va), vg_(vg), ia_(ia) {}

    template <typename T>
    bool operator()(const T* const kg, const T* const vct, const T* const a, const T* const mu, T* residual) const {
        T e1t = va_ / mu[0] + vg_ + vct[0];
        if (e1t < 0.0) {
            e1t = mu[0] - mu[0];
        }
        T ia = pow(e1t, a[0]) / kg[0];
        residual[0] = ia_ - ia;
        return !(isnan(ia) || isinf(ia));
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
        return !(isnan(ia) || isinf(ia));
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
        return !(isnan(ia) || isinf(ia));
    }

private:
    const double va_;
    const double vg_;
    const double ia_;
};

DeviceModel::DeviceModel(int _modelType, int _model) : modelType(_modelType), model(_model)
{
    if (modelType == MODEL_TRIODE) {
        parameterValue[TRI_KG] = 0.7;
        parameterName[TRI_KG] = "Kg:";
        parameterValue[TRI_KP] = 500.0;
        parameterName[TRI_KP] = "Kp:";
        parameterValue[TRI_KVB] = 300.0;
        parameterName[TRI_KVB] = "Kvb:";
        parameterValue[TRI_KVB2] = 30.0;
        parameterName[TRI_KVB2] = "Kvb2:";
        parameterValue[TRI_VCT] = 0.01;
        parameterName[TRI_VCT] = "Vct:";
        parameterValue[TRI_ALPHA] = 1.5;
        parameterName[TRI_ALPHA] = "Alpha:";
        parameterValue[TRI_MU] = 100.0;
        parameterName[TRI_MU] = "Mu:";
    }
}

void DeviceModel::addTriodeSample(double va, double vg1, double ia)
{
    if (model == SIMPLE_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<SimpleTriodeResidual, 1, 1, 1, 1, 1>(
                new SimpleTriodeResidual(va, vg1, ia)),
            NULL,
            &parameterValue[TRI_KG],
            &parameterValue[TRI_VCT],
            &parameterValue[TRI_ALPHA],
            &parameterValue[TRI_MU]);
    } else if (model == KOREN_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<KorenTriodeResidual, 1, 1, 1, 1, 1, 1>(
                new KorenTriodeResidual(va, vg1, ia)),
            NULL,
            &parameterValue[TRI_KG],
            &parameterValue[TRI_KP],
            &parameterValue[TRI_KVB],
            &parameterValue[TRI_ALPHA],
            &parameterValue[TRI_MU]);
    } else if (model == IMPROVED_KOREN_TRIODE) {
        problem.AddResidualBlock(
            new AutoDiffCostFunction<ImprovedKorenTriodeResidual, 1, 1, 1, 1, 1, 1, 1, 1>(
                new ImprovedKorenTriodeResidual(va, vg1, ia)),
            NULL,
            &parameterValue[TRI_KG],
            &parameterValue[TRI_KP],
            &parameterValue[TRI_KVB],
            &parameterValue[TRI_KVB2],
            &parameterValue[TRI_VCT],
            &parameterValue[TRI_ALPHA],
            &parameterValue[TRI_MU]);
    }
}

void DeviceModel::addPentodeSample(double va, double vg1, double vg2, double ia)
{

}

void DeviceModel::solve()
{
    Solver::Options options;

    if (model == SIMPLE_TRIODE) {
        problem.SetParameterLowerBound(&parameterValue[TRI_KG], 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&parameterValue[TRI_ALPHA], 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_ALPHA], 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&parameterValue[TRI_MU], 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_MU], 0, 1000.0); // mu <= 1000
        problem.SetParameterLowerBound(&parameterValue[TRI_VCT], 0, -2.0); // Vct >= -2.0
        problem.SetParameterUpperBound(&parameterValue[TRI_VCT], 0, 2.0); // Vct <= 2.0
    } else if (model == KOREN_TRIODE) {
        problem.SetParameterLowerBound(&parameterValue[TRI_KG], 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&parameterValue[TRI_KP], 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&parameterValue[TRI_ALPHA], 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_ALPHA], 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&parameterValue[TRI_MU], 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_MU], 0, 1000.0); // mu <= 1000
        problem.SetParameterLowerBound(&parameterValue[TRI_KVB], 0, 0.0); // Kvb >= 0
        problem.SetParameterUpperBound(&parameterValue[TRI_KVB], 0, 10000.0); // Kvb <= 10000
        options.linear_solver_type = ceres::CGNR;
        options.preconditioner_type = ceres::JACOBI;
    } else if (model == IMPROVED_KOREN_TRIODE) {
        problem.SetParameterLowerBound(&parameterValue[TRI_KG], 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&parameterValue[TRI_KP], 0, 0.0000001); // Kg > 0
        problem.SetParameterLowerBound(&parameterValue[TRI_ALPHA], 0, 1.0); // a >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_ALPHA], 0, 2.0); // a <= 2.0
        problem.SetParameterLowerBound(&parameterValue[TRI_MU], 0, 1.0); // mu >= 1.0
        problem.SetParameterUpperBound(&parameterValue[TRI_MU], 0, 1000.0); // mu <= 1000
        problem.SetParameterLowerBound(&parameterValue[TRI_KVB], 0, 0.0); // Kvb >= 0
        problem.SetParameterUpperBound(&parameterValue[TRI_KVB], 0, 10000.0); // Kvb <= 10000
        problem.SetParameterLowerBound(&parameterValue[TRI_KVB2], 0, 0.0); // Kvb2 >= 0
        problem.SetParameterUpperBound(&parameterValue[TRI_KVB2], 0, 1000.0); // Kvb2 <= 1000
        problem.SetParameterLowerBound(&parameterValue[TRI_VCT], 0, -2.0); // Vct >= -2.0
        problem.SetParameterUpperBound(&parameterValue[TRI_VCT], 0, 2.0); // Vct <= 2.0
        options.linear_solver_type = ceres::CGNR;
        options.preconditioner_type = ceres::JACOBI;
    }

    options.max_num_iterations = 100;
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
        double e1t = va / parameterValue[TRI_MU] + vg1 + parameterValue[TRI_VCT];
        if (e1t > 0) {
            ia = pow(e1t, parameterValue[TRI_ALPHA]) / parameterValue[TRI_KG];
        }
    } else if (model == KOREN_TRIODE) {
        ia = korenCurrent(va, vg1, parameterValue[TRI_KP], parameterValue[TRI_KVB], parameterValue[TRI_ALPHA], parameterValue[TRI_MU]) / parameterValue[TRI_KG];
    } else if (model == IMPROVED_KOREN_TRIODE) {
        ia = improvedKorenCurrent(va, vg1, parameterValue[TRI_KP], parameterValue[TRI_KVB], parameterValue[TRI_KVB2], parameterValue[TRI_VCT], parameterValue[TRI_ALPHA], parameterValue[TRI_MU]) / parameterValue[TRI_KG];
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

double DeviceModel::getParameter(int index) const
{
    return parameterValue[index];
}

void DeviceModel::updateUI(QLabel *labels[], QLineEdit *values[])
{
    if (model == SIMPLE_TRIODE) {
        updateParameter(labels[0], values[0], parameterName[TRI_MU], parameterValue[TRI_MU]);
        updateParameter(labels[1], values[1], parameterName[TRI_KG], parameterValue[TRI_KG]);
        updateParameter(labels[2], values[2], parameterName[TRI_ALPHA], parameterValue[TRI_ALPHA]);
        updateParameter(labels[3], values[3], parameterName[TRI_VCT], parameterValue[TRI_VCT]);
    } else if (model == KOREN_TRIODE) {
        updateParameter(labels[0], values[0], parameterName[TRI_MU], parameterValue[TRI_MU]);
        updateParameter(labels[1], values[1], parameterName[TRI_KG], parameterValue[TRI_KG]);
        updateParameter(labels[2], values[2], parameterName[TRI_ALPHA], parameterValue[TRI_ALPHA]);
        updateParameter(labels[3], values[3], parameterName[TRI_KVB], parameterValue[TRI_KVB]);
        updateParameter(labels[4], values[4], parameterName[TRI_KP], parameterValue[TRI_KP]);
    } else if (model == IMPROVED_KOREN_TRIODE) {
        updateParameter(labels[0], values[0], parameterName[TRI_MU], parameterValue[TRI_MU]);
        updateParameter(labels[1], values[1], parameterName[TRI_KG], parameterValue[TRI_KG]);
        updateParameter(labels[2], values[2], parameterName[TRI_ALPHA], parameterValue[TRI_ALPHA]);
        updateParameter(labels[3], values[3], parameterName[TRI_VCT], parameterValue[TRI_VCT]);
        updateParameter(labels[4], values[4], parameterName[TRI_KP], parameterValue[TRI_KP]);
        updateParameter(labels[5], values[5], parameterName[TRI_KVB], parameterValue[TRI_KVB]);
        updateParameter(labels[6], values[6], parameterName[TRI_KVB2], parameterValue[TRI_KVB2]);
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
