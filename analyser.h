#ifndef ANALYSER_H
#define ANALYSER_H

#include <QRegularExpression>

#include "sample.h"


class Analyser
{
public:
    Analyser();

    Sample *createSample(QString response);
    int convertTargetVoltage(int electrode, double voltage);
    double convertMeasuredVoltage(int electrode, int voltage);
    double convertMeasuredCurrent(int electrode, int current, int currentLo = 0);
    void reset(void);

    enum eElectrode {
        HEATER,
        ANODE,
        GRID,
        SCREEN
    };

    double getIaMax() const;

    double getIg2Max() const;

private:
    QRegularExpression *sampleMatcher;
    double vRefMaster = 4.096;
    double vRefSlave = 2.048;
    double iaMax = 0.0;
    double ig2Max = 0.0;
};

#endif // ANALYSER_H
