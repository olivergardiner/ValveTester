#include "analyser.h"

Analyser::Analyser()
{
    sampleMatcher = new QRegularExpression(R"(^OK: Mode\(2\) (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+))");
}

Sample *Analyser::createSample(QString response)
{
    QRegularExpressionMatch match = sampleMatcher->match(response);

    double vg1 = convertMeasuredVoltage(GRID, match.captured(3).toInt());
    double va = convertMeasuredVoltage(ANODE, match.captured(4).toInt());
    double ia = convertMeasuredCurrent(ANODE, match.captured(5).toInt(), match.captured(6).toInt()) * 1000;
    double vg2 = convertMeasuredVoltage(SCREEN, match.captured(8).toInt());
    double ig2 = convertMeasuredCurrent(SCREEN, match.captured(9).toInt(), match.captured(10).toInt()) * 1000;
    double vh = convertMeasuredVoltage(HEATER, match.captured(1).toInt());
    double ih = convertMeasuredCurrent(HEATER, match.captured(2).toInt());

    if (ia > iaMax) {
        iaMax = ia;
    }

    if (ig2 > ig2Max) {
        ig2Max = ig2;
    }

    return new Sample(vg1, va, ia, vg2, ig2, vh, ih);
}

int Analyser::convertTargetVoltage(int electrode, double voltage)
{
    int value = 0;

    switch (electrode) {
    case HEATER:
        value = (voltage * 1023 * 470 / 3770 / vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        value = (voltage * 1023 * 9400 / 1419400 / vRefMaster);
        break;
    case GRID:
        value = (voltage * 4095 / 16.5 / vRefMaster);
        break;
    default:
        break;
    }

    return value;
}

double Analyser::convertMeasuredVoltage(int electrode, int voltage)
{
    double value = 0;

    switch (electrode) {
    case HEATER:
        value = (((double) voltage) / 1023 / 470 * 3770 * vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        value = (((double) voltage) / 1023 / 9400 * 1419400 * vRefMaster);
        break;
    case GRID:
        value = -(((double) voltage) / 4095 * 16.5 * vRefMaster);
        break;
    default:
        break;
    }

    return value;
}

double Analyser::convertMeasuredCurrent(int electrode, int current, int currentLo)
{
    double value = 0;
    double voltageHi;

    switch (electrode) {
    case HEATER:
        value = (((double) current) / 1023 / 0.22 * vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        voltageHi = ((double) current) / 1023 / 2.0 * vRefMaster;
        if (voltageHi < 1.9) { // If we're close to 3 diode drops we should use the Lo value
            value = voltageHi / 33.333333;
        } else {
            value = (((double) currentLo) / 1023 / 2.0 * vRefMaster / 3.333333);
        }
        break;
    case GRID:
        break;
    default:
        break;
    }

    return value;
}

void Analyser::reset()
{
    iaMax = 0.0;
    ig2Max = 0.0;
}

double Analyser::getIaMax() const
{
    return iaMax;
}

double Analyser::getIg2Max() const
{
    return ig2Max;
}
