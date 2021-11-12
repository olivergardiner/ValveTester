#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <QJsonObject>
#include <QString>

class Template
{
public:
    Template();

    void read(QJsonObject tpl);

    const QString &getName() const;
    int getDeviceType() const;
    int getTestType() const;
    double getMu() const;
    double getKg() const;
    double getKp() const;
    double getAlpha() const;
    double getVct() const;
    double getKvb() const;
    double getKvb2() const;

    double getVHeater() const;
    double getVaStart() const;
    double getVaStop() const;
    double getVaStep() const;
    double getVgStart() const;
    double getVgStop() const;
    double getVgStep() const;
    double getVsStart() const;
    double getVsStop() const;
    double getVsStep() const;
    double getIaMax() const;
    double getPaMax() const;    

private:
    double mu;
    double kg;
    double kp;
    double alpha;
    double vct;
    double kvb;
    double kvb2;

    double vHeater;
    double vaStart;
    double vaStop;
    double vaStep;
    double vgStart;
    double vgStop;
    double vgStep;
    double vsStart;
    double vsStop;
    double vsStep;
    double iaMax;
    double paMax;

    QString name;
    int deviceType;
    int testType;
};

#endif // TEMPLATE_H
