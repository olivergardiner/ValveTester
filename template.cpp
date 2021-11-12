#include "template.h"
#include "valveanalyser.h"

Template::Template()
{
   mu = 70.0;
   kg = 1.0;
   kp = 500.0;
   alpha = 1.5;
   vct = 0.5;
   kvb = 300.0;
   kvb2 = 20.0;

   vHeater = 0.0;
   vaStart = 0.0;
   vaStop = 200.0;
   vaStep = 0.0;
   vgStart = 0.0;
   vgStop = 2.0;
   vgStep = 1.0;
   vsStart = 0.0;
   vsStop = 0.0;
   vsStep = 0.0;
   iaMax = 1.0;
   paMax = 1.0;

   name = "Default";
   deviceType = TRIODE;
   testType = ANODE_CHARACTERISTICS;
}

void Template::read(QJsonObject tpl)
{
    if (tpl.contains("name") && tpl["name"].isString()) {
        name = tpl["name"].toString();
    }
    if (tpl.contains("device_type") && tpl["device_type"].isDouble()) {
        deviceType = tpl["device_type"].toInt();
    }
    if (tpl.contains("test_type") && tpl["test_type"].isDouble()) {
        testType = tpl["test_type"].toInt();
    }
    if (tpl.contains("mu_exp") && tpl["mu_exp"].isDouble()) {
        mu = tpl["mu_exp"].toDouble();
    }
    if (tpl.contains("kg_exp") && tpl["kg_exp"].isDouble()) {
        kg = tpl["kg_exp"].toDouble();
    }
    if (tpl.contains("kp_exp") && tpl["kp_exp"].isDouble()) {
        kp = tpl["kp_exp"].toDouble();
    }
    if (tpl.contains("alpha_exp") && tpl["alpha_exp"].isDouble()) {
        alpha = tpl["alpha_exp"].toDouble();
    }
    if (tpl.contains("vct_exp") && tpl["vct_exp"].isDouble()) {
        vct = tpl["vct_exp"].toDouble();
    }
    if (tpl.contains("kvb_exp") && tpl["kvb_exp"].isDouble()) {
        kvb = tpl["kvb_exp"].toDouble();
    }
    if (tpl.contains("kvb2_exp") && tpl["kvb2_exp"].isDouble()) {
        kvb2 = tpl["kvb2_exp"].toDouble();
    }
    if (tpl.contains("v_heater") && tpl["v_heater"].isDouble()) {
        vHeater = tpl["v_heater"].toDouble();
    }
    if (tpl.contains("va_start") && tpl["va_start"].isDouble()) {
        vaStart = tpl["va_start"].toDouble();
    }
    if (tpl.contains("va_stop") && tpl["va_stop"].isDouble()) {
        vaStop = tpl["va_stop"].toDouble();
    }
    if (tpl.contains("va_step") && tpl["va_step"].isDouble()) {
        vaStep = tpl["va_step"].toDouble();
    }
    if (tpl.contains("vg_start") && tpl["vg_start"].isDouble()) {
        vgStart = tpl["vg_start"].toDouble();
    }
    if (tpl.contains("vg_stop") && tpl["vg_stop"].isDouble()) {
        vgStop = tpl["vg_stop"].toDouble();
    }
    if (tpl.contains("vg_step") && tpl["vg_step"].isDouble()) {
        vgStep = tpl["vg_step"].toDouble();
    }
    if (tpl.contains("vs_start") && tpl["vs_start"].isDouble()) {
        vsStart = tpl["vs_start"].toDouble();
    }
    if (tpl.contains("vs_stop") && tpl["vs_stop"].isDouble()) {
        vsStop = tpl["vs_stop"].toDouble();
    }
    if (tpl.contains("vs_step") && tpl["vs_step"].isDouble()) {
        vsStep = tpl["vs_step"].toDouble();
    }
    if (tpl.contains("ia_max") && tpl["ia_max"].isDouble()) {
        iaMax = tpl["ia_max"].toDouble();
    }
    if (tpl.contains("pa_max") && tpl["pa_max"].isDouble()) {
        paMax = tpl["pa_max"].toDouble();
    }
}

const QString &Template::getName() const
{
    return name;
}

int Template::getDeviceType() const
{
    return deviceType;
}

int Template::getTestType() const
{
    return testType;
}

double Template::getMu() const
{
    return mu;
}

double Template::getKg() const
{
    return kg;
}

double Template::getKp() const
{
    return kp;
}

double Template::getAlpha() const
{
    return alpha;
}

double Template::getVct() const
{
    return vct;
}

double Template::getKvb() const
{
    return kvb;
}

double Template::getKvb2() const
{
    return kvb2;
}

double Template::getVaStart() const
{
    return vaStart;
}

double Template::getVaStop() const
{
    return vaStop;
}

double Template::getVaStep() const
{
    return vaStep;
}

double Template::getVgStart() const
{
    return vgStart;
}

double Template::getVgStop() const
{
    return vgStop;
}

double Template::getVgStep() const
{
    return vgStep;
}

double Template::getVsStart() const
{
    return vsStart;
}

double Template::getVsStop() const
{
    return vsStop;
}

double Template::getVsStep() const
{
    return vsStep;
}

double Template::getIaMax() const
{
    return iaMax;
}

double Template::getPaMax() const
{
    return paMax;
}

double Template::getVHeater() const
{
    return vHeater;
}
