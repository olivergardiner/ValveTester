#ifndef COMMAND_H
#define COMMAND_H

#include<QString>

class ValveAnalyser;

class Command
{
public:
    Command(QString command, void (ValveAnalyser::*read)(QString), void (ValveAnalyser::*timeout)());

    void send(ValveAnalyser *analyser);
private:
    QString command;
    void (ValveAnalyser::* responseCallback)(QString);
    void (ValveAnalyser::* timeoutCallback)();
};

#include "valveanalyser.h"

#endif // COMMAND_H
