#include "command.h"
#include "valveanalyser.h"

Command::Command(QString command, void (ValveAnalyser::*read)(QString), void (ValveAnalyser::*timeout)())
{
    this->command = command;
    this->responseCallback = read;
    this->timeoutCallback = timeout;
}

void Command::send(ValveAnalyser *analyser)
{
    analyser->sendCommand(command, responseCallback, timeoutCallback);
}
