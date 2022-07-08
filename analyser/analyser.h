#ifndef ANALYSER_H
#define ANALYSER_H

#include <QRegularExpression>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>

#include "sampleset.h"
#include "constants.h"
#include "client.h"

#define VH       0   //Heater voltage  [example: 12.6V = adc391   6.3V = adc195]
#define IH       1   //Heater current
#define VG1      2   //Grid voltage 1  [example: 1V = dac60, 10V=dac605, 20V=dac1210, 30V=dac1815, 40V=dac2420, 50V=dac3020]
#define HV1      3   //Anode voltage 1 [example: 600V=3.97V=adc992   300V=1.98V=adc496    200V=0.76V=330   100V=0.381V=V=adc165]
#define IA_HI_1  4   //Anode current hi 1
#define IA_LO_1  5   //Anode current lo 1
#define VG2      6   //Grid voltage 2
#define HV2      7   //Anode voltage 2
#define IA_HI_2  8   //Anode current hi 2
#define IA_LO_2  9   //Anode current lo 2

class Analyser
{
public:
    Analyser(Client *client, QSerialPort *port, QTimer *timeoutTimer, QTimer *heaterTimer);
    ~Analyser();

    void reset(void);

    void startTest();
    void stopTest();

    const QString &getHwVersion() const;

    const QString &getSwVersion() const;

    void setTestType(int newTestType);

    void setDeviceType(int newDeviceType);

    void setSweepPoints(int newSweepPoints);

    void setSweepParameters(double aStart, double aStop, double aStep, double gStart, double gStop, double gStep, double sStart, double sStop, double sStep);

    void setPMax(double newPMax);

    void setIaMax(double newIaMax);

    void setIg2Max(double newIg2Max);

    void setHeaterVoltage(double newHeaterVoltage);

    void setIsHeatersOn(bool newIsHeatersOn);

    double getMeasuredIaMax() const;

    double getMeasuredIg2Max() const;

    const QList<QList<Sample *> > *getSweepResult() const;

    SampleSet *getResult();

    bool getIsDataSetValid() const;

    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError error);

    void handleCommandTimeout();
    void handleHeaterTimeout();

private:
    double vRefMaster = 4.096;
    double vRefSlave = 2.048;

    double heaterVoltage;

    double anodeStart;
    double anodeStop;
    double anodeStep;

    double gridStart;
    double gridStop;
    double gridStep;

    double screenStart;
    double screenStop;
    double screenStep;

    double pMax = 0.0;
    double iaMax = 0.0;

    double measuredIaMax = 0.0;
    double measuredIg2Max = 0.0;

    QString hwVersion;
    QString swVersion;

    Client *client;

    QSerialPort *serialPort;
    bool awaitingResponse = false;
    QByteArray serialBuffer;

    QTimer *timeoutTimer;
    QTimer *heaterTimer;

    QList<QString> commandBuffer;
    QList<QString> setupCommands;
    QString stepCommandPrefix;
    QString sweepCommandPrefix;

    SampleSet result;
    QList<QList <Sample *>> sweepResult;
    QList<Sample *> *currentSweep;
    int setupIndex;

    int deviceType = TRIODE;
    int testType = ANODE_CHARACTERISTICS;

    QList<int> stepParameter;
    QList<QList <int>> sweepParameter;
    int stepIndex;
    int sweepIndex;
    int stepType;
    int sweepType;
    int sweepPoints = 40;

    bool isHeatersOn = false;
    bool isStopRequested = false;
    bool isTestRunning = false;
    bool isTestAborted = false;
    bool isEndSweep = false;
    bool isDataSetValid = false;
    bool isVersionRead = false;

    static QRegularExpression *sampleMatcher;
    static QRegularExpression *getMatcher;
    static QRegularExpression *infoMatcher;

    Sample *createSample(QString response);
    int convertTargetVoltage(int electrode, double voltage);
    double convertMeasuredVoltage(int electrode, int voltage);
    double convertMeasuredCurrent(int electrode, int current, int currentLo = 0);
    void sendCommand(QString command);
    void nextCommand();
    void nextSample();
    void abortTest();
    QString buildSetCommand(QString command, int value);
    void checkResponse(QString response);
    void responseTimeout();
    void steppedSweep(double sweepStart, double sweepStop, double stepStart, double stepStop, double step);
    void singleSweep(double sweepStart, double sweepStop);
    double sampleFunction(double linearValue);
};

#endif // ANALYSER_H
