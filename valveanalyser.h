#ifndef VALVEANALYSER_H
#define VALVEANALYSER_H

#include <QMainWindow>
#include <QString>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>

#include "preferencesdialog.h"

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

enum eDevice {
    PENTODE,
    TRIODE,
    DOUBLE_TRIODE,
    DIODE
};

enum eTest {
    ANODE_CHARACTERISTICS,
    SCREEN_CHARACTERISTICS,
    TRANSFER_CHARACTERISTICS
};

enum eElectrode {
    HEATER,
    ANODE,
    GRID,
    SCREEN
};

QT_BEGIN_NAMESPACE
namespace Ui { class ValveAnalyser; }
QT_END_NAMESPACE

class ValveAnalyser : public QMainWindow
{
    Q_OBJECT

public:
    ValveAnalyser(QWidget *parent = nullptr);
    ~ValveAnalyser();

private slots:
    void on_actionPrint_triggered();

    void on_actionQuit_triggered();

    void on_actionOptions_triggered();

    void on_heaterButton_clicked();

    void on_runButton_clicked();

    void handleReadyRead();

    void handleTimeout();

    void handleError(QSerialPort::SerialPortError error);

    void on_deviceType_currentIndexChanged(int index);

    void on_testType_currentIndexChanged(int index);

private:
    Ui::ValveAnalyser *ui;

    QList<QSerialPortInfo> serialPorts;
    QString port = "COM1";
    QSerialPort serialPort;
    QTimer timeoutTimer;
    bool awaitingResponse = false;
    QByteArray serialBuffer;

    int mode = PENTODE;
    int device = PENTODE;
    int test = ANODE_CHARACTERISTICS;

    double anodeStart;
    double anodeStop;
    double anodeStep;

    double gridStart;
    double gridStop;
    double gridStep;

    double screenStart;
    double screenStop;
    double screenStep;

    bool heaters = false;

    double vRefMaster = 4.096;
    double vRefSlave = 2.048;

    void checkComPorts();

    void sendCommand(QString command);
    void sendCommand(QString command, void (ValveAnalyser::* read)(QString), void (ValveAnalyser::* timeout)());

    void (ValveAnalyser::* responseCallback)(QString);
    void (ValveAnalyser::* timeoutCallback)();

    void checkResponse(QString response);
    void responseTimeout();

    void pentodeMode();
    void triodeMode(bool doubleTriode);
    void diodeMode();

    QList<QString> setupCommands;
    QList<int> stepParameter;
    QList<QList <int>> sweepParameter;
    QList<QList <QString>> sweepResult;
    QList<QString> *currentSweep;
    int setupIndex;
    int stepIndex;
    int sweepIndex;
    bool isMeasurement;
    int stepType;
    int sweepType;
    QString stepCommandPrefix;
    QString sweepCommandPrefix;
    int sweepPoints = 20;
    bool isStopRequested;
    bool isTestRunning = false;;
    bool isTestAborted;

    QString buildSetCommand(QString command, int value);
    void startTest();
    void stopTest();
    void updateTest();
    void prepareTest();
    void abortTest();
    void checkTestResponse(QString response);
    void testTimeout();
    int convertVoltage(int electrode, double voltage);
    void steppedSweep(double sweepStart, double sweepStop, double stepStart, double stepStop, double step);
    void singleSweep(double sweepStart, double sweepStop);
    double sampleFunction(double linearValue);
};

#endif // VALVEANALYSER_H
