#ifndef VALVEANALYSER_H
#define VALVEANALYSER_H

#include <QMainWindow>
#include <QString>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>
#include <QLineEdit>
#include <QFile>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QColor>

#include <cmath>

#include "preferencesdialog.h"
#include "command.h"
#include "ledindicator.h"
#include "ceres/ceres.h"
#include "glog/logging.h"

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

#define PLOT_WIDTH 430
#define PLOT_HEIGHT 370

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

extern double (*curveFunction)(double va, double vg, int n, double *p);

QT_BEGIN_NAMESPACE
namespace Ui { class ValveAnalyser; }
QT_END_NAMESPACE

class ValveAnalyser : public QMainWindow
{
    Q_OBJECT

public:
    ValveAnalyser(QWidget *parent = nullptr);
    ~ValveAnalyser();

    void sendCommand(QString command, void (ValveAnalyser::* read)(QString), void (ValveAnalyser::* timeout)());

private slots:
    void on_actionPrint_triggered();

    void on_actionQuit_triggered();

    void on_actionOptions_triggered();

    void on_heaterButton_clicked();

    void on_runButton_clicked();

    void handleReadyRead();

    void handleTimeout();

    void handleHeaterTimeout();

    void handleError(QSerialPort::SerialPortError error);

    void on_deviceType_currentIndexChanged(int index);

    void on_testType_currentIndexChanged(int index);

    void on_anodeStart_editingFinished();

    void on_anodeStop_editingFinished();

    void on_anodeStep_editingFinished();

    void on_gridStart_editingFinished();

    void on_gridStop_editingFinished();

    void on_gridStep_editingFinished();

    void on_screenStart_editingFinished();

    void on_screenStop_editingFinished();

    void on_screenStep_editingFinished();

    void on_heaterVoltage_editingFinished();

    void on_iaMax_editingFinished();

    void on_pMax_editingFinished();

private:
    Ui::ValveAnalyser *ui;

    LedIndicator *heaterIndicator;
    QGraphicsScene scene;

    QList<QSerialPortInfo> serialPorts;
    QString port = "COM1";
    QSerialPort serialPort;
    QTimer timeoutTimer;
    QTimer heaterTimer;  // Used to control the polling of the heater values
    bool awaitingResponse = false;
    QByteArray serialBuffer;

    int measuredValues[10];
    double measuredHeaterVoltage = 0.0;
    double measuredHeaterCurrent = 0.0;

    int mode = PENTODE;
    int device = PENTODE;
    int test = ANODE_CHARACTERISTICS;

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

    double iaMax;
    double pMax;

    bool heaters = false;

    double vRefMaster = 4.096;
    double vRefSlave = 2.048;

    int sweepPoints = 40;

    void checkComPorts();

    void sendCommand(QString command);

    void (ValveAnalyser::* responseCallback)(QString);
    void (ValveAnalyser::* timeoutCallback)();

    void checkResponse(QString response);
    void responseTimeout();

    void pentodeMode();
    void triodeMode(bool doubleTriode);
    void diodeMode();

    QList<Command> commandBuffer;

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
    bool isStopRequested;
    bool isTestRunning = false;
    bool isTestAborted;
    bool endSweep;
    QFile *logFile;

    void log(QString message);

    QString buildSetCommand(QString command, int value);
    void startTest();
    void stopTest();
    void doPlot();
    void plotAnode();
    double korenCurrent(double va, double vg, double kp, double kvb, double a, double mu);
    double improvedKorenCurrent(double va, double vg, double kp, double kvb, double kvb2, double vct, double a, double mu);
    void updateTest();
    void prepareTest();
    void abortTest();
    void checkTestResponse(QString response);
    void testTimeout();
    int convertTargetVoltage(int electrode, double voltage);
    double convertMeasuredVoltage(int electrode, int voltage);
    double convertMeasuredCurrent(int electrode, int current, int currentLo = 0);
    void steppedSweep(double sweepStart, double sweepStop, double stepStart, double stepStop, double step);
    void singleSweep(double sweepStart, double sweepStop);
    double sampleFunction(double linearValue);

    double updateVoltage(QLineEdit *input, double oldValue, int electrode);
    double updatePMax();
    double updateIaMax();
    double checkDoubleValue(QLineEdit *input, double oldValue);
    void updateDoubleValue(QLineEdit *input, double value);
};

#endif // VALVEANALYSER_H
