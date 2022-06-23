#ifndef VALVEANALYSER_H
#define VALVEANALYSER_H

#include <QMainWindow>
#include <QString>
#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QFile>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <QGraphicsScene>
#include <QGraphicsTextItem>
#include <QColor>

#include <cmath>

#include "preferencesdialog.h"
#include "template.h"
#include "analyser/analyser.h"
#include "analyser/client.h"
#include "analyser/sample.h"
#include "valvemodel/plot.h"
#include "valvemodel/modelfactory.h"

#include "ledindicator.h"

QT_BEGIN_NAMESPACE
namespace Ui { class ValveAnalyser; }
QT_END_NAMESPACE

class ValveAnalyser : public QMainWindow, public Client
{
    Q_OBJECT

public:
    ValveAnalyser(QWidget *parent = nullptr);
    ~ValveAnalyser();

    virtual void updateHeater(double vh, double ih);
    virtual void testProgress(int progress);
    virtual void testFinished();
    virtual void testAborted();

private slots:
    void on_actionPrint_triggered();

    void on_actionQuit_triggered();

    void on_actionOptions_triggered();

    void on_heaterButton_clicked();

    void on_runButton_clicked();

    void handleReadyRead();

    void handleError(QSerialPort::SerialPortError error);

    void handleTimeout();

    void handleHeaterTimeout();

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

    void on_modelSelection_currentIndexChanged(int index);

    void on_fitModelButton_clicked();

    void on_showMeasuredValues_clicked(bool checked);

    void on_showModelledValues_clicked(bool checked);

private:
    Ui::ValveAnalyser *ui;
    QLineEdit *parameterValues[8];
    QLabel *parameterLabels[8];

    LedIndicator *heaterIndicator;
    QGraphicsItemGroup *measuredCurves = nullptr;
    QGraphicsItemGroup *modelledCurves = nullptr;

    Analyser *analyser;
    QString port;
    QSerialPort serialPort;

    QTimer timeoutTimer;
    QTimer heaterTimer;

    Plot plot;

    QList<QSerialPortInfo> serialPorts;

    QJsonObject config;
    QList<Template> templates;

    Model *model = nullptr;

    int deviceType = TRIODE;
    int testType = ANODE_CHARACTERISTICS;

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

    void checkComPorts();
    void setSerialPort(QString portName);

    void readConfig(QString filename);
    void loadTemplate(int index);
    void buildModelSelection();
    void buildModelParameters();
    void resetPlot();
    void updateParameterDisplay();

    void pentodeMode();
    void triodeMode(bool doubleTriode);
    void diodeMode();

    QString plotTitle = "";

    QFile *logFile;

    void log(QString message);

    void doPlot();
    void plotAnode();
    void plotTransfer();
    void plotTransferModel();
    void runModel();
    void plotModel();
    void plotAnodeModel();
    QLine createSegment(double x1, double y1, double x2, double y2, QPen pen);

    double updateVoltage(QLineEdit *input, double oldValue, int electrode);
    double updatePMax();
    double updateIaMax();
    double checkDoubleValue(QLineEdit *input, double oldValue);
    void updateDoubleValue(QLineEdit *input, double value);
};

#endif // VALVEANALYSER_H
