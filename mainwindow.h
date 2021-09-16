#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>

#include "preferencesdialog.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_actionPrint_triggered();

    void on_actionQuit_triggered();

    void on_actionOptions_triggered();

    void on_heaterButton_clicked();

    void on_runButton_clicked();

    void handleReadyRead();

    void handleTimeout();

    void handleError(QSerialPort::SerialPortError error);

private:
    Ui::MainWindow *ui;

    QList<QSerialPortInfo> serialPorts;
    QString port = "COM1";
    QSerialPort serialPort;
    QTimer timeoutTimer;
    bool awaitingResponse = false;
    QByteArray serialBuffer;

    bool heaters = false;

    void checkComPorts();

    void sendCommand(QString command);
    void sendCommand(QString command, void (MainWindow::* read)(QString), void (MainWindow::* timeout)());

    void (MainWindow::* responseCallback)(QString);
    void (MainWindow::* timeoutCallback)();

    void checkResponse(QString response);
    void checkTestResponse(QString response);
    void responseTimeout();
    void testTimeout();
};

#endif // MAINWINDOW_H
