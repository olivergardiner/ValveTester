#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->testType->addItem("Pentode");
    ui->testType->addItem("Triode");
    ui->testType->addItem("Diode");

    ui->plotType->addItem("Plate Characteristics");

    ui->runButton->setEnabled(false);

    connect(&serialPort, &QSerialPort::readyRead, this, &MainWindow::handleReadyRead);
    connect(&serialPort, &QSerialPort::errorOccurred, this, &MainWindow::handleError);
    connect(&timeoutTimer, &QTimer::timeout, this, &MainWindow::handleTimeout);

    checkComPorts();
}

MainWindow::~MainWindow()
{
    serialPort.close();

    delete ui;
}

void MainWindow::on_actionPrint_triggered()
{

}

void MainWindow::on_actionQuit_triggered()
{
    QCoreApplication::quit();
}

void MainWindow::checkComPorts() {
    serialPorts = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &serialPortInfo : serialPorts) {
        if (serialPortInfo.vendorIdentifier() == 0x1a86 && serialPortInfo.productIdentifier() == 0x7523) {
            port = serialPortInfo.portName();
        }
    }

    serialPort.setPortName(port);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setBaudRate(QSerialPort::Baud115200);
    serialPort.open(QSerialPort::ReadWrite);
}

void MainWindow::sendCommand(QString command)
{
    sendCommand(command, &MainWindow::checkResponse, &MainWindow::responseTimeout);
}

void MainWindow::sendCommand(QString command, void (MainWindow::*read)(QString), void (MainWindow::*timeout)())
{
    responseCallback = read;
    timeoutCallback = timeout;

    QByteArray c = command.toLatin1();

    serialPort.write(c);

    timeoutTimer.start(5000);
    awaitingResponse = true;
}

void MainWindow::checkResponse(QString response)
{
    // Just looking to check for an "OK:" response and log anything else (one day)
    awaitingResponse = false;
    timeoutTimer.stop();
}

void MainWindow::checkTestResponse(QString response)
{
    awaitingResponse = false;
    timeoutTimer.stop();
    ui->runButton->setChecked(false);
}

void MainWindow::responseTimeout()
{
    awaitingResponse = false;
}

void MainWindow::testTimeout()
{
    ui->runButton->setChecked(false);
    awaitingResponse = false;
}

void MainWindow::handleReadyRead()
{
    serialBuffer.append(serialPort.readAll());

    if (awaitingResponse) {
        if (serialBuffer.contains('\n') || serialBuffer.contains('\r')) {
            // We have a complete line and so can process it as a response
            (this->*responseCallback)(serialBuffer);
            serialBuffer.clear();
        }
    } else {
        // We should log the unexpected characters
    }
}

void MainWindow::handleTimeout()
{
    (this->*timeoutCallback)();
}

void MainWindow::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ReadError) {

    }
}

void MainWindow::on_actionOptions_triggered()
{
    PreferencesDialog dialog;

    dialog.setPort(port);

    if (dialog.exec() == 1) {
        serialPort.close();

        port = dialog.getPort();

        serialPort.setPortName(port);
        serialPort.setBaudRate(QSerialPort::Baud115200);
        serialPort.open(QSerialPort::ReadWrite);
    }
}

void MainWindow::on_heaterButton_clicked()
{
    heaters = !heaters;

    ui->heaterButton->setText(heaters ? "Heaters On" : "Heaters Off");
    ui->heaterButton->setChecked(heaters);
    ui->runButton->setEnabled(heaters);
}

void MainWindow::on_runButton_clicked()
{
    if (heaters) {
        ui->runButton->setChecked(true);
        sendCommand("M2\n", &MainWindow::checkTestResponse, &MainWindow::testTimeout);
    } else {
        ui->runButton->setChecked(false);
    }
}
