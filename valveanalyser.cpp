#include "valveanalyser.h"
#include "ui_valveanalyser.h"

ValveAnalyser::ValveAnalyser(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ValveAnalyser)
{
    logFile = new QFile("analyser.log");
    if (!logFile->open(QIODevice::WriteOnly)) {
        qWarning("Couldn't open log file.");
        logFile = nullptr;
    }

    ui->setupUi(this);

    ui->deviceType->addItem("Pentode", PENTODE);
    ui->deviceType->addItem("Triode", TRIODE);
    ui->deviceType->addItem("Double Triode", TRIODE);
    ui->deviceType->addItem("Diode", DIODE);

    on_deviceType_currentIndexChanged(0); // Force set up of Device Type
    on_testType_currentIndexChanged(0);

    ui->runButton->setEnabled(false);

    ui->progressBar->setRange(0, 100);
    ui->progressBar->reset();
    ui->progressBar->setVisible(false);

    connect(&serialPort, &QSerialPort::readyRead, this, &ValveAnalyser::handleReadyRead);
    connect(&serialPort, &QSerialPort::errorOccurred, this, &ValveAnalyser::handleError);
    connect(&timeoutTimer, &QTimer::timeout, this, &ValveAnalyser::handleTimeout);
    connect(&heaterTimer, &QTimer::timeout, this, &ValveAnalyser::handleHeaterTimeout);

    checkComPorts();

    heaterTimer.start(2000); // waits for 2s before starting to poll the measured heater values
}

ValveAnalyser::~ValveAnalyser()
{
    serialPort.close();

    delete ui;
}

void ValveAnalyser::on_actionPrint_triggered()
{

}

void ValveAnalyser::on_actionQuit_triggered()
{
    QCoreApplication::quit();
}

void ValveAnalyser::checkComPorts() {
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

void ValveAnalyser::sendCommand(QString command)
{
    sendCommand(command, &ValveAnalyser::checkResponse, &ValveAnalyser::responseTimeout);
}

void ValveAnalyser::sendCommand(QString command, void (ValveAnalyser::*read)(QString), void (ValveAnalyser::*timeout)())
{
    if (awaitingResponse) { // Need to wait for previous command to complete (or timeout) before sending next command
        Command bufferedCommand(command, read, timeout);
        commandBuffer.append(bufferedCommand);

        return;
    }

    qInfo(command.toStdString().c_str());

    responseCallback = read;
    timeoutCallback = timeout;

    QByteArray c = command.toLatin1();

    serialPort.write(c);
    serialPort.write("\r\n");

    timeoutTimer.start(5000);
    awaitingResponse = true;
}

void ValveAnalyser::checkResponse(QString response)
{
    // Just looking to check for an "OK:" response and log anything else (one day)
    awaitingResponse = false;
    timeoutTimer.stop();

    QRegularExpression matcher(R"(^OK: Get\((\d+)\) = (\d+))");
    QRegularExpressionMatch match = matcher.match(response);
    if (match.lastCapturedIndex() == 2) {
        int variable = match.captured(1).toInt();
        int value = match.captured(2).toInt();

        if (variable >= 0 && variable <= 9) {
            measuredValues[variable] = value;
        }

        if (variable == VH) {
            measuredHeaterVoltage = convertMeasuredVoltage(HEATER, value);
            //measuredHeaterVoltage += convertMeasuredVoltage(HEATER, value);
            //measuredHeaterVoltage /= 2.0;
            QString vh = QString {"%1"}.arg(measuredHeaterVoltage, -5, 'f', 2, '0');
            ui->heaterVlcd->display(vh);
        } else if (variable == IH) {
            measuredHeaterCurrent = convertMeasuredCurrent(HEATER, value);
            //measuredHeaterCurrent += convertMeasuredCurrent(HEATER, value);
            //measuredHeaterCurrent /= 2.0;
            QString ih = QString {"%1"}.arg(measuredHeaterCurrent, -5, 'f', 2, '0');
            ui->heaterIlcd->display(ih);
        }
    }

    if (!commandBuffer.isEmpty()) { // There is a command to send
        commandBuffer.first().send(this);
        commandBuffer.removeFirst();
    }
}

void ValveAnalyser::responseTimeout()
{
    awaitingResponse = false;

    if (!commandBuffer.isEmpty()) { // There is a command to send
        commandBuffer.first().send(this);
        commandBuffer.removeFirst();
    }
}

void ValveAnalyser::pentodeMode()
{
    mode = PENTODE;

    ui->testType->clear();
    ui->testType->addItem("Anode Characteristics", ANODE_CHARACTERISTICS);
    ui->testType->addItem("Transfer Characteristics", TRANSFER_CHARACTERISTICS);
    ui->testType->addItem("Screen Characteristics", SCREEN_CHARACTERISTICS);

    ui->gridLabel->setEnabled(true);
    ui->gridStart->setEnabled(true);
    ui->gridStop->setEnabled(true);
    ui->gridStep->setEnabled(true);

    ui->screenLabel->setEnabled(true);
    ui->screenStart->setEnabled(true);
    ui->screenStop->setEnabled(true);
    ui->screenStep->setEnabled(true);
}

void ValveAnalyser::triodeMode(bool doubleTriode)
{
    mode = TRIODE;

    ui->testType->clear();
    ui->testType->addItem("Anode Characteristics", ANODE_CHARACTERISTICS);
    ui->testType->addItem("Transfer Characteristics", TRANSFER_CHARACTERISTICS);

    ui->gridLabel->setEnabled(true);
    ui->gridStart->setEnabled(true);
    ui->gridStop->setEnabled(true);
    ui->gridStep->setEnabled(true);

    ui->screenLabel->setEnabled(false);
    ui->screenStart->setEnabled(false);
    ui->screenStop->setEnabled(false);
    ui->screenStep->setEnabled(false);
}

void ValveAnalyser::diodeMode()
{
    mode = DIODE;

    ui->testType->clear();
    ui->testType->addItem("Anode Charcteristics", ANODE_CHARACTERISTICS);

    ui->gridLabel->setEnabled(false);
    ui->gridStart->setEnabled(false);
    ui->gridStop->setEnabled(false);
    ui->gridStep->setEnabled(false);

    ui->screenLabel->setEnabled(false);
    ui->screenStart->setEnabled(false);
    ui->screenStop->setEnabled(false);
    ui->screenStep->setEnabled(false);
}

void ValveAnalyser::log(QString message)
{
    if (logFile != nullptr) {
        logFile->write(message.toLatin1());
        logFile->write("\n");
    }
}

QString ValveAnalyser::buildSetCommand(QString command, int value)
{
    QString stringValue;
    stringValue.setNum(value);
    command.append(stringValue);

    return command;
}

void ValveAnalyser::stopTest()
{

}

void ValveAnalyser::startTest()
{
    // Clear down the previous result set
    sweepResult.clear();
    currentSweep = new QList<QString>;

    isStopRequested = false;
    isTestAborted = false;
    isTestRunning = true;

    switch (test) {
    case ANODE_CHARACTERISTICS:
        stepType = GRID;
        stepCommandPrefix = "S2 ";
        sweepType = ANODE;
        sweepCommandPrefix = "S3 ";

        // Forced values for testing
        //anodeStart = 0;
        //anodeStop = 300;
        //gridStart = 0;
        //gridStop = 1.0;
        //gridStep = 0.5;
        //screenStart = 250;

        steppedSweep(anodeStart, anodeStop, gridStart, gridStop, gridStep);

        if (mode == PENTODE) {
            setupCommands.append(buildSetCommand("S7 ", convertTargetVoltage(SCREEN, screenStart)));
        } else {
            setupCommands.append("S7 0");
        }
        setupCommands.append(buildSetCommand(stepCommandPrefix, stepParameter.at(0)));

        prepareTest();
        break;
    case TRANSFER_CHARACTERISTICS:
        if (mode == PENTODE) {

        } else {

        }
        break;
    case SCREEN_CHARACTERISTICS:
        break;
    default:
        break;
    }
}

void ValveAnalyser::updateTest()
{
    if (!setupCommands.isEmpty()) { // If there are any setup commands, pop them in turn and run them
        isMeasurement = false;
        QString nextCommand = setupCommands.takeFirst();
        sendCommand(nextCommand, &ValveAnalyser::checkTestResponse, &ValveAnalyser::testTimeout);
    } else { // If the setup is all done then we just need to run the test
        isMeasurement = true;
        sendCommand("M2", &ValveAnalyser::checkTestResponse, &ValveAnalyser::testTimeout);
    }
}

void ValveAnalyser::prepareTest() {
    if (sweepIndex < sweepParameter.at(stepIndex).length()) {
        // Run the next value in the sweep
        setupCommands.append(buildSetCommand(sweepCommandPrefix, sweepParameter.at(stepIndex).at(sweepIndex)));
        updateTest();
        sweepIndex++;
    } else { // We've reached the end of this sweep so onto the next step
        stepIndex++;
        sweepIndex = 0;
        sweepResult.append(*currentSweep);
        currentSweep = new QList<QString>;

        if (stepIndex < stepParameter.length()) {
            setupCommands.append("M1"); // Discharge the capacitor banks at the end of a sweep (or it may take a while)
            setupCommands.append(buildSetCommand(stepCommandPrefix, stepParameter.at(stepIndex)));
            setupCommands.append(buildSetCommand(sweepCommandPrefix, sweepParameter.at(stepIndex).at(sweepIndex)));
            updateTest();
        } else {
            // We've reached the end of the test!
            ui->runButton->setChecked(false);
            ui->progressBar->setVisible(false);
            sendCommand("M1");
            isTestRunning = false;

            if (!commandBuffer.isEmpty()) { // There is a command to send
                commandBuffer.first().send(this);
                commandBuffer.removeFirst();
            }
        }
    }

    int progress = ((stepIndex * sweepPoints) + sweepIndex) * 100 / (sweepPoints * stepParameter.length());
    ui->progressBar->setValue(progress);
}

void ValveAnalyser::abortTest()
{
    isTestRunning = false;
    isTestAborted = true;
    ui->runButton->setChecked(false);
    ui->progressBar->reset();
    ui->progressBar->setVisible(false);
}

void ValveAnalyser::checkTestResponse(QString response)
{
    awaitingResponse = false;
    timeoutTimer.stop();

    QString message = " Test response received: ";
    message += response;
    qInfo(message.toStdString().c_str());

    if (isStopRequested) {
        return;
    }

    if (response.startsWith("OK: ")) {
        if (isMeasurement) {
            // Store the measurement
            currentSweep->append(response);
            // Prepare the next test
            isMeasurement = false;
            setupCommands.clear(); // Just in case;
            prepareTest();
        } else {
            updateTest();
        }
    } else {
        // Abort
        abortTest();

        if (!commandBuffer.isEmpty()) { // There is a command to send
            commandBuffer.first().send(this);
            commandBuffer.removeFirst();
        }
    }
}

void ValveAnalyser::testTimeout()
{
    qWarning("Test timeout");
    awaitingResponse = false;
    timeoutTimer.stop();

    abortTest();

    if (!commandBuffer.isEmpty()) { // Just in case - there shouldn't be any queued commands
        commandBuffer.first().send(this);
        commandBuffer.removeFirst();
    }
}

int ValveAnalyser::convertTargetVoltage(int electrode, double voltage)
{
    int value = 0;

    switch (electrode) {
    case HEATER:
        value = (voltage * 1023 * 470 / 3770 / vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        value = (voltage * 1023 * 9400 / 1419400 / vRefMaster);
        break;
    case GRID:
        value = (voltage * 4095 / 16.5 / vRefMaster);
        break;
    default:
        break;
    }

    return value;
}

double ValveAnalyser::convertMeasuredVoltage(int electrode, int voltage)
{
    double value = 0;

    switch (electrode) {
    case HEATER:
        value = (((double) voltage) / 1023 / 470 * 3770 * vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        value = (((double) voltage) / 1023 / 9400 * 1419400 * vRefMaster);
        break;
    case GRID:
        value = (((double) voltage) / 4095 * 16.5 * vRefMaster);
        break;
    default:
        break;
    }

    return value;
}

double ValveAnalyser::convertMeasuredCurrent(int electrode, int current, int currentLo)
{
    double value = 0;
    double voltageHi;

    switch (electrode) {
    case HEATER:
        value = (((double) current) / 1023 / 0.22 * vRefSlave);
        break;
    case ANODE:
    case SCREEN:
        voltageHi = ((double) current) / 1023 / 2.0 * vRefMaster;
        if (voltageHi < 1.9) { // If we're close to 3 diode drops we should use the Lo value
            value = voltageHi / 33.333333;
        } else {
            value = (((double) currentLo) / 1023 / 2.0 * vRefMaster / 3.333333);
        }
        break;
    case GRID:
        break;
    default:
        break;
    }

    return value;
}

void ValveAnalyser::steppedSweep(double sweepStart, double sweepStop, double stepStart, double stepStop, double step)
{
    double increment = 1.0 / sweepPoints;

    double stepVoltage = stepStart;

    stepParameter.clear();
    sweepParameter.clear();
    stepIndex = 0;
    sweepIndex = 0;

    while (stepVoltage <= stepStop) {
        stepParameter.append(convertTargetVoltage(stepType, stepVoltage));

        QList<int> thisSweep;
        QList<QString> results;

        for (double sweep = 0.0; sweep <= 1.0; sweep += increment) {
            double sweepVoltage = sweepStart + (sweepStop - sweepStart) * sampleFunction(sweep);
            thisSweep.append(convertTargetVoltage(sweepType, sweepVoltage));
        }

        sweepParameter.append(thisSweep);
        sweepResult.append(results);

        stepVoltage += step;
    }
}

double ValveAnalyser::sampleFunction(double linearValue)
{
    // Converts a linear % value to a transformed % value to concentrate sweep sample points where there is most change

    // For now, is just linear
    return linearValue;
}

void ValveAnalyser::handleReadyRead()
{
    serialBuffer.append(serialPort.readAll());

    if (awaitingResponse) {
        if (serialBuffer.contains('\n') || serialBuffer.contains('\r')) {
            // We have a complete line and so can process it as a response
            qInfo(serialBuffer);

            (this->*responseCallback)(serialBuffer);

            serialBuffer.clear();
        }
    } else {
        // We should log the unexpected characters
    }
}

void ValveAnalyser::handleTimeout()
{
    (this->*timeoutCallback)();
}

void ValveAnalyser::handleHeaterTimeout()
{
    if (heaters) { // Only poll if the heaters are on
        if (!isTestRunning) { // Only poll if we're not running a test
            sendCommand("G0");
            sendCommand("G1"); // Second command should get queued
        }
    } else {
        measuredHeaterVoltage = 0.0;
        ui->heaterVlcd->display("0.000");
        measuredHeaterCurrent = 0.0;
        ui->heaterIlcd->display("0.000");
    }

    heaterTimer.start(500); // Do it again in 500ms...
}

void ValveAnalyser::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ReadError) {

    }
}

void ValveAnalyser::on_actionOptions_triggered()
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

void ValveAnalyser::on_heaterButton_clicked()
{
    heaters = !heaters;

    ui->heaterButton->setText(heaters ? "Heaters On" : "Heaters Off");
    ui->heaterButton->setChecked(heaters);
    ui->runButton->setEnabled(heaters);

    if (heaters) {
        sendCommand(buildSetCommand("S0 ", convertTargetVoltage(HEATER, heaterVoltage)));
    } else {
        sendCommand("S0 0");
    }
}

void ValveAnalyser::on_runButton_clicked()
{
    if (heaters) {
        ui->runButton->setChecked(true);
        ui->progressBar->reset();
        ui->progressBar->setVisible(true);
        startTest();
    } else {
        ui->runButton->setChecked(false);
    }
}

void ValveAnalyser::on_deviceType_currentIndexChanged(int index)
{
    switch (ui->deviceType->itemData(index).toInt()) {
    case PENTODE:
        pentodeMode();
        break;
    case TRIODE:
        triodeMode(index == DOUBLE_TRIODE);
        break;
    case DIODE:
        diodeMode();
        break;
    default:
        break;
    }

    on_testType_currentIndexChanged(0);
    device = index;
}

void ValveAnalyser::on_testType_currentIndexChanged(int index)
{
    switch (ui->testType->itemData(index).toInt()) {
    case ANODE_CHARACTERISTICS: // Anode swept and Grid stepped
        ui->anodeStop->setEnabled(true);
        ui->anodeStep->setEnabled(false);
        if (mode != DIODE) {
            ui->gridStop->setEnabled(true);
            ui->gridStep->setEnabled(true);
        }
        if (mode == PENTODE) { // Screen fixed (if Pentode)
            ui->screenStop->setEnabled(false);
            ui->screenStep->setEnabled(false);
        }
        break;
    case TRANSFER_CHARACTERISTICS: // Grid swept
        ui->gridStop->setEnabled(true);
        ui->gridStep->setEnabled(false);
        if (mode == PENTODE) { // Anode fixed and Screen stepped
            ui->anodeStop->setEnabled(false);
            ui->anodeStep->setEnabled(false);
            ui->screenStop->setEnabled(true);
            ui->screenStep->setEnabled(true);
        } else { // (Triode) Anode stepped and no Screen
            ui->anodeStop->setEnabled(true);
            ui->anodeStep->setEnabled(true);
        }
        break;
    case SCREEN_CHARACTERISTICS: // Screen fixed, Anode swept and Grid stepped
        ui->anodeStop->setEnabled(true);
        ui->anodeStep->setEnabled(false);
        ui->gridStop->setEnabled(true);
        ui->gridStep->setEnabled(true);
        ui->screenStop->setEnabled(false);
        ui->screenStep->setEnabled(false);
        break;
    default:
        break;
    }

    test = index;
}

void ValveAnalyser::on_anodeStart_editingFinished()
{
    anodeStart = updateVoltage(ui->anodeStart, anodeStart, ANODE);
}

void ValveAnalyser::on_anodeStop_editingFinished()
{
    anodeStop = updateVoltage(ui->anodeStop, anodeStop, ANODE);
}

void ValveAnalyser::on_anodeStep_editingFinished()
{
    anodeStep = updateVoltage(ui->anodeStep, anodeStep, ANODE);
}

void ValveAnalyser::on_gridStart_editingFinished()
{
    gridStart = updateVoltage(ui->gridStart, gridStart, GRID);
}

void ValveAnalyser::on_gridStop_editingFinished()
{
    gridStop = updateVoltage(ui->gridStop, gridStop, GRID);
}

void ValveAnalyser::on_gridStep_editingFinished()
{
    gridStep = updateVoltage(ui->gridStep, gridStep, GRID);
}

void ValveAnalyser::on_screenStart_editingFinished()
{
    screenStart = updateVoltage(ui->screenStart, screenStart, SCREEN);
}

void ValveAnalyser::on_screenStop_editingFinished()
{
    screenStop = updateVoltage(ui->screenStop, screenStop, SCREEN);
}

void ValveAnalyser::on_screenStep_editingFinished()
{
    screenStep = updateVoltage(ui->screenStep, screenStep, SCREEN);
}

void ValveAnalyser::on_heaterVoltage_editingFinished()
{
    heaterVoltage = updateVoltage(ui->heaterVoltage, heaterVoltage, HEATER);
}

double ValveAnalyser::updateVoltage(QLineEdit *input, double oldValue, int electrode)
{
    float parsedValue;

    const char *value = _strdup(input->text().toStdString().c_str());

    int n = sscanf_s(value, "%f", &parsedValue, strlen(value));

    if (n < 1) {
        parsedValue = oldValue;
    }

    if (parsedValue < 0) {
        parsedValue = 0.0;
    }
    switch (electrode) {
    case HEATER:
        if (parsedValue > 20.0) {
            parsedValue = 20.0;
        }
        break;
    case GRID:
        if (parsedValue > 66.0) {
            parsedValue = 66.0;
        }
        break;
    case ANODE:
    case SCREEN:
        if (parsedValue > 540.0) {
            parsedValue = 540.0;
        }
        break;
    default:
        break;
    }

    char number[32];

    sprintf(number, "%.2f", parsedValue);

    int length = strlen(number);
    for (int i=length-1;i >= 0; i--) {
        char test = number[i];
        if (test == '0' || test == '.') {
            number[i] = 0;
        }

        if (test != '0') {
            break;
        }
    }

    input->setText(number);

    return parsedValue;
}
