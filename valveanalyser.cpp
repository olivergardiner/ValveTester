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

    // Default test parameters
    ui->deviceType->setCurrentIndex(1); // Default to Triode anode test with ECC83/12AX7
    ui->heaterVoltage->setText("6.3");
    ui->anodeStart->setText("0");
    ui->anodeStop->setText("300");
    ui->gridStart->setText("0");
    ui->gridStop->setText("4");
    ui->gridStep->setText("0.5");
    ui->pMax->setText("1.125");
    ui->iaMax->setText("5");

    on_deviceType_currentIndexChanged(1);
    on_testType_currentIndexChanged(0);
    heaterVoltage = 6.3;
    anodeStart = 0;
    anodeStop = 300;
    gridStart = 0;
    gridStop = 4;
    gridStep = 0.5;
    pMax = 1.125; // Max anode power dissipation (W)
    iaMax = 5.0; // Max anode current (mA)

    ui->runButton->setEnabled(false);

    ui->progressBar->setRange(0, 100);
    ui->progressBar->reset();
    ui->progressBar->setVisible(false);

    heaterIndicator = new LedIndicator();
    heaterIndicator->setOffColor(QColorConstants::LightGray);
    ui->heaterLayout->addWidget(heaterIndicator);

    ui->graphicsView->setScene(&scene);

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

    timeoutTimer.start(15000);
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
            QString vh = QString {"%1"}.arg(measuredHeaterVoltage, -6, 'f', 3, '0');
            ui->heaterVlcd->display(vh);
        } else if (variable == IH) {
            measuredHeaterCurrent = convertMeasuredCurrent(HEATER, value);
            //measuredHeaterCurrent += convertMeasuredCurrent(HEATER, value);
            //measuredHeaterCurrent /= 2.0;
            QString ih = QString {"%1"}.arg(measuredHeaterCurrent, -6, 'f', 3, '0');
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

void ValveAnalyser::doPlot()
{
    switch (test) {
    case ANODE_CHARACTERISTICS:
        plotAnode();
        break;
    default:
        break;
    }
}

void ValveAnalyser::plotAnode()
{
    QRegularExpression matcher(R"(^OK: Mode\(2\) (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+), (\d+))");
    QList<double> gridValues;
    QList<QList<double>> anodeVoltageValues;
    QList<QList<double>> anodeCurrentValues;

    QList<QList<double>> curves;

    double x[64];
    double y[64];
    double e[64];
    double g[64];

    double xa[1000];
    double ya[1000];
    double ea[1000];
    double ga[1000];

    double maxVoltage = 0.0;
    double maxCurrent = 0.0;
    double majorVDivision = 25.0;
    double minorVDivision = 5.0;
    double majorIDivision = 1.0;
    double minorIDivision = 0.1;

    mp_config curveConfig;
    curveConfig.ftol = 0.0;
    curveConfig.xtol = 0.0;
    curveConfig.gtol = 0.0;
    curveConfig.epsfcn = 0.0;
    curveConfig.stepfactor = 0.0;
    curveConfig.covtol = 0.0;
    curveConfig.maxiter = 1000;
    curveConfig.maxfev = 0;
    curveConfig.douserscale = 0;
    curveConfig.nofinitecheck = 0;
    curveConfig.iterproc = nullptr;

    mp_result curveResult;
    double parameters[CRV_MAX_PARAMETERS];
    mp_par constraints[CRV_MAX_PARAMETERS];
    int n;

    // First parse the results and establish axes
    int sweeps = sweepResult.length();
    int setIndex = 0;
    for (int i = 0; i < sweeps; i++) {
        QList<QString> thisSweep = sweepResult.at(i);

        QString sample = thisSweep.at(0);
        QRegularExpressionMatch match = matcher.match(sample);
        double gridVoltage = convertMeasuredVoltage(GRID, match.captured(3).toInt());
        gridValues.append(gridVoltage);
        QList<double> anodeVoltageSweep;
        QList<double> anodeCurrentSweep;

        int curveIndex = 0;
        int samples = thisSweep.length();
        for (int j = 0; j < samples; j++) {
            sample = thisSweep.at(j);
            match = matcher.match(sample);

            double anodeVoltage = convertMeasuredVoltage(ANODE, match.captured(4).toInt());
            double anodeCurrent = convertMeasuredCurrent(ANODE, match.captured(5).toInt(), match.captured(6).toInt()) * 1000;

            if (anodeVoltage > maxVoltage) {
                maxVoltage = anodeVoltage;
            }

            if (anodeCurrent > maxCurrent) {
                maxCurrent = anodeCurrent;
            }

            anodeVoltageSweep.append(anodeVoltage);
            anodeCurrentSweep.append(anodeCurrent);

            x[curveIndex] = anodeVoltage;
            y[curveIndex] = anodeCurrent;
            e[curveIndex] = 0.2;
            g[curveIndex] = -(gridVoltage + 0.01);
            curveIndex++;

            xa[setIndex] = anodeVoltage;
            ya[setIndex] = anodeCurrent;
            ea[setIndex] = 0.2;
            ga[setIndex] = -(gridVoltage + 0.01);
            setIndex++;
        }

        if (samples > 2) {
            anodeVoltageValues.append(anodeVoltageSweep);
            anodeCurrentValues.append(anodeCurrentSweep);
        }

        if (curveIndex > CRV_MAX_PARAMETERS) {
            n = initKorenTriode(-gridVoltage, parameters, constraints);

            struct curveData data;
            data.x = x;
            data.y = y;
            data.y_error = e;
            data.g = g;

            memset(&curveResult, 0, sizeof(curveResult));
            int status = mpfit(&fitCurve, curveIndex, n, parameters, constraints, &curveConfig, &data, &curveResult);

            if (status >= 0) {
                QList<double> pars;
                for (int i=0; i < n; i++) {
                    pars.append(parameters[i]);
                    QString message = QString {"Parameter (%1): "}.arg(i);
                    message += QString {"%1 "}.arg(parameters[i], 6, 'g', 5, '0');
                    qInfo(message.toStdString().c_str());
                }
                pars.append(gridVoltage);
                curves.append(pars);
            } else {
                qInfo("Could not fit curve");
            }
        }
    }

    n = initKorenTriode(1.0, parameters, constraints);

    struct curveData data;
    data.x = xa;
    data.y = ya;
    data.y_error = ea;
    data.g = ga;

    memset(&curveResult, 0, sizeof(curveResult));
    int status = mpfit(&fitCurve, setIndex, n, parameters, constraints, &curveConfig, &data, &curveResult);

    if (status >= 0) {
        for (int i=0; i < n; i++) {
            QString message = QString {"Parameter (%1): "}.arg(i);
            message += QString {"%1 "}.arg(parameters[i], 6, 'g', 5, '0');
            qInfo(message.toStdString().c_str());
        }
    } else {
        qInfo("Could not fit data set");
    }

    if (maxCurrent > 20.0) {
        majorIDivision = 10.0;
        minorIDivision = 1.0;
    }

    double iaAxisMax = (((int) (maxCurrent / majorIDivision)) + 1) * majorIDivision; // Over 20mA we plot the Ia axis in steps of 10mA
    double iaScale = PLOT_HEIGHT / iaAxisMax;

    double vaAxisMax = (((int) (maxVoltage / majorVDivision)) + 2) * majorVDivision;
    double vaScale = PLOT_WIDTH / vaAxisMax;

    scene.clear();

    //scene.addLine(0, 0, 0, 500); // Y axis
    //scene.addLine(0, 0, 500, 0); // X axis

    double va = 0.0;
    bool doLabel = true;
    while (va <= vaAxisMax) {
        scene.addLine(va * vaScale, 0, va * vaScale, PLOT_HEIGHT);

        if (doLabel) { // Label every other Va scale point
            QGraphicsTextItem *text;
            char labelText[16];
            sprintf(labelText, "%d", (int) (va + 0.5));
            text = scene.addText(labelText);
            double offset = 6.0 * strlen(labelText);
            text->setPos(va * vaScale - offset, PLOT_HEIGHT + 10);
        }

        doLabel = !doLabel;
        va += majorVDivision;
    }

    double ia = 0;
    while (ia <= iaAxisMax) {
        scene.addLine(0, (iaAxisMax - ia) * iaScale, PLOT_WIDTH, (iaAxisMax - ia) * iaScale);

        QGraphicsTextItem *text;
        char labelText[16];
        sprintf(labelText, "%d", (int) (ia + 0.5));
        text = scene.addText(labelText);
        double offset = 12.0 * strlen(labelText);
        text->setPos(-10 - offset, (iaAxisMax - ia) * iaScale - 10);

        ia += majorIDivision;
    }

    QPen samplePen;
    samplePen.setColor(QColor::fromRgb(0, 0, 0));
    QPen curvePen;
    curvePen.setColor(QColor::fromRgb(255, 0, 0));


    for (int i = 0; i < sweeps; i++) {
        double vg = gridValues.at(i);
        QList<double> anodeVoltageSweep = anodeVoltageValues.at(i);
        QList<double> anodeCurrentSweep = anodeCurrentValues.at(i);

        va = anodeVoltageSweep.at(0);
        ia = anodeCurrentSweep.at(0);

        int samples = anodeVoltageSweep.length();
        for (int j = 1; j < samples; j++) {
            double vaNext = anodeVoltageSweep.at(j);
            double iaNext = anodeCurrentSweep.at(j);

            scene.addLine(va *vaScale, (iaAxisMax - ia) * iaScale, vaNext * vaScale, (iaAxisMax - iaNext) * iaScale, samplePen);

            va = vaNext;
            ia = iaNext;
        }

        QGraphicsTextItem *text;
        char labelText[16];
        sprintf(labelText, "%.1fv", vg + 0.04);
        text = scene.addText(labelText);
        text->setPos(va * vaScale + 5, (iaAxisMax - ia) * iaScale - 10);
    }

    int numberCurves = curves.length();
    for (int i=0; i < numberCurves; i++) {
        double vg = -gridValues.at(i);

        va = 0;
        ia = 0;
        for (int j=0; j < 101; j++) {
            double vaNext = (maxVoltage * j) / 100.0;
            double iaNext = curveFunction(vaNext, vg, n, parameters);

            if (ia <= iaAxisMax) {
                scene.addLine(va *vaScale, (iaAxisMax - ia) * iaScale, vaNext * vaScale, (iaAxisMax - iaNext) * iaScale, curvePen);
            }

            va = vaNext;
            ia = iaNext;
        }
    }
}

int ValveAnalyser::initSimpleTriode(double vg, double *parameters, mp_par *constraints)
{
    parameters[0] = -0.005; // Grid current - start with -5uA
    setDefaultParameterConstraints(&constraints[0]);
    constraints[0].limited[0] = true;
    constraints[0].limited[1] = true;
    constraints[0].limits[0] = -2.0;
    constraints[0].limits[1] = 0.0;
    parameters[1] = 1.70; // G (perveance) - start with 1.7 (rough 12AX7 value)
    setDefaultParameterConstraints(&constraints[1]);
    constraints[1].limited[0] = true;
    constraints[1].limited[1] = false;
    constraints[1].limits[0] = 0.0;
    //constraints[1].limits[1] = 10.0;
    parameters[2] = 1.5; // Alpha - should be 1.5
    setDefaultParameterConstraints(&constraints[2]);
    constraints[2].limited[0] = true;
    constraints[2].limited[1] = true;
    constraints[2].limits[0] = 1.0;
    constraints[2].limits[1] = 2.0;
    parameters[3] = 100.0; // Mu - start in 12AX7 territory
    setDefaultParameterConstraints(&constraints[3]);
    constraints[3].limited[0] = true;
    constraints[3].limited[1] = true;
    constraints[3].limits[0] = 1.0;
    constraints[3].limits[1] = 200.0;

    curveFunction = simpleTriodeModel;

    return 4; // Number of parameters
}

int ValveAnalyser::initKorenTriode(double vg, double *parameters, mp_par *constraints)
{
    parameters[0] = 0.0; // kvb
    setDefaultParameterConstraints(&constraints[0]);
    constraints->fixed = true;
    constraints[0].limited[0] = true;
    constraints[0].limited[1] = true;
    constraints[0].limits[0] = 0.0;
    constraints[0].limits[1] = 10.0;
    parameters[1] = 100.0; // kp (perveance)
    setDefaultParameterConstraints(&constraints[1]);
    constraints[1].limited[0] = true;
    constraints[1].limited[1] = false;
    constraints[1].limits[0] = 0.01;
    //constraints[1].limits[1] = 10.0;
    parameters[2] = 1.5; // Alpha - should be 1.5
    setDefaultParameterConstraints(&constraints[2]);
    constraints[2].limited[0] = true;
    constraints[2].limited[1] = true;
    constraints[2].limits[0] = 1.0;
    constraints[2].limits[1] = 2.0;
    parameters[3] = 100.0; // Mu - start in 12AX7 territory
    setDefaultParameterConstraints(&constraints[3]);
    constraints[3].limited[0] = true;
    constraints[3].limited[1] = true;
    constraints[3].limits[0] = 1.0;
    constraints[3].limits[1] = 200.0;
    parameters[4] = 1.0; // kg (scale factor)
    setDefaultParameterConstraints(&constraints[4]);
    constraints[4].limited[0] = true;
    constraints[4].limited[1] = false;
    constraints[4].limits[0] = 0.01;

    curveFunction = korenTriodeModel;

    return 5; // Number of parameters
}

void ValveAnalyser::startTest()
{
    // Clear down the previous result set
    sweepResult.clear();
    currentSweep = new QList<QString>;

    isStopRequested = false;
    isTestAborted = false;
    isTestRunning = true;
    endSweep = false;

    switch (test) {
    case ANODE_CHARACTERISTICS:
        stepType = GRID;
        stepCommandPrefix = "S2 ";
        sweepType = ANODE;
        sweepCommandPrefix = "S3 ";

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
    if (!endSweep && sweepIndex < sweepParameter.at(stepIndex).length()) {
        // Run the next value in the sweep
        setupCommands.append(buildSetCommand(sweepCommandPrefix, sweepParameter.at(stepIndex).at(sweepIndex)));
        updateTest();
        sweepIndex++;
    } else { // We've reached the end of this sweep so onto the next step
        stepIndex++;
        sweepIndex = 0;
        endSweep = false;
        if (!currentSweep->isEmpty()) {
            sweepResult.append(*currentSweep);
            currentSweep = new QList<QString>;
        }

        if (stepIndex < stepParameter.length()) {
            //setupCommands.append("M1"); // Discharge the capacitor banks at the end of a sweep (or it may take a while)
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

            doPlot();
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

            // Check whether response exceeds current or power limits
            QRegularExpression matcher(R"(^OK: Mode\(2\) (\d+), (\d+)\, (\d+)\, (\d+)\, (\d+)\, (\d+)\, (\d+)\, (\d+)\, (\d+)\, (\d+))");
            QRegularExpressionMatch match = matcher.match(response);
            double anodeVoltage = convertMeasuredVoltage(ANODE, match.captured(4).toInt());
            double anodeCurrent = convertMeasuredCurrent(ANODE, match.captured(5).toInt(), match.captured(6).toInt());

            message = QString {"Anode voltage: %1v"}.arg(anodeVoltage, 6, 'f', 1, '0' );
            qInfo(message.toStdString().c_str());
            message = QString {"Anode current: %1A"}.arg(anodeCurrent, 6, 'f', 4, '0' );
            qInfo(message.toStdString().c_str());

            if ((anodeCurrent * 1000) > iaMax || (anodeCurrent * anodeVoltage) > pMax) {
                endSweep = true;
                qInfo("Ending sweep");
            }

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

    while (stepVoltage <= (stepStop + 0.01)) {
        stepParameter.append(convertTargetVoltage(stepType, stepVoltage));

        QList<int> thisSweep;

        for (double sweep = 0.0; sweep <= 1.01; sweep += increment) {
            double sweepVoltage = sweepStart + (sweepStop - sweepStart) * sampleFunction(sweep);
            thisSweep.append(convertTargetVoltage(sweepType, sweepVoltage));
        }

        sweepParameter.append(thisSweep);

        stepVoltage += step;
    }
}

double ValveAnalyser::sampleFunction(double linearValue)
{
    // Converts a linear % value to a transformed % value to concentrate sweep sample points where there is most change
    if (mode == TRIODE && test == ANODE_CHARACTERISTICS) { // A triode may start to conduct at any anode voltage so we can't predict where the "bend" will be...
        return linearValue; // ...so a linear sampling is best
    } else if (mode == PENTODE && test == ANODE_CHARACTERISTICS) { // A Pentode has a knee as the anode voltage rises from 0v...
        // ...so we want more smaples early on - the code below uses the profile of a log pot to do the necessary bending

        // From: https://electronics.stackexchange.com/questions/304692/formula-for-logarithmic-audio-taper-pot
        // y = a.b^x - a
        // b = ((1 / ym) - 1)^2
        // a = 1 / (b - 1)

        //double b = pow((1.0 / ym) - 1.0, 2);

        double ym = 0.2;

        double b = ((1.0 / ym) - 1.0);
        b = b * b;

        double a = 1 / (b - 1.0);

        return a * pow(b, linearValue) - a;
    }

    return linearValue; // Backstop is to return linear sampling
}

double ValveAnalyser::updateVoltage(QLineEdit *input, double oldValue, int electrode)
{
    double value = checkDoubleValue(input, oldValue);

    switch (electrode) {
    case HEATER:
        if (value > 16.0) {
            value = 16.0;
        }
        break;
    case GRID:
        if (value > 66.0) {
            value = 66.0;
        }
        break;
    case ANODE:
    case SCREEN:
        if (value > 540.0) {
            value = 540.0;
        }
        break;
    default:
        break;
    }

    updateDoubleValue(input, value);

    return value;
}

double ValveAnalyser::updatePMax()
{
    double value = checkDoubleValue(ui->pMax, pMax);

    if (value > 50.0) {
        value = 50.0;
    }

    updateDoubleValue(ui->pMax, value);
    pMax = value;

    return value;
}

double ValveAnalyser::updateIaMax()
{
    double value = checkDoubleValue(ui->iaMax, iaMax);

    if (value > 200.0) {
        value = 200.0;
    }

    updateDoubleValue(ui->iaMax, value);
    iaMax = value;

    return value;
}

double ValveAnalyser::checkDoubleValue(QLineEdit *input, double oldValue)
{
    float parsedValue;

    const char *value = _strdup(input->text().toStdString().c_str());

    int n = sscanf_s(value, "%f.3", &parsedValue, strlen(value));

    if (n < 1) {
        return oldValue;
    }

    if (parsedValue < 0) {
        return 0.0;
    }

    return parsedValue;
}

void ValveAnalyser::updateDoubleValue(QLineEdit *input, double value)
{
    char number[32];

    sprintf(number, "%.3f", value);

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

    //ui->heaterButton->setChecked(heaters);
    heaterIndicator->setState(heaters);
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

void ValveAnalyser::on_iaMax_editingFinished()
{
    updateIaMax();
}


void ValveAnalyser::on_pMax_editingFinished()
{
    updatePMax();
}
