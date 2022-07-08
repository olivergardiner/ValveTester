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

    readConfig(tr("analyser.json"));

    ui->setupUi(this);
    buildModelParameters();

    ui->deviceType->addItem("Triode", TRIODE);
    ui->deviceType->addItem("Pentode", PENTODE);
    //ui->deviceType->addItem("Double Triode", TRIODE);
    //ui->deviceType->addItem("Diode", DIODE);

    loadTemplate(0);

    buildModelSelection();

    ui->runButton->setEnabled(false);

    ui->progressBar->setRange(0, 100);
    ui->progressBar->reset();
    ui->progressBar->setVisible(false);

    heaterIndicator = new LedIndicator();
    heaterIndicator->setOffColor(QColorConstants::LightGray);
    ui->heaterLayout->addWidget(heaterIndicator);

    ui->graphicsView->setScene(plot.getScene());

    connect(&serialPort, &QSerialPort::readyRead, this, &ValveAnalyser::handleReadyRead);
    connect(&serialPort, &QSerialPort::errorOccurred, this, &ValveAnalyser::handleError);
    connect(&timeoutTimer, &QTimer::timeout, this, &ValveAnalyser::handleTimeout);
    connect(&heaterTimer, &QTimer::timeout, this, &ValveAnalyser::handleHeaterTimeout);

    checkComPorts();

    analyser = new Analyser(this, &serialPort, &timeoutTimer, &heaterTimer);
}

ValveAnalyser::~ValveAnalyser()
{
    serialPort.close();
    delete ui;
}

void ValveAnalyser::updateHeater(double vh, double ih)
{
    // Update the heater display
    if (vh >= 0.0) {
        QString vhValue = QString {"%1"}.arg(vh, -6, 'f', 3, '0');
        ui->heaterVlcd->display(vhValue);
    }

    if (ih >= 0.0) {
        QString ihValue = QString {"%1"}.arg(ih, -6, 'f', 3, '0');
        ui->heaterIlcd->display(ihValue);
    }
}

void ValveAnalyser::testProgress(int progress)
{
    ui->progressBar->setValue(progress);
}

void ValveAnalyser::testFinished()
{
    ui->runButton->setChecked(false);
    ui->progressBar->setVisible(false);

    plotTitle = ui->deviceName->text();
    if (testType == ANODE_CHARACTERISTICS) {
        plotTitle.append(" Anode Characterisitcs");
        if (deviceType == PENTODE) {
            plotTitle.append(QString {" with Vg2 = %1V"}.arg(screenStart, -6, 'f', 3, '0'));
        }
    } else if (testType == TRANSFER_CHARACTERISTICS) {
        plotTitle.append(" Transfer Characterisitcs");
        if (deviceType == PENTODE) {
            plotTitle.append(QString {" with Vg2 = %1V"}.arg(screenStart, -6, 'f', 3, '0'));
        }
    }

    ui->plotTitle->setText(plotTitle);

    buildModelSelection();

    doPlot();
}

void ValveAnalyser::testAborted()
{
    ui->runButton->setChecked(false);
    ui->progressBar->setVisible(false);
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

    setSerialPort(port);
}

void ValveAnalyser::setSerialPort(QString portName)
{
    if (serialPort.isOpen()) {
        serialPort.close();
    }

    serialPort.setPortName(portName);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setBaudRate(QSerialPort::Baud115200);
    serialPort.open(QSerialPort::ReadWrite);
}

void ValveAnalyser::saveModel(QString filename)
{
    QFile modelFile(filename);

    if (!modelFile.open(QIODevice::ReadWrite)) {
        qWarning("Couldn't open model file.");
    } else {
        QJsonObject modelObject;

        modelObject["name"] = ui->deviceName->text();
        modelObject["vaMax"] = anodeStop;
        modelObject["iaMax"] = iaMax;
        modelObject["paMax"] = pMax;
        model->toJson(modelObject, gridStop, screenStop);

        modelFile.write(QJsonDocument(modelObject).toJson());
    }
}

void ValveAnalyser::saveSamples(QString filename)
{
    QFile samplelFile(filename);

    if (!samplelFile.open(QIODevice::ReadWrite)) {
        qWarning("Couldn't open model file.");
    } else {
        QJsonObject modelObject;

        modelObject["name"] = ui->deviceName->text();
        modelObject["vaMax"] = anodeStop;
        modelObject["iaMax"] = iaMax;
        modelObject["paMax"] = pMax;
        model->toJson(modelObject, gridStop, screenStop);

        samplelFile.write(QJsonDocument(modelObject).toJson());
    }
}

void ValveAnalyser::readConfig(QString filename)
{
    QFile configFile(filename);

    if (!configFile.open(QIODevice::ReadOnly)) {
        qWarning("Couldn't open config file.");
    } else {
        QByteArray configData = configFile.readAll();

        QJsonDocument configDoc(QJsonDocument::fromJson(configData));
        if (configDoc.isObject()) {
            config = configDoc.object();
        }

        if (config.contains("templates") && config["templates"].isArray()) {
            QJsonArray tpls = config["templates"].toArray();
            for (int i=0; i < tpls.count(); i++) {
                QJsonValue currentTemplate = tpls.at(i);
                if (currentTemplate.isObject()) {
                    Template *tpl = new Template();
                    tpl->read(currentTemplate.toObject());
                    templates.append(*tpl);
                }
            }
        }
    }
}

void ValveAnalyser::loadTemplate(int index)
{
    Template tpl = templates.at(index);

    ui->deviceName->setText(tpl.getName());
    heaterVoltage = tpl.getVHeater();
    anodeStart = tpl.getVaStart();
    anodeStop = tpl.getVaStop();
    anodeStep = tpl.getVaStep();
    gridStart = tpl.getVgStart();
    gridStop = tpl.getVgStop();
    gridStep = tpl.getVgStep();
    screenStart = tpl.getVsStart();
    screenStop = tpl.getVsStop();
    screenStep = tpl.getVsStep();
    pMax = tpl.getPaMax();
    iaMax = tpl.getIaMax();

    updateParameterDisplay();

    ui->deviceType->setCurrentIndex(tpl.getDeviceType());
    on_deviceType_currentIndexChanged(tpl.getDeviceType());

    ui->testType->setCurrentIndex(tpl.getTestType());
    on_testType_currentIndexChanged(tpl.getTestType());
}

void ValveAnalyser::buildModelSelection()
{
    ui->modelSelection->clear();

    if (deviceType == TRIODE) {
        ui->modelSelection->addItem("Simple", SIMPLE_TRIODE);
        ui->modelSelection->addItem("Koren", KOREN_TRIODE);
        ui->modelSelection->addItem("Improved Koren", IMPROVED_KOREN_TRIODE);
    } else if (deviceType == PENTODE) {
        ui->modelSelection->addItem("Koren", KOREN_PENTODE);
        ui->modelSelection->addItem("Derk", DERK_PENTODE);
        ui->modelSelection->addItem("DerkE", DERKE_PENTODE);
    }
}

void ValveAnalyser::buildModelParameters()
{
    parameterLabels[0] = ui->par1Label;
    parameterLabels[1] = ui->par2Label;
    parameterLabels[2] = ui->par3Label;
    parameterLabels[3] = ui->par4Label;
    parameterLabels[4] = ui->par5Label;
    parameterLabels[5] = ui->par6Label;
    parameterLabels[6] = ui->par7Label;

    parameterValues[0] = ui->par1Value;
    parameterValues[1] = ui->par2Value;
    parameterValues[2] = ui->par3Value;
    parameterValues[3] = ui->par4Value;
    parameterValues[4] = ui->par5Value;
    parameterValues[5] = ui->par6Value;
    parameterValues[6] = ui->par7Value;

    for (int i=0; i < 7; i++) { // Parameters all initially hidden
        parameterValues[i]->setVisible(false);
        parameterLabels[i]->setVisible(false);
    }
}

void ValveAnalyser::resetPlot()
{
    for (int i=0; i < 7; i++) { // Parameters all initially hidden
        parameterValues[i]->setVisible(false);
        parameterLabels[i]->setVisible(false);
    }

    plot.clear();
}

void ValveAnalyser::updateParameterDisplay()
{
    updateDoubleValue(ui->heaterVoltage, heaterVoltage);
    updateDoubleValue(ui->anodeStart, anodeStart);
    updateDoubleValue(ui->anodeStop, anodeStop);
    updateDoubleValue(ui->anodeStep, anodeStep);
    updateDoubleValue(ui->gridStart, gridStart);
    updateDoubleValue(ui->gridStop, gridStop);
    updateDoubleValue(ui->gridStep, gridStep);
    updateDoubleValue(ui->screenStart, screenStart);
    updateDoubleValue(ui->screenStop, screenStop);
    updateDoubleValue(ui->screenStep, screenStop);
    updateDoubleValue(ui->pMax, pMax);
    updateDoubleValue(ui->iaMax, iaMax);
}

void ValveAnalyser::pentodeMode()
{
    deviceType = PENTODE;

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
    deviceType = TRIODE;

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
    deviceType = DIODE;

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

void ValveAnalyser::doPlot()
{
    switch (testType) {
    case ANODE_CHARACTERISTICS:
        plotAnode2();
        break;
    case TRANSFER_CHARACTERISTICS:
        plotTransfer();
        break;
    default:
        break;
    }
}

void ValveAnalyser::plotAnode()
{
    double majorVDivision = 25.0;
    double majorIDivision = 1.0;

    double maxVoltage = anodeStop;
    double maxCurrent = analyser->getMeasuredIaMax();

    if (maxCurrent > 50.0) { // Over 20mA we plot the Ia axis in steps of 10mA
        majorIDivision = 10.0;
    } else if (maxCurrent > 20.0) {
        majorIDivision = 5.0;
    }

    double iaAxisMax = (((int) (maxCurrent / majorIDivision)) + 1) * majorIDivision;
    double vaAxisMax = (((int) (maxVoltage / majorVDivision)) + 2) * majorVDivision;
    plot.setAxes(0.0, vaAxisMax, majorVDivision, 0.0, iaAxisMax, majorIDivision, 2, 1);

    QPen samplePen;
    samplePen.setColor(QColor::fromRgb(0, 0, 0));

    QList<QGraphicsItem *> segments;

    const QList<QList <Sample *>> *sweepResult = analyser->getSweepResult();

    int sweeps = sweepResult->length();
    for (int i = 0; i < sweeps; i++) {
        QList<Sample *> thisSweep = sweepResult->at(i);

        Sample *firstSample = thisSweep.at(0);

        double vg = firstSample->getVg1();
        double va = firstSample->getVa();
        double ia = firstSample->getIa();

        int samples = thisSweep.length();
        for (int j = 1; j < samples; j++) {
             Sample *sample = thisSweep.at(j);

             double vaNext = sample->getVa();
             double iaNext = sample->getIa();

             segments.append(plot.createSegment(va, ia, vaNext, iaNext, samplePen));

             va = vaNext;
             ia = iaNext;
         }

        plot.createLabel(va, ia, vg);
    }

    ui->showMeasuredValues->setChecked(true);

    measuredCurves = plot.getScene()->createItemGroup(segments);
}

void ValveAnalyser::plotAnode2()
{
    double majorVDivision = 25.0;
    double majorIDivision = 1.0;

    double maxVoltage = anodeStop;
    double maxCurrent = analyser->getMeasuredIaMax();

    if (maxCurrent > 50.0) { // Over 20mA we plot the Ia axis in steps of 10mA
        majorIDivision = 10.0;
    } else if (maxCurrent > 20.0) {
        majorIDivision = 5.0;
    }

    double iaAxisMax = (((int) (maxCurrent / majorIDivision)) + 1) * majorIDivision;
    double vaAxisMax = (((int) (maxVoltage / majorVDivision)) + 2) * majorVDivision;
    plot.setAxes(0.0, vaAxisMax, majorVDivision, 0.0, iaAxisMax, majorIDivision, 2, 1);

    QPen samplePen;
    samplePen.setColor(QColor::fromRgb(0, 0, 0));

    QList<QGraphicsItem *> *segments = analyser->getResult()->plot(&plot);

    ui->showMeasuredValues->setChecked(true);

    measuredCurves = plot.getScene()->createItemGroup(*segments);
}

void ValveAnalyser::plotTransfer()
{
    double majorVDivision = 1.0;
    double majorIDivision = 1.0;

    double maxVoltage = -gridStop;
    double maxCurrent = analyser->getMeasuredIaMax();

    if (maxCurrent > 50.0) { // Over 20mA we plot the Ia axis in steps of 10mA
        majorIDivision = 10.0;
    } else if (maxCurrent > 20.0) {
        majorIDivision = 5.0;
    }

    double iaAxisMax = (((int) (maxCurrent / majorIDivision)) + 1) * majorIDivision;
    //double vgAxisMax = (((int) (maxVoltage / majorVDivision)) - 1) * majorVDivision;
    double vgAxisMax = maxVoltage;
    plot.setAxes(vgAxisMax, 0.0, majorVDivision, 0.0, iaAxisMax, majorIDivision);

    QPen samplePen;
    samplePen.setColor(QColor::fromRgb(0, 0, 0));

    QList<QGraphicsItem *> segments;

    const QList<QList <Sample *>> *sweepResult = analyser->getSweepResult();

    int sweeps = sweepResult->length();
    for (int i = 0; i < sweeps; i++) {
        QList<Sample *> thisSweep = sweepResult->at(i);

        Sample *firstSample = thisSweep.at(0);

        double vg = firstSample->getVg1();
        double va = firstSample->getVa();
        double ia = firstSample->getIa();

        int samples = thisSweep.length();
        for (int j = 1; j < samples; j++) {
             Sample *sample = thisSweep.at(j);

             double vgNext = sample->getVg1();
             double iaNext = sample->getIa();

             segments.append(plot.createSegment(vg, ia, vgNext, iaNext, samplePen));

             vg = vgNext;
             ia = iaNext;
         }

        plot.createLabel(vg, ia, va);
    }

    ui->showMeasuredValues->setChecked(true);

    measuredCurves = plot.getScene()->createItemGroup(segments);
}

void ValveAnalyser::runModel()
{
    if (model == nullptr) {
        model = ModelFactory::createModel(ui->modelSelection->currentData().toInt());
        //model = new DeviceModel(modelType, ui->modelSelection->currentData().toInt());
    }

    const QList<QList <Sample *>> *sweepResult = analyser->getSweepResult();

    int sweeps = sweepResult->length();

    for (int i = 0; i < sweeps; i++) {
        QList<Sample *> thisSweep = sweepResult->at(i);

        int samples = thisSweep.length();
        for (int j = 0; j < samples; j++) {
             Sample *sample = thisSweep.at(j);
             model->addSample(sample->getVa(), sample->getIa(), sample->getVg1(), sample->getVg2());

             /*if (dataSetDeviceType == TRIODE) {
                 model->addTriodeSample(sample->getVa(), sample->getVg1(), sample->getIa());
             } else if (dataSetDeviceType == PENTODE) {
                 model->addPentodeSample(sample->getVa(), sample->getVg1(), sample->getVg2(), sample->getIa());
             }*/
        }
    }

    model->solve();

    model->updateUI(parameterLabels, parameterValues);

    ui->showModelledValues->setChecked(true);

    ui->saveModelButton->setEnabled(true);

    plotModel();
}

void ValveAnalyser::runModel2()
{
    if (model == nullptr) {
        model = ModelFactory::createModel(ui->modelSelection->currentData().toInt());
        //model = new DeviceModel(modelType, ui->modelSelection->currentData().toInt());
    }

    SampleSet *result = analyser->getResult();

    result->beginRead();

    Sample *sample = result->readNextSample();
    while (sample != nullptr) {
        model->addSample(sample->getVa(), sample->getIa(), sample->getVg1(), sample->getVg2());
        sample = result->readNextSample();
    }

    model->solve();

    model->updateUI(parameterLabels, parameterValues);

    ui->showModelledValues->setChecked(true);

    ui->saveModelButton->setEnabled(true);

    plotModel();
}

void ValveAnalyser::plotModel() {
    switch (testType) {
    case ANODE_CHARACTERISTICS:
        plotAnodeModel();
        break;
    case TRANSFER_CHARACTERISTICS:
        plotTransferModel();
        break;
    default:
        break;
    }
}

void ValveAnalyser::plotAnodeModel()
{
    QList<QGraphicsItem *> segments;

    QPen modelPen;
    modelPen.setColor(QColor::fromRgb(255, 0, 0));

    if (modelledCurves) {
       plot.getScene()->removeItem(modelledCurves);
    }

    const QList<QList <Sample *>> *sweepResult = analyser->getSweepResult();

    int sweeps = sweepResult->length();

    for (int i=0; i < sweeps; i++) {
        QList<Sample *> thisSweep = sweepResult->at(i);
        double vg = thisSweep.at(0)->getVg1();
        double va = thisSweep.at(0)->getVa();
        double ia = model->anodeCurrent(va, vg);

        for (int j=1; j < 101; j++) {
            double vaNext = (anodeStop * j) / 100.0;
            double iaNext = model->anodeCurrent(va, vg);
            segments.append(plot.createSegment(va, ia, vaNext, iaNext, modelPen));

            va = vaNext;
            ia = iaNext;
        }
    }

    modelledCurves = plot.getScene()->createItemGroup(segments);
}

void ValveAnalyser::plotTransferModel()
{
    QList<QGraphicsItem *> segments;

    QPen modelPen;
    modelPen.setColor(QColor::fromRgb(255, 0, 0));

    if (modelledCurves) {
       plot.getScene()->removeItem(modelledCurves);
    }

    const QList<QList <Sample *>> *sweepResult = analyser->getSweepResult();

    int sweeps = sweepResult->length();

    for (int i=0; i < sweeps; i++) {
        QList<Sample *> thisSweep = sweepResult->at(i);
        double vg = thisSweep.at(0)->getVg1();
        double va = thisSweep.at(0)->getVa();
        double ia = model->anodeCurrent(va, vg);

        for (int j=1; j < 101; j++) {
            double vgNext = (-gridStop * (100 - j)) / 100.0;
            double iaNext = model->anodeCurrent(va, vg);
            segments.append(plot.createSegment(vg, ia, vgNext, iaNext, modelPen));

            vg = vgNext;
            ia = iaNext;
        }
    }

    modelledCurves = plot.getScene()->createItemGroup(segments);
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

void ValveAnalyser::on_actionOptions_triggered()
{
    PreferencesDialog dialog;

    dialog.setPort(port);

    if (dialog.exec() == 1) {
        setSerialPort(dialog.getPort());

        analyser->reset();
    }
}

void ValveAnalyser::on_heaterButton_clicked()
{
    heaters = !heaters;

    heaterIndicator->setState(heaters);
    ui->runButton->setEnabled(heaters);
    analyser->setHeaterVoltage(heaterVoltage);
    analyser->setIsHeatersOn(heaters);
}

void ValveAnalyser::on_runButton_clicked()
{
    if (heaters) {
        ui->runButton->setChecked(true);
        ui->progressBar->reset();
        ui->progressBar->setVisible(true);

        analyser->setDeviceType(deviceType);
        analyser->setTestType(testType);
        analyser->setPMax(pMax);
        analyser->setIaMax(iaMax);
        analyser->setSweepParameters(anodeStart, anodeStop, anodeStep, gridStart, gridStop, gridStep, screenStart, screenStop, screenStep);

        analyser->startTest();

        ui->saveModelButton->setEnabled(false);
    } else {
        ui->runButton->setChecked(false);
    }
}

void ValveAnalyser::handleReadyRead()
{
    analyser->handleReadyRead();
}

void ValveAnalyser::handleError(QSerialPort::SerialPortError error)
{
    analyser->handleError(error);
}

void ValveAnalyser::handleTimeout()
{
    analyser->handleCommandTimeout();
}

void ValveAnalyser::handleHeaterTimeout()
{
    analyser->handleHeaterTimeout();
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

    ui->testType->setCurrentIndex(0);
    on_testType_currentIndexChanged(0);
}

void ValveAnalyser::on_testType_currentIndexChanged(int index)
{
    switch (ui->testType->itemData(index).toInt()) {
    case ANODE_CHARACTERISTICS: // Anode swept and Grid stepped
        ui->anodeStop->setEnabled(true);
        ui->anodeStep->setEnabled(false);
        if (deviceType != DIODE) {
            ui->gridStop->setEnabled(true);
            ui->gridStep->setEnabled(true);
        }
        if (deviceType == PENTODE) { // Screen fixed (if Pentode)
            ui->screenStop->setEnabled(false);
            ui->screenStep->setEnabled(false);
        }
        break;
    case TRANSFER_CHARACTERISTICS: // Grid swept
        ui->gridStop->setEnabled(true);
        ui->gridStep->setEnabled(false);
        if (deviceType == PENTODE) { // Anode fixed and Screen stepped
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

    testType = ui->testType->itemData(index).toInt();
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

void ValveAnalyser::on_modelSelection_currentIndexChanged(int index)
{
    for (int i=0; i < 7; i++) { // Parameters all initially hidden
        parameterValues[i]->setVisible(false);
        parameterLabels[i]->setVisible(false);
    }

    model = nullptr;
}

void ValveAnalyser::on_fitModelButton_clicked()
{
    if (analyser->getIsDataSetValid()) {
        runModel2();
    }
}

void ValveAnalyser::on_showMeasuredValues_clicked(bool checked)
{
    if (measuredCurves) {
        measuredCurves->setVisible(checked);
    }
}

void ValveAnalyser::on_showModelledValues_clicked(bool checked)
{
    if (modelledCurves) {
        modelledCurves->setVisible(checked);
    }
}


void ValveAnalyser::on_saveModelButton_clicked()
{
    QFileDialog dialog(this, tr("Save Model"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setNameFilter("*.json");

    QStringList fileNames;
    if (dialog.exec() == 1) {
        //fileNames = dialog.selectedFiles();
        //QMessageBox message(this);
        //message.setText(fileNames.at(0));
        //message.exec();
        saveModel(dialog.selectedFiles().at(0));
    }
}

void ValveAnalyser::on_saveSamplesButton_clicked()
{
    QFileDialog dialog(this, tr("Save Samples"));
    dialog.setFileMode(QFileDialog::AnyFile);
    dialog.setAcceptMode(QFileDialog::AcceptSave);
    dialog.setNameFilter("*.json");

    QStringList fileNames;
    if (dialog.exec() == 1) {
        //fileNames = dialog.selectedFiles();
        //QMessageBox message(this);
        //message.setText(fileNames.at(0));
        //message.exec();
        saveModel(dialog.selectedFiles().at(0));
    }
}

