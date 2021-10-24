#include "preferencesdialog.h"
#include "ui_preferencesdialog.h"

PreferencesDialog::PreferencesDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PreferencesDialog)
{    
    ui->setupUi(this);

    QList<QSerialPortInfo> serialPorts = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &serialPort : serialPorts) {
        ui->portSelect->addItem(serialPort.portName());
    }
}

PreferencesDialog::~PreferencesDialog()
{
    delete ui;
}

void PreferencesDialog::setPort(QString port)
{
    for (int i = 0; i < ui->portSelect->count(); i++) {
        if (ui->portSelect->itemText(i) == port) {
            ui->portSelect->setCurrentIndex(i);
        }
    }
}

QString PreferencesDialog::getPort()
{
    return ui->portSelect->currentText();
}
