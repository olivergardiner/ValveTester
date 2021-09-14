#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    libusb_init(NULL); // Should really check that it starts ok

    libusb_device **devices;

    ssize_t cnt = libusb_get_device_list(NULL, &devices);
    if (cnt >= 0){
        print_devs(devices);
    }

    libusb_free_device_list(devices, 1);

    QTextStream out(stdout);
    const auto serialPortInfos = QSerialPortInfo::availablePorts();

    out << "Total number of ports available: " << serialPortInfos.count() << "\n";

    const QString blankString = "N/A";
    QString description;
    QString manufacturer;
    QString serialNumber;

    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        description = serialPortInfo.description();
        manufacturer = serialPortInfo.manufacturer();
        serialNumber = serialPortInfo.serialNumber();
        out << "\nPort: " << serialPortInfo.portName()
            << "\nLocation: " << serialPortInfo.systemLocation()
            << "\nDescription: " << (!description.isEmpty() ? description : blankString)
            << "\nManufacturer: " << (!manufacturer.isEmpty() ? manufacturer : blankString)
            << "\nSerial number: " << (!serialNumber.isEmpty() ? serialNumber : blankString)
            << "\nVendor Identifier: " << (serialPortInfo.hasVendorIdentifier()
                                         ? QByteArray::number(serialPortInfo.vendorIdentifier(), 16)
                                         : blankString)
            << "\nProduct Identifier: " << (serialPortInfo.hasProductIdentifier()
                                          ? QByteArray::number(serialPortInfo.productIdentifier(), 16)
                                          : blankString) << "\n";
    }

}

MainWindow::~MainWindow()
{
    libusb_exit(NULL);

    delete ui;
}


void MainWindow::on_actionPrint_triggered()
{

}

void MainWindow::on_actionQuit_triggered()
{
    QCoreApplication::quit();
}

static void print_devs(libusb_device **devs)
{
    libusb_device *dev;
    int i = 0, j = 0;
    uint8_t path[8];

    while ((dev = devs[i++]) != NULL) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
            fprintf(stderr, "failed to get device descriptor");
            return;
        }

        fprintf(stderr, "%04x:%04x (bus %d, device %d)",
            desc.idVendor, desc.idProduct,
            libusb_get_bus_number(dev), libusb_get_device_address(dev));

        r = libusb_get_port_numbers(dev, path, sizeof(path));
        if (r > 0) {
            fprintf(stderr, " path: %d", path[0]);
            for (j = 1; j < r; j++)
                fprintf(stderr, ".%d", path[j]);
        }

        libusb_device_handle *handle;
        r = libusb_open(dev, &handle);

        if (r >= 0) {
            unsigned char utf16String[256];
            libusb_get_string_descriptor(handle, desc.iManufacturer, 0, utf16String, 255);

            int end = utf16String[0];

            QString description = QString::fromUtf16((const char16_t *)(&utf16String[2]), (end / 2) - 1);

            fprintf(stderr, " description %s", description.toStdString().c_str());

            libusb_close(handle);
        }

        fprintf(stderr, "\n");
    }
}
