#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "libusb.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    int r = libusb_init(NULL);

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_actionPrint_triggered()
{

}

void MainWindow::on_actionQuit_triggered()
{
    QCoreApplication::quit();
}
