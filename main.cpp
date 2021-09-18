#include "valveanalyser.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ValveAnalyser w;
    w.show();
    return a.exec();
}
