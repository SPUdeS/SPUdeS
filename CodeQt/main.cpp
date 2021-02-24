#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{   int delayMs = 1000;
    QApplication a(argc, argv);
    MainWindow w(delayMs);
    w.show();
    return a.exec();
}
