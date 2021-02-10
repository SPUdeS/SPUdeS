#ifndef YOURMAINWINDOW_H
#define YOURMAINWINDOW_H

#include "mainwindow.h"

#include <QDebug>

class YourMainWindow : public MainWindow {
public:
    static const int UPDATE_RATE_MS = 10000;

    YourMainWindow()
        : MainWindow("/dev/ttyACM0", UPDATE_RATE_MS) // port name, update rate
    {
    }
    // Available functions

    // sendMessage(QString JSONstring); send JSON messages to the arduino
    // setUpdateRate(int rateMs); change preriodic loop frequency

    void onMessageReceived(QString msg) override {

        //Function executed on message received
        qDebug().noquote() << "Message received from arduino" << msg;

    }

    void onPeriodicUpdate() override {

        //Main periodic loop
        qDebug().noquote() << "Periodic loop (nothing for now)";

    }
};

#endif // YOURMAINWINDOW_H
