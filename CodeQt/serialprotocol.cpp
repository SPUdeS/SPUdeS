#include "serialprotocol.h"
#include <QDebug>
#include <QJsonObject>
#include <QJsonDocument>

SerialProtocol::SerialProtocol(QString portName, qint32 baudRate) {
    // Ouverture du port serie
    serial_ = new QSerialPort();
    serial_->setPortName(portName);
    serial_->setBaudRate(baudRate);
    serial_->open(QIODevice::ReadWrite);
    // Connection des signaux
    connectSignals();
}

SerialProtocol::~SerialProtocol(){
 delete serial_;
}

void SerialProtocol::connectSignals(){
    connect(serial_, SIGNAL(readyRead()), this, SLOT(readReceivedMsg()));
}

void SerialProtocol::sendMessage(QString msg) {
    // Fonction d'ecriture sur le port serie
    if (serial_->isOpen()) {
        serial_->write(msg.toUtf8());
    }
}

void SerialProtocol::readReceivedMsg(){
    // Fonction de lecture du port serie
    emit newMessage(serial_->readAll());
}
