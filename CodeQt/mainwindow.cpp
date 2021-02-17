#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Class constructor
    // UI initialization
    ui = new Ui::MainWindow;
    ui->setupUi(this);

    // Events/slots connection functions
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();
    connectTextInputs();
    connectComboBox();

    // Port census
    portCensus();

    // Timer initialization
    updateTimer_.start();
}

MainWindow::~MainWindow(){
    // Class destructor
    updateTimer_.stop();
    if(serialCom_!=nullptr){
      delete serialCom_;
    }
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Function called when window is destroyed
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg){
    // Function called on reception of a message on serial port
    // Accumulation of message bits
    msgBuffer_ += msg;

    //If message ends
    if(msgBuffer_.endsWith('\n')){
        // ASCII to JSon structure
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer_.toUtf8());

        // JSon message analysis
        if(~jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);

            // Show Json messages
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));

            // Show data in graph
            if(jsonObj.contains(JsonKey_)){
                double time = jsonObj["time"].toDouble();
                if(series_.points().size()>50){
                    series_.remove(0);
                }
                series_.append(time, jsonObj[JsonKey_].toDouble());
                //Graph format(not optimal)
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }

            // Message reception function
            msgReceived_ = msgBuffer_;
            onMessageReceived(msgReceived_);

            // If data needs to be recorded
            if(record){
                writer_->write(jsonObj);
            }
        }
        // Reinitialization of message receiver
        msgBuffer_ = "";
    }
}

void MainWindow::connectTimers(int updateRate){
    // Timer connection function
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead(){
    // Class message connection function (serialProtocol)
    connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    // Send button connection function
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(sendAngles()));
}

void MainWindow::sendAngles(){
    //Function that sends messages to the Arduino
    //***************Need to change the lineEdit names**********************
    int angle_1 = ui->lineEdit->text().toInt();
    int angle_2 = ui->lineEdit_2->text().toInt();
    int angle_3 = ui->lineEdit_3->text().toInt();
    int angle_4 = ui->lineEdit_4->text().toInt();
    int angle_5 = ui->lineEdit_5->text().toInt();
    int angle_6 = ui->lineEdit_6->text().toInt();

    QJsonArray array = { QString::number(angle_1),
                         QString::number(angle_2),
                         QString::number(angle_3),
                         QString::number(angle_4),
                         QString::number(angle_5),
                         QString::number(angle_6)
                         };
    QJsonObject jsonObject
    {
        {"setGoal", array}
    };
    QJsonDocument doc(jsonObject);
    QString strJson(doc.toJson(QJsonDocument::Compact));
    sendMessage(strJson);
}
void MainWindow::connectTextInputs(){
    // Fonction de connection des entrees de texte
    //Exemple
    connect(ui->lineEdit, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit->text();
    connect(ui->lineEdit_2, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit_2->text();
    connect(ui->lineEdit_3, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit_3->text();
    connect(ui->lineEdit_4, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit_4->text();
    connect(ui->lineEdit_5, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit_5->text();
    connect(ui->lineEdit_6, SIGNAL(returnPressed()), this, SLOT(changeJsonKeyValue()));
    JsonKey_ = ui->lineEdit_6->text();
}

void MainWindow::connectComboBox(){
    // Function that connects combo boxes
    connect(ui->comboBoxPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}

void MainWindow::portCensus(){
    // Function that makes a census of all available ports
    ui->comboBoxPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(port.portName());
    }
}

void MainWindow::startSerialCom(QString portName){
    // SLOT function to start serial communication
    qDebug().noquote() << "Connection to port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();
}

// I'm not so sure what this does
void MainWindow::changeJsonKeyValue(){
    // SLOT function to change the JSonKeyValue
    series_.clear();
    JsonKey_ = ui->lineEdit->text();
}

void MainWindow::sendMessage(QString msg){
    // SLOT function to write on the serial port
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Error no serial port!!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message from RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs){
    // Timer initialization function
    updateTimer_.start(rateMs);
}

void MainWindow::manageRecording(int stateButton){
    // SLOT function to find the record button's state and record or not according to it
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // SLOT function to create a new cvs file
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    //ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Function that lets us stop writing in a csv file
    record = false;
    delete writer_;
}
void MainWindow::onMessageReceived(QString msg){
    // Function called on message reception
    // Decomment the following ligne for debugging
     qDebug().noquote() << "Message from Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Periodically called SLOT function as defined in the constructor
    qDebug().noquote() << "*";
}
