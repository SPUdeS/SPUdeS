#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int updateRate, QWidget *parent):
    QMainWindow(parent)
{
    // Constructeur de la classe
    // Initialisation du UI
    ui = new Ui::MainWindow;
    ui->setupUi(this);

    // Fonctions de connections events/slots
    connectTimers(updateRate);
    connectButtons();
    connectSpinBoxes();
    connectTextInputs();
    connectComboBox();

    // Recensement des ports
    portCensus();

    // initialisation du timer
    updateTimer_.start();
}

MainWindow::~MainWindow(){
    // Destructeur de la classe
    updateTimer_.stop();
    if(serialCom_!=nullptr){
      delete serialCom_;
    }
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent *event){
    // Fonction appelee lorsque la fenetre est detruite
    event->accept();
}

void MainWindow::receiveFromSerial(QString msg){
    // Fonction appelee lors de reception sur port serie
    // Accumulation des morceaux de message
    msgBuffer_ += msg;

    //Si un message est termine
    if(msgBuffer_.endsWith('\n')){
        // Passage ASCII vers structure Json
        QJsonDocument jsonResponse = QJsonDocument::fromJson(msgBuffer_.toUtf8());

        // Analyse du message Json
        if(~jsonResponse.isEmpty()){
            QJsonObject jsonObj = jsonResponse.object();
            QString buff = jsonResponse.toJson(QJsonDocument::Indented);

            // Affichage des messages Json
            ui->textBrowser->setText(buff.mid(2,buff.length()-4));

            // Affichage des donnees dans le graph
            if(jsonObj.contains(JsonKey_)){
                double time = jsonObj["time"].toDouble();
                if(series_.points().size()>50){
                    series_.remove(0);
                }
                series_.append(time, jsonObj[JsonKey_].toDouble());
                // Mise en forme du graphique (non optimal)
                chart_.removeSeries(&series_);
                chart_.addSeries(&series_);
                chart_.createDefaultAxes();
            }

            // Fonction de reception de message (vide pour l'instant)
            msgReceived_ = msgBuffer_;
            onMessageReceived(msgReceived_);

            // Si les donnees doivent etre enregistrees
            if(record){
                writer_->write(jsonObj);
            }
        }
        // Reinitialisation du message tampon
        msgBuffer_ = "";
    }
}

void MainWindow::connectTimers(int updateRate){
    // Fonction de connection de timers
    connect(&updateTimer_, &QTimer::timeout, this, [this]{onPeriodicUpdate();});
    updateTimer_.start(updateRate);
}

void MainWindow::connectSerialPortRead(){
    // Fonction de connection au message de la classe (serialProtocol)
    connect(serialCom_, SIGNAL(newMessage(QString)), this, SLOT(receiveFromSerial(QString)));
}

void MainWindow::connectButtons(){
    // Fonction de connection du boutton Send
    //Exemple:
    connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(sendAngles()));

}

void MainWindow::sendAngles(){
    //Fonction qui envoie les angles au arduino
    //double goal = ui->lineEdit_DesVal->text().toDouble();
        int angle_1 = ui->lineEdit->text().toInt();
        int angle_2 = ui->lineEdit_2->text().toInt();
        int angle_3 = ui->lineEdit_3->text().toInt();
        int angle_4 = ui->lineEdit_4->text().toInt();
        int angle_5 = ui->lineEdit_5->text().toInt();
        int angle_6 = ui->lineEdit_6->text().toInt();
        //double thresh = ui->lineEdit_Thresh->text().toDouble();
        // pour minimiser le nombre de decimales( QString::number)

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

void MainWindow::connectSpinBoxes(){
    // Fonction de connection des spin boxes
    //Exemple
    //connect(ui->PWMBox, SIGNAL(valueChanged(double)), this, SLOT(sendPulseSetting()));
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
    // Fonction de connection des entrees deroulantes
    connect(ui->comboBoxPort, SIGNAL(activated(QString)), this, SLOT(startSerialCom(QString)));
}

void MainWindow::portCensus(){
    // Fonction pour recenser les ports disponibles
    ui->comboBoxPort->clear();
    Q_FOREACH(QSerialPortInfo port, QSerialPortInfo::availablePorts()) {
        ui->comboBoxPort->addItem(port.portName());
    }
}

void MainWindow::startSerialCom(QString portName){
    // Fonction SLOT pour demarrer la communication serielle
    qDebug().noquote() << "Connection au port"<< portName;
    if(serialCom_!=nullptr){
        delete serialCom_;
    }
    serialCom_ = new SerialProtocol(portName, BAUD_RATE);
    connectSerialPortRead();
}

void MainWindow::changeJsonKeyValue(){
    // Fonction SLOT pour changer la valeur de la cle Json
    series_.clear();
    //exmeple
    //JsonKey_ = ui->JsonKey->text();
    JsonKey_ = ui->lineEdit->text();
}



void MainWindow::sendMessage(QString msg){
    // Fonction SLOT d'ecriture sur le port serie
    if(serialCom_==nullptr){
        qDebug().noquote() <<"Erreur aucun port serie !!!";
        return;
    }
    serialCom_->sendMessage(msg);
    qDebug().noquote() <<"Message du RPI: "  <<msg;
}

void MainWindow::setUpdateRate(int rateMs){
    // Fonction d'initialisation du chronometre
    updateTimer_.start(rateMs);
}

void MainWindow::manageRecording(int stateButton){
    // Fonction SLOT pour determiner l'etat du bouton d'enregistrement
    if(stateButton == 2){
        startRecording();
    }
    if(stateButton == 0){
        stopRecording();
    }
}

void MainWindow::startRecording(){
    // Fonction SLOT pour creation d'un nouveau fichier csv
    record = true;
    writer_ = new CsvWriter("/home/pi/Desktop/");
    //ui->label_pathCSV->setText(writer_->folder+writer_->filename);
}

void MainWindow::stopRecording(){
    // Fonction permettant d'arreter l'ecriture du CSV
    record = false;
    delete writer_;
}
void MainWindow::onMessageReceived(QString msg){
    // Fonction appelee lors de reception de message
    // Decommenter la ligne suivante pour deverminage
     qDebug().noquote() << "Message du Arduino: " << msg;
}

void MainWindow::onPeriodicUpdate(){
    // Fonction SLOT appelee a intervalle definie dans le constructeur
    qDebug().noquote() << "*";
}
