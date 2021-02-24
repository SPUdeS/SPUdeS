#include "csvwriter.h"
CsvWriter::CsvWriter(QString folder_name){
    folder = folder_name;
    filename = QTime::currentTime().toString();
    file.setFileName(folder + filename + ".csv");
    outStream.setDevice(&file);
    file.open(QIODevice::WriteOnly);
}


CsvWriter::~CsvWriter(){
    file.close();
}

void CsvWriter::write(QJsonObject msg){
    if(firstLine){
        setHeader(msg);
        firstLine = false;
    }
    QVariantMap result = msg.toVariantMap();
    QString msg_str = "";
    for(QVariantMap::const_iterator iter = result.begin(); iter != result.end(); ++iter) {
        msg_str += iter.value().toString();
        msg_str += ",";
    }
    msg_str = msg_str.left(msg_str.length() - 1); // remove last coma
    msg_str += "\r\n";
    outStream << msg_str;
}

void CsvWriter::setHeader(QJsonObject msg){
    QVariantMap result = msg.toVariantMap();
    QString msg_str = "";
    for(QVariantMap::const_iterator iter = result.begin(); iter != result.end(); ++iter) {
        msg_str += iter.key();
        msg_str += ",";
    }
    msg_str = msg_str.left(msg_str.length() - 1); // remove last coma
    msg_str += "\r\n";
    outStream << msg_str;
}
void CsvWriter::close(){
    file.close();
}
