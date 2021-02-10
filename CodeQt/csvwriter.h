#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <QFile>
#include <QTime>
#include <QTextStream>
#include <QJsonObject>
#include <QJsonDocument>

//Classe permettant d'enregistrer les messages Json en .csv
class CsvWriter: public QObject
{
    Q_OBJECT

public:
    CsvWriter(QString folder);
    ~CsvWriter();
    void write(QJsonObject);
    void close();
    QString folder;
    QString filename;
private:
    void setHeader(QJsonObject);
    QFile file;
    QTextStream outStream;
    bool firstLine = true;
};
#endif // CSVWRITER_H
