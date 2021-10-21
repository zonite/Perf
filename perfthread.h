#ifndef PERFTHREAD_H
#define PERFTHREAD_H

#include <QApplication>
#include <QtCore/QCoreApplication>
#include <QThread>
#include <QObject>
#include <QString>

#include "perfanal.h"

class PerfThread : public QObject
{
public:
    PerfThread(QString path);
    ~PerfThread();
//    void run() override;
    void start();
    void show();

private:
    QApplication *m_pApp;
    QString m_QtPluginPath;
    //QCoreApplication *m_pApp;
    PerfAnal *m_pW;
};

#endif // PERFTHREAD_H
