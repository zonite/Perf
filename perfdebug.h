#ifndef PERFDEBUG_H
#define PERFDEBUG_H

#include <QWidget>
#include <QDebug>

#include <QPlainTextEdit>
#include <QDoubleSpinBox>
#include <QFileDialog>

#include "perfplugin.h"

class PerfDebug : public QWidget
{
    Q_OBJECT
public:
    explicit PerfDebug(QWidget *parent = nullptr);
    static void TextEditOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);

public slots:
    void newData();
    void showChooser();

private slots:
    void newTest();
    void newExport(QString dir);

signals:

private:
    //static QTextBrowser *m_pTextBrowser;
    static QPlainTextEdit *m_pTextLog;
    QDoubleSpinBox *t1;
    QDoubleSpinBox *t2;
    QDoubleSpinBox *t3;
    QDoubleSpinBox *t4;
    QDoubleSpinBox *t5;
    QDoubleSpinBox *t6;
    QDoubleSpinBox *t7;
    QDoubleSpinBox *t8;
    QDoubleSpinBox *t9;
    PerfPlugin *m_pPlugin = nullptr;
    QFileDialog *fileChooser = nullptr;
};

#endif // PERFDEBUG_H
