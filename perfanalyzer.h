#ifndef PERFANALYZER_H
#define PERFANALYZER_H

#include <QMainWindow>
#include <QMenu>

#include <QTextEdit>

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

#include "perfplugin.h"

//QT_BEGIN_NAMESPACE
//namespace Ui { class PerfAnalyzer; }
//QT_END_NAMESPACE

class PerfPlugin;

class PerfAnalyzer : public QMainWindow
{
    Q_OBJECT
public:
    explicit PerfAnalyzer(QWidget *parent = nullptr);
    ~PerfAnalyzer();

signals:


protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void about();

private:
    void createActions();
    void createMenus();
    //Ui::PerfAnal *ui;
    QMenu *fileMenu;
    QMenu *helpMenu;

    QAction *exitAct;
    QAction *aboutAct;
    QAction *aboutQtAct;


    QString statusMessage;
    PerfPlugin *m_pPlugin = nullptr;
};

#endif // PERFANALYZER_H
