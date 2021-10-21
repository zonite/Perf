#ifndef PERFANAL_H
#define PERFANAL_H

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

QT_BEGIN_NAMESPACE
namespace Ui { class PerfAnal; }
QT_END_NAMESPACE

class PerfPlugin;

class PerfAnal : public QMainWindow
{
    Q_OBJECT
public:
    explicit PerfAnal(QWidget *parent = nullptr);
    ~PerfAnal();

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

#endif // PERFANAL_H
