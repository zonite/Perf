#include <QSettings>
#include <QMessageBox>
#include <QHBoxLayout>
#include <QPushButton>
//#include <QTextEdit>

#include "perfanal.h"
#include "testpoint.h"
#include "perfdebug.h"
//#include "ui_perfanal.h"

//PerfAnal::PerfAnal(QWidget *parent) : QMainWindow(parent), ui(new Ui::PerfAnal)
PerfAnal::PerfAnal(QWidget *parent) : QMainWindow(parent)
{
    //ui->setupUi();
    //setAttribute(Qt::WA_DeleteOnClose);
    m_pPlugin = PerfPlugin::get();

    statusMessage = tr("Initializing");
    statusBar()->showMessage(statusMessage);

    createActions();
    createMenus();

    setWindowTitle(tr("Performance Analyzer"));
    setMinimumSize(160, 160);

    QSettings settings("Ilmavoimat", "Perf");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    setUnifiedTitleAndToolBarOnMac(true);

    QWidget *window = new PerfDebug();

    setCentralWidget(window);
}

PerfAnal::~PerfAnal()
{
    //delete ui;
    //QSettings settings(QCoreApplication::organizationName(), QCoreApplication::applicationName());
    //settings.setValue("geometry", saveGeometry());
    //settings.setValue("windowState", saveState());
    delete exitAct;
    delete aboutAct;
}

void PerfAnal::closeEvent(QCloseEvent *event)
{
    QSettings settings(QCoreApplication::organizationName(), QCoreApplication::applicationName());
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    QMainWindow::closeEvent(event);
}

void PerfAnal::about()
{
    QMessageBox::about(this, tr("About Application"),
             tr("The <b>Performance Plugin</b> "
                "Commands X-Plane simulation "
                "to fly test flight patterns."));

}

void PerfAnal::createActions()
{
    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcuts(QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit Application"));
    connect(exitAct, &QAction::triggered, this, &QApplication::quit);

    aboutAct = new QAction(tr("&About"), this);
    //exitAct->setShortcuts(QKeySequence::About);
    aboutAct->setStatusTip(tr("About..."));
    connect(aboutAct, &QAction::triggered, this, &PerfAnal::about);

    //aboutQtAct = new QAction(tr("About &Qt"));
    //exitAct->setShortcuts(QKeySequence::AboutQt);
    //aboutQtAct->setStatusTip(tr("About Qt"));
    //connect(aboutQtAct, &QAction::triggered, this, &PerfAnal::aboutQt);
}


void PerfAnal::createMenus()
{
    fileMenu = menuBar()->addMenu(tr("&File"));
    //fileMenu->addAction(newAct);
    //fileMenu->addAction(openAct);
    //fileMenu->addAction(saveAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
    aboutQtAct = helpMenu->addAction(tr("About &Qt"), qApp, &QApplication::aboutQt);
    aboutQtAct->setStatusTip(tr("Show the Qt library's About box"));
}
