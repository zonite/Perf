#include "perfthread.h"
#include "perfanalyzer.h"
#include "debug.h"

PerfThread::PerfThread(QString path)
{
    m_QtPluginPath = path;
}

PerfThread::~PerfThread()
{
    //delete m_pApp;
    //m_pApp = nullptr;
    //if (qApp)
    //    delete qApp;
}

void PerfThread::show()
{
    m_pW->show();
}

void PerfThread::start()
{
//    int argc = 0;
//    char *argv = nullptr;
//    qputenv("PATH", "C:\\Program Files\\X-Plane 10\\Resources\\plugins\\Position\\64");
//    qputenv("QT_PLUGIN_PATH", "C:\\Program Files\\X-Plane 10\\Resources\\plugins\\Position\\64");
//    qputenv("QT_PLUGIN_PATH", m_QtPluginPath.toUtf8());

//    Q_INIT_RESOURCE(perf);

    m_pApp = qApp;

    if (!m_pApp) {
        CONSOLE("qApp == null");
        int argc = 0;
        char *argv = nullptr;

        qputenv("QT_PLUGIN_PATH", m_QtPluginPath.toUtf8());
        CONSOLE("New QApplication");

        m_pApp = new QApplication(argc, &argv);
        QCoreApplication::setOrganizationName("Ilmavoimat");
        QCoreApplication::setApplicationName("Perf");
        QCoreApplication::setApplicationVersion(QT_VERSION_STR);

        CONSOLE("App created");
    }
    //m_pApp = new QCoreApplication(argc, &argv);

    CONSOLE("Create windows");

    m_pW = new PerfAnalyzer();
    m_pW->show();

}
/*
    CONSOLE("QApp exec");
    m_pApp->exec();

    CONSOLE("CloseallWins");
    m_pApp->closeAllWindows();
    CONSOLE("Destroy QApp");
    m_pApp->~QApplication();
    //exec();

    CONSOLE("Delete m_pApp");
    //delete m_pApp;
    CONSOLE("Set m_pApp null");
    m_pApp = nullptr;
    CONSOLE("Delete qApp");
    if (qApp) {
        CONSOLE("qApp not null");
        //delete qApp;
    } else {
        CONSOLE("qApp == null");
    }
    CONSOLE("Done");
}
*/
