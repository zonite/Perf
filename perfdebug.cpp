#include "perfdebug.h"

#include <QHBoxLayout>
#include <QPushButton>
#include <QQuaternion>
#include <QLabel>

#include <Physics>

//QTextBrowser *PerfDebug::m_pTextBrowser = nullptr;
QPlainTextEdit *PerfDebug::m_pTextLog = nullptr;

PerfDebug::PerfDebug(QWidget *parent) : QWidget(parent)
{
    m_pPlugin = PerfPlugin::get();
    if (!m_pTextLog) {
        m_pTextLog = new QPlainTextEdit();
        m_pTextLog->setMaximumBlockCount(5000);
    }
    //m_pTextedit = new QTextEdit();

    QPushButton *button = new QPushButton("Test");
    QPushButton *reset = new QPushButton("Export");
    QHBoxLayout *layout = new QHBoxLayout(this);
    QVBoxLayout *left = new QVBoxLayout(this);

    fileChooser = new QFileDialog(this);
    fileChooser->setLabelText(QFileDialog::FileName, "Choose Export Directory");
    fileChooser->setLabelText(QFileDialog::FileType, "Directory");
    fileChooser->setFileMode(QFileDialog::Directory);
    fileChooser->setOptions(QFileDialog::DontResolveSymlinks | QFileDialog::ShowDirsOnly);
    fileChooser->setDirectory(PerfPlugin::getExportDir());

    t1 = new QDoubleSpinBox(this);
    t2 = new QDoubleSpinBox(this);
    t3 = new QDoubleSpinBox(this);
    t4 = new QDoubleSpinBox(this);
    t5 = new QDoubleSpinBox(this);
    t6 = new QDoubleSpinBox(this);
    t7 = new QDoubleSpinBox(this);
    t8 = new QDoubleSpinBox(this);
    t9 = new QDoubleSpinBox(this);
    t1->setMaximum(50000);
    t2->setMaximum(50000);
    t3->setMaximum(50000);
    t4->setMaximum(1000);
    t1->setMinimum(-1000);
    t2->setMinimum(-1000);
    t3->setMinimum(-1000);
    t4->setMinimum(-1000);
    t1->setValue(40000);
    t2->setValue(0);
    t3->setValue(20000);
    t4->setValue(200);

    QLabel *l1 = new QLabel(this);
    QLabel *l2 = new QLabel(this);
    QLabel *l3 = new QLabel(this);
    QLabel *l4 = new QLabel(this);
    QLabel *l5 = new QLabel(this);
    QLabel *l6 = new QLabel(this);
    QLabel *l7 = new QLabel(this);
    QLabel *l8 = new QLabel(this);
    QLabel *l9 = new QLabel(this);
    l1->setText("Max Alt");
    l2->setText("Min Alt");
    l3->setText("Alt inc");
    l4->setText("Speed Inc");
    l5->setText("PreDump");
    l6->setText("PostDump");
    l7->setText("");
    l8->setText("");
    l9->setText("");

    left->addStretch();
    left->addWidget(l1);
    left->addWidget(t1);
    left->addWidget(l2);
    left->addWidget(t2);
    left->addWidget(l3);
    left->addWidget(t3);
    left->addWidget(l4);
    left->addWidget(t4);
    left->addWidget(button);
    left->addWidget(l5);
    left->addWidget(t5);
    left->addWidget(l6);
    left->addWidget(t6);
    left->addWidget(l7);
    left->addWidget(t7);
    left->addWidget(l8);
    left->addWidget(t8);
    left->addWidget(l9);
    left->addWidget(t9);
    left->addWidget(reset);
    left->addStretch();

    layout->addLayout(left);
    layout->addWidget(m_pTextLog);

    connect(button, &QPushButton::clicked, this, &PerfDebug::newTest, Qt::QueuedConnection);
    //connect(reset, &QPushButton::clicked, m_pPlugin, &PerfPlugin::controlButton, Qt::QueuedConnection);
    connect(reset, &QPushButton::clicked, this, &PerfDebug::showChooser, Qt::QueuedConnection);
    connect(m_pPlugin, &PerfPlugin::outData, this, &PerfDebug::newData, Qt::QueuedConnection);
    connect(fileChooser, &QFileDialog::fileSelected, this, &PerfDebug::newExport, Qt::QueuedConnection);

    connect(t5, &QDoubleSpinBox::valueChanged, m_pPlugin, &PerfPlugin::controlPara1, Qt::QueuedConnection);
    connect(t6, &QDoubleSpinBox::valueChanged, m_pPlugin, &PerfPlugin::controlPara2, Qt::QueuedConnection);
    connect(t7, &QDoubleSpinBox::valueChanged, m_pPlugin, &PerfPlugin::controlPara3, Qt::QueuedConnection);
    connect(t8, &QDoubleSpinBox::valueChanged, m_pPlugin, &PerfPlugin::controlPara4, Qt::QueuedConnection);
    connect(t9, &QDoubleSpinBox::valueChanged, m_pPlugin, &PerfPlugin::controlPara5, Qt::QueuedConnection);

    qInstallMessageHandler(PerfDebug::TextEditOutput);


    //((QMainWindow *)parent)->statusBar()->showMessage("Ready");
}

void PerfDebug::showChooser()
{
    fileChooser->show();
}

void PerfDebug::newExport(QString dir)
{
    qDebug() << "Exporting data..." << dir;
    if (!dir.isEmpty()) {
        qDebug() << "Directory valid.";
        PerfPlugin::exportDirectory(dir);
    }
    //fileChooser->hide();
}

void PerfDebug::newData()
{
    //m_pTextedit->append("Data arrived");
    m_pTextLog->appendPlainText("Data arrived");
    TestPointList* points = m_pPlugin->getData();

    for (qsizetype i = 0; i < points->size(); ++i) {
        m_pTextLog->appendPlainText(QVariant(points->at(i)->fnrml_aero).toString());
    }

    delete points;
}

void PerfDebug::newTest()
{
    //TestPoint* point;
    Aircraft* ac;

    ac = PerfPlugin::getAircraft();

    ac->setMaxAlt(Length(Unit("ft", 0.3048f), t1->value()));
    ac->setMinAlt(Length(Unit("ft", 0.3048f), t2->value()));
    ac->setAltInc(Length(Unit("ft", 0.3048f), t3->value()));
    ac->setSpeedInc(Quantity(Unit("kt", 1.852f/3.6f), t4->value()));

    PerfPlugin::showMessage("Creating Test Points! Please wait...");
    Config *cfg = ac->addConfig();
    PerfPlugin::showMessage("Done.");

    /*
    point = new TestPoint();
    //point->pitch = t1->value();
    //point->roll = t2->value();
    //point->yaw = t3->value();
    point->speed = t1->value();
    point->alt = t2->value();
    point->throttle = t3->value();
    point->alpha = t4->value();
    point->gear = 0.0f;
    point->flaps = 0.0f;
    point->pitchTrim = 0.0f;
    point->rollTrim = 0.0f;
    point->yawTrim = 0.0f;
    */
    m_pPlugin->newConfig(cfg);
    //m_pPlugin->newPoint(cfg->firstPoint());

    PerfPlugin::showMessage("Test requested.");
    m_pTextLog->appendPlainText("Start test");
    //m_pTextedit->append("Start test");
    //QQuaternion test = QQuaternion::fromEulerAngles(60, -200, 40);
    //float lat, lon, elev;
    //test.getEulerAngles(&lat, &lon, &elev);
    //qDebug() << test.toEulerAngles() << lat << lon << elev;

    //test = QQuaternion::fromEulerAngles(100, -20, -40);

    //test.getEulerAngles(&lat, &lon, &elev);
    //qDebug() << test.toEulerAngles() << lat << lon << elev;
    //this->setStatusTip("Start Test");
}

void PerfDebug::TextEditOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    //QByteArray localMsg = msg.toLocal8Bit();
    QString file = context.file ? context.file : "";
    QString function = context.function ? context.function : "";
    switch (type) {
    case QtDebugMsg:
        m_pTextLog->appendPlainText(QString("Debug: %1").arg(msg));
        //m_pTextedit->append(QString("Debug: %1 (%2:%3, %4)").arg(msg).arg(file).arg(context.line).arg(function));
        //fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtInfoMsg:
        m_pTextLog->appendPlainText(QString("Info: %1 (%2:%3, %4)").arg(msg, file, QString::number(context.line), function));
        //fprintf(stderr, "Info: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtWarningMsg:
        m_pTextLog->appendPlainText(QString("Warning: %1 (%2:%3, %4)").arg(msg, file, QString::number(context.line), function));
        //m_pTextedit->append(QString("Warning: %1 (%2:%3, %4)").arg(msg).arg(file).arg(context.line).arg(function));
        //fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtCriticalMsg:
        m_pTextLog->appendPlainText(QString("Critical: %1 (%2:%3, %4)").arg(msg, file, QString::number(context.line), function));
        //fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtFatalMsg:
        m_pTextLog->appendPlainText(QString("Fatal: %1 (%2:%3, %4)").arg(msg, file, QString::number(context.line), function));
        //fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    }
}
