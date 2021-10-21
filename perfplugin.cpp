#include <QString>
#include <clocale>
#include <QDir>
#include <QPluginLoader>
#include <QMetaMethod>
#include <QMessageBox>

#include <QVector3D>
#include <QQuaternion>

#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMUtilities.h"
#include "XPLMPlanes.h"
//#include "XPLMPlugin.h"

#include "xpdata.h"
#include "perfplugin.h"
#include "wrapper.h"
#include "debug.h"
#include "Control"
#include "autopilot.h"

//Config
const QByteArray PerfPlugin::pluginOrg = "Ilmavoimat";
const QByteArray PerfPlugin::pluginName = "Perf";
const QByteArray PerfPlugin::m_menuStart = "Start";
const QByteArray PerfPlugin::m_menuReset = "Reload";
const QVector3D PerfPlugin::m_cZeroV(0.0f, 0.0f, 0.0f);

int PerfPlugin::m_iNumSamples = 3;
float PerfPlugin::m_fAccuracy = 0.003f;

float PerfPlugin::m_fAlpha = 0.0f;
double PerfPlugin::m_dMinAlt = 20.0;

PerfPlugin* PerfPlugin::m_pPlugin = nullptr;
PerfAnal *PerfPlugin::m_pPerfApp = nullptr;
Aircraft *PerfPlugin::m_pCurAC = nullptr;
QString PerfPlugin::m_sQtPluginPath = "";
QString PerfPlugin::m_pExportPath = "";


QList<Config*> PerfPlugin::m_cCfgQueue;
TestPointList PerfPlugin::m_cQueue;
TestPointList PerfPlugin::m_cReady;
QVectorD3D PerfPlugin::m_cTestGLpos = QVectorD3D(0.0, 0.0, 0.0);
QVectorD3D PerfPlugin::m_cSavedpos = QVectorD3D(0.0, 0.0, 0.0);

PerfPlugin *PerfPlugin::get()
{
    if (!m_pPlugin)
        m_pPlugin = new PerfPlugin();

    return m_pPlugin;
}

void PerfPlugin::inc()
{
    XPdata::inc();
}

TestPointList *PerfPlugin::getData()
{
    TestPointList *list = new TestPointList();
    *list = m_cReady;
    m_cReady.clear();

    return list;
}

PerfPlugin::PerfPlugin(QObject *parent) : QObject(parent)
{
    //QObject::connect(this, &PerfPlugin::inData, this, &PerfPlugin::registerFL);
    QObject::connect(this, &PerfPlugin::inData, this, &PerfPlugin::scheduleFL, Qt::QueuedConnection);

    //preLooId = nullptr;
    //afterLooId = nullptr;
}

PerfPlugin::~PerfPlugin()
{
    /*
    CONSOLE("Instances");
    QList list = QPluginLoader::staticPlugins();
    QString size;
    size.setNum(list.size());
    CONSOLE(size);
    //if (qApp)
    //    delete qApp;
    */
    CONSOLE("perfPlug destructor");

    m_pPlugin = nullptr;
}

float PerfPlugin::preFlightLoop(float inElapsedSinceLastCall,
                             float inElapsedTimeSinceLastFlightLoop,
                             int inCounter,
                             void *inRefcon)
{
    Q_UNUSED(inElapsedSinceLastCall);
    Q_UNUSED(inElapsedTimeSinceLastFlightLoop);
    Q_UNUSED(inCounter);
    Q_UNUSED(inRefcon);

    if (isPaused())
        return -1.0f;
    XPdata::inc();

    if (reset) {
        if (m_cQueue.size()) {
            m_pCurrentCfg = m_cCfgQueue.first();
            m_cQueue.removeFirst();
        }
        //setAttitude(m_pCurrent->alpha, 0.0f, 0.0f);
        //m_fAlpha = m_pCurrent->alpha;
        m_fAlpha = 0.0f;
        reset = false;
    }

    //qDebug() << "Preloop, state" << m_eComState << "cur =" << m_eCurState;
    if (m_bPreDump)
        dumpData("PreDump");
    static int i = 0;

/*
    if (m_eState == PerfState::alt) {
        if (i > 100)
            m_cQueue.clear();
        ++i;

        if (m_cQueue.size()) {

            m_eState = PerfState::power;
        } else {

            m_eState = PerfState::init;
        }
        return -1.0f;
    }
*/
    if (m_eComState == PerfState::stable) {
        m_eCurState = PerfState::stable;
        //m_eState = PerfState::off;
        //return 0.0f;
        //setGLpos(m_cTestGLpos);
        //pause();
        //qDebug() << "Acceleration = " << getOpenGLacc() << "Angular acceleration" << getAngVel() << "Alpha = " << getAlphaF() << "Rotation" << getOpenGLatt().toEulerAngles();
    }

    if (m_eComState == PerfState::stabilize) {
        m_eCurState = PerfState::stabilize;

        //if (m_cQueue.size()) {
        if (m_pCurrentCfg && m_pCurrentCfg->getTestPoint()) {
            //setGLpos(m_cTestGLpos, X | Z);
            //setAlpha(m_pCurrent->alpha);

            m_pCurrentCfg->runTest();
            //pause();
            //setTestAltitude(m_pCurrent->alt);
            //setTestAtt();
            //setGLpos(m_cTestGLpos);
            //setTAS(m_pCurrent->speed, false);
            //setAngVel(m_cZeroV);

            //qDebug() << "Pre Acceleration = " << getOpenGLacc() << "AeroF" << getAeroForces() << "Engine" << getEngineForces() << "Total" << getTotalForces() << "Angular acceleration" << getAngVel() << "Alpha = " << getAlphaF() << "Rotation" << getOpenGLatt().toEulerAngles();
            //qDebug() << "Pre Total moments = " << getTotalMoments() << "Aero mom =" << getAeroMoments() << "Engine mom = " << getEngineMoments();

            //qDebug() << "GOTO Stable!";
            //m_eState = PerfState::stable;
            return -1.0f;
        } //else {
            //pause();
            //getGear();
            //setGear(1.0f);

            //m_eComState = PerfState::init;
        //}
        m_eComState = PerfState::check;
    }

    if (m_eComState == PerfState::check) {
        m_eCurState = PerfState::init;

        if (m_pCurrentCfg) {
            m_pCurrent = m_pCurrentCfg->getTestPoint(); // if m_pCurrent == nullptr, the profile ended
            TestPoint *checked = m_pCurrentCfg->checkTP();

            //qDebug() << "Checked TP" << (void*)checked;
            //CONSOLE(QString("Checked TP %1, Current TP %2").arg((long long)checked).arg((long long)m_pCurrent));
            if (checked) {
                if (m_pCurrent && checked->alt.raw() == m_pCurrent->alt.raw()) {
                //if (checked->alt.raw() == m_pCurrent->alt.raw()) {

                    //setTAS(checked->speed, false);
                    //setAngVel(m_cZeroV);

                    m_pCurrent = checked;
                    m_pCurrentCfg->runTest();
                    m_eComState = PerfState::stabilize;
                    m_eCurState = PerfState::stabilize;

                    QString msg = QString("Run: %1, CAS %2, M%3, %4").arg(m_pCurrent->alt.toStr()).arg(m_pCurrent->speed.toStr()).arg(Aerodynamics::casToMach(m_pCurrent->speed, getEnvironment())).arg(m_pCurrent->type ? QString("Auto Thr") : QString("%1%").arg(m_pCurrent->throttle * 100.0f));
                    m_pPerfApp->statusBar()->showMessage(msg);
                    qDebug() << msg;
                    return -1.0f;
                } else {
                    freeze();

                    m_pCurrent = checked;
                    setTestPosition();
                    //setGLpos(m_cTestGLpos);
                    return -1.0f;
                }
            } else {
                m_pCurrentCfg = nullptr;
                m_pCurrent = nullptr;
                m_eComState = PerfState::init;
            }
        } else {
            m_eComState = PerfState::init;
        }
    }

    if (m_eComState == PerfState::initAC) {
        m_eCurState = PerfState::initAC;
        if (m_pCurAC->check()) {
            m_pCurrent = m_pCurrentCfg->initTest();
            m_eComState = PerfState::check;
        }
        return -1.0;
    }

    //if (m_bTestConditions) {
    if (m_eComState == PerfState::init) {
        m_eCurState = PerfState::init;
        //qDebug() << "Pre freezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Roll =" << getAngVel() << "Rotation" << getOpenGLatt().toEulerAngles();

        if (m_cCfgQueue.size()) {
            showMessage("Starting Test...");
            setGLq(QQuaternion(1, 0, 0, 0));
            //m_cTestGLpos = getGLpos();
            m_cTestGLpos = m_cZeroV; //Move to 0, 0, 0 GL coordinates
            m_cTestGLpos.setY(getAltm()); //reset correct altitude, current altitude.
            qDebug() << "Init TestGLpos" << m_cTestGLpos.toVector3D() << getAltm();
            //m_cTestGLpos.setY(getAlt().raw()); //reset correct altitude
            setTestPosition();

            if (!m_pCurrentCfg) {
                qDebug() << "Check cur";
                if (m_cCfgQueue.first()->getAC() != m_pCurAC) {
                    setCurrentAC(m_cCfgQueue.first()->getAC());
                    return -1.0;
                }

                m_pCurrentCfg = m_cCfgQueue.first();
                m_cCfgQueue.removeFirst();

                //m_pCurrentCfg->initAC();
                //m_pProfile = m_pCurrentCfg->getCurProfile();
                //m_pCurrent = m_pProfile->last();

                //m_pCurrent = m_pCurrentCfg->initTest();
                //m_pCurrent = m_pCurAC->curTP();
                //initTestpoint(m_pCurrent);
            }

            //XPdata *path = XPdata::ref("sim/operation/override/override_planepath");
            //qDebug() << "Path pre" << path->getiList();


            //setApLevelFlight(); //No Autopilot
            //qDebug() << "Path aft" << path->getiList();

            //zeroAcceleration();
            //zeroForces();
            m_eComState = PerfState::initAC;
            //m_eComState = PerfState::check;
        } else {
            // TODO check and wait a change back to original user aircraft and continue only after that.
            showMessage("Reseting");
            setControls(false);
            setISAweather(false);
            setTestPos(false);

            m_eComState = PerfState::off;
        }
        return -1.0f;
    }

    if (m_eComState == PerfState::off) {
        m_eCurState = PerfState::off;
        if (m_cCfgQueue.size()) {
            //setConditions(true);
            saveVelAttAng(true);
            setTestPos(true);
            setISAweather(true);
            setControls(true);
            XPLMScheduleFlightLoop(afterLoopId, -1.0, true);

            i = 0;
            m_eComState = PerfState::init;
            m_eCurState = PerfState::skip;
        } else {
            showMessage("Ready");
            saveVelAttAng(false);
            m_bPreDump = true;
            m_bAfterDump = true;

            return 0.0f;
        }
    }
    return -1.0f;



    //Testing
    static int count = 0;
    ++count;

    qDebug() << "Pre freezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Count =" << count;

    if (count > 10) {
        count = 0;
        return 0.0;
    }
    freeze(false);

    qDebug() << "Pre unfreezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Count =" << count;

    return -1.0;

    //Testing End

    if (!m_pCurrent && m_cQueue.size()) {
        m_pCurrent = m_cQueue.first();
        m_cQueue.removeFirst();
    }

    if (m_pCurrent) {
        if (!m_bFltRunning) {
            setConditions(true);
            m_bFltRunning = true;
            XPLMScheduleFlightLoop(afterLoopId, -1.0, true);
        }

        initializeTestpoint(true);

        return -1.0f;
    }

    m_bFltRunning = false;
    setConditions(false);

    if (m_cQueue.size()) //Should never happen!
        return -1.0f;

    return 0.0f;
}

float PerfPlugin::afterFlightLoop(float inElapsedSinceLastCall,
                             float inElapsedTimeSinceLastFlightLoop,
                             int inCounter,
                             void *inRefcon)
{
    Q_UNUSED(inElapsedSinceLastCall);
    Q_UNUSED(inElapsedTimeSinceLastFlightLoop);
    Q_UNUSED(inCounter);
    Q_UNUSED(inRefcon);

    if (isPaused())
        return -1.0f;

    //qDebug() << "Afterloop, state =" << m_eComState << "cur =" << m_eCurState;
    XPdata::inc();
    if (m_bAfterDump)
        dumpData("PostDump");

    if (m_eCurState == PerfState::stabilize) {
        static int count = 0;
        //qDebug() << "Pos" << getGLpos().toVector3D() << "TestGLpos" << m_cTestGLpos.toVector3D() << "Acceleration = " << getOpenGLacc() << "AeroF" << getAeroForces() << "Engine" << getEngineForces() << "Total" << getTotalForces() << "Angular acceleration" << getAngVel() << "Alpha = " << getAlphaF() << "set alpha" << m_fAlpha << "Rotation" << getOpenGLatt().toEulerAngles();
        //qDebug() << "Total moments = " << getTotalMoments() << "Aero mom =" << getAeroMoments() << "Engine mom = " << getEngineMoments();
        //static PID lift(0.0f, 0.6f, 1.0f, 2.0f, 1.0f, 70.0f, -70.0f);
        //static PID pitch(0.0f, 0.5f, 1.0f, 0.02f, 0.05f, 2.0f, -2.0f);
        //static PID roll(0.0f, 0.3f, 1.0f, 4.0f, 1.0f, 2.0f, -2.0f);
        //static PID yaw(0.0f, 0.3f, 1.0f, 4.0f, 1.0f, 2.0f, -2.0f);
        //static LPFilter lp(60.0f, 0.1f, m_fAlpha);
        //static LPFilter pitchLP(60.0f, 20.0f, 0.0f);
        //static LPFilter rollLP(60.0f, 20.0f, 0.0f);
        //static LPFilter yawLP(60.0f, 20.0f, 0.0f);
        //static int dir = 0;
        //if (dir == 1 )
        //static float vpath_old = getVPathF();

        //X-Plane IAS is actually EAS!!!
        //Stable Engine: "sim/flightmodel/engine/ENGN_N2_[8]
        //Stable Engine: "sim/flightmodel/engine/ENGN_EPR[8]

        //qDebug() << "Post: Stabilize";
        //m_pCurrentCfg->set();
        //if (m_pCurrentCfg->isStable()) {
        //TestPoint *tp = m_pCurrentCfg->getTestPoint();
        if (m_pCurrentCfg->set()) {
            m_pCurrentCfg->getAC()->recordThrust();
            if (m_pCurrentCfg->getTestPoint()->state == completed)
                qDebug() << "Stable!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
            else
                qDebug() << "Failed... Unstable" << m_pCurrentCfg->getTestPoint()->state;
            //QVectorD3D v = getGLpos();
            int w = 22;
            //qDebug() << QString("Dump PosGL:\tx = %1m\ty = %2m\tz = %3m\tALT = %4m").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(getAltm(), w).toUtf8().constData();
            qDebug() << QString("Dump Vel:\tIAS = %1m/s\tTAS = %2m/s\tALT = %3m").arg(getIASms(), w).arg(getTASms(), w).arg(getAltm(), w).toUtf8().constData();
            //if (m_pCurrentCfg->getTestPoint()->throttle == 0.5f)
                //pause();
            //if (m_pCurrentCfg->getTestPoint()->state == completed)
            m_pCurrentCfg->captureData();
            m_pCurrentCfg->nextTestPoint();
            m_eComState = PerfState::check;
        }

        if (count < 20) {
            if (count > 18) {
                m_bPreDump = false;
                m_bAfterDump = false;
            }

            ++count;
        }
        /*
        int w = 22;
        QVector3D v = getOpenGLacc();
        qDebug() << QString("Dump Acc:\tx = %1m/s2\ty = %2m/s2\tz = %3m/s2\tLen = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(v.length(), w).toUtf8().constData();
        v = getTotalForces();
        qDebug() << QString("Dump Total Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getEngineForces();
        qDebug() << QString("Dump Engine Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        qDebug() << QString("Dump Engine:\tN1 = %1%\tN2 = %2%\tEPR = %3\tThrust = %4N\tTP = %5%").arg(getN1mean(), w).arg(getN2mean(), w).arg(getEPRmean(), w).arg(getThrustTotal(), w).arg(tp->throttle * 100.0f, w).toUtf8().constData();
        qDebug() << QString("Dump Att:\tAlpha = %1deg\tBeta = %2deg\tVpath = %3deg\tHpath = %4").arg(getAlpha(), w).arg(getBeta(), w).arg(getVPath(), w).arg(getHPath(), w).toUtf8().constData();
        v = getAngAcc();
        qDebug() << QString("Dump Ang Acc:\tQ' = %1d/s2\tR' = %2d/s2\tP' = %3d/s2\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        qDebug() << QString("Dump Trim:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getTrimPitch(), w).arg(getTrimYaw(), w).arg(getTrimRoll(), w).toUtf8().constData();
        qDebug() << QString("Dump TP:\tthr = %1\ttype = %2\talt = %3\tspd = %4\tAC thr = %5\tProfile T = %6").arg(tp->throttle, w).arg(tp->type ? "Auto": "FXD", w).arg(tp->alt.toStr(), w).arg(tp->speed.toStr(), w).arg(m_pCurrentCfg->getAC()->getThrottle(), w).arg(tp->getProfile()->getThrottle(), w).toUtf8().constData();
        //freezeAccNrot();
        //pause();
        */

        //if (m_pCurrentCfg->getTestPoint()->count == 5)
            //setPitch(getTrimPitch() - 0.1f);
            //setPitch(-0.276155052f);
            //setPitch(-0.174266177f);
            //setPitch(0.4f);

        /*
        if (m_pCurrentCfg->getTestPoint()->count > 10) {
            //m_eComState = PerfState::stable;
            //store_data();
            //qDebug() << "Testpoint" << m_pCurrentCfg->getTestPoint() << m_pCurrentCfg->getTestPoint()->speed.raw() << m_pCurrentCfg->getTestPoint()->alt.raw() << m_pCurrentCfg->getTestPoint()->throttle;
            //pause();
            static int i = 0;
            m_pCurrent = m_pCurrentCfg->nextTestPoint();
            if (m_pCurrent) {
                m_pCurrent->count = 11;
                //m_pCurrent->pitchTrim = 0.4f;
            }

            m_eComState = PerfState::check;
            if (i > 15)
                pause();
            ++i;
        }
        */

        //m_pCurAC->set();
        //if (m_pCurAC->isStable())
            //pause();
        //m_pAutoPilot->set();
        //if (m_pAutoPilot->isStable())
            //pause();

        //        APalpha(getAlphaF, setPitch, pitch);
        //APalpha.setScale(getMaxAlphaF() * 2.0f);

        //float vpath = getVPathF();
        //float hpath = getHPathF();
        //float rollAng = getRealAtt().toEulerAngles().z();

        /*
        if (vpath > 0 && vpath_old < 0)
            lift.max = m_fAlpha;
        if (vpath < 0 && vpath_old > 0)
            lift.min = m_fAlpha;
        vpath_old = vpath;
        */
        /*
        float control = pitch.get(pitchLP.get(vpath/10));
        float controlPitch = pitchLP.get(pitch.get(vpath/10));
        float controlRoll = rollLP.get(roll.get(rollAng/10));
        float controlYaw = yaw.get(hpath/10);
        */

        //APalpha.set();

        //qDebug() << "Velocity" << getGLvel();
        //qDebug() << "Max alpha" << getMaxAlphaF() << "Alpha" << getAlphaF();
        //qDebug() << "Yoke and trim" << getYokePitch() << getTrimPitch();
        //qDebug() << "Yoke and trim" << getYokePitch() << getTrimPitch() << "Control" << controlPitch;
        //qDebug() << "Yoke and trim" << getYokeRoll() << getTrimRoll() << "Control" << controlRoll;
        //qDebug() << "Yoke and trim" << getYokeYaw() << getTrimYaw() << "Control" << controlYaw;
        //setPitch(controlPitch);
        //setYaw(controlYaw);
        //setRoll(controlRoll);


        //m_fAlpha = lift.get(vpath);
        //m_fAlpha = lift.get(lp.get(vpath));
        //m_fAlpha = lift.get(getOpenGLacc().y());
        //m_fAlpha = lift.get(lp.get(getVPathF()));
        //lift.Kp *= 0.95f;
        //qDebug() << "New Alpha" << m_fAlpha << ", Cl =" << getCL() << ", Cd =" << getCD() << "VPath" << getVPathF() << "HPath" << getHPathF() << "Kp" << lift.Kp << "Kd" << lift.Kd << "dT" << lift.dt << "Aplha max =" << lift.max << ", min = " << lift.min;
    }

    if (m_eCurState == PerfState::initAC) {
        m_pCurAC->check(false);
        freeze();
        //pause();
        //zeroAcceleration();
        //zeroForces();
        //qDebug() << "After freezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Roll =" << getAngVel();
    }

    if (m_eCurState == PerfState::init) {
        freeze();
        //zeroAcceleration();
        //zeroForces();
        qDebug() << "After freezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Roll =" << getAngVel();
    }

    if (m_eCurState == PerfState::off) {
        //freeze();
        return 0.0f;
    }

    return -1.0f;




    //Testing
    static int count = 0;
    ++count;

    qDebug() << "After: Vel = " << getGLvel() << "Accel =" << getOpenGLacc() << "Count =" << count;

    if (count <= 8) {
        /*
        qDebug() << "sim/cockpit/weapons/guns_armed" << XPLMGetDataRefTypes(XPdata::ref("sim/cockpit/weapons/guns_armed")->getRef()); //int
        qDebug() << "sim/flightmodel/forces/fside_gear" << XPLMGetDataRefTypes(XPdata::ref("sim/flightmodel/forces/fside_gear")->getRef()); //float
        qDebug() << "sim/flightmodel/position/local_x" << XPLMGetDataRefTypes(XPdata::ref("sim/flightmodel/position/local_x")->getRef()); //double
        qDebug() << "sim/flightmodel/failures/prop_ice_per_engine" << XPLMGetDataRefTypes(XPdata::ref("sim/flightmodel/failures/prop_ice_per_engine")->getRef()); //float[8]
        qDebug() << "sim/flightmodel/engine/POINT_XYZ" << XPLMGetDataRefTypes(XPdata::ref("sim/flightmodel/engine/POINT_XYZ")->getRef()); //float[8][3]
        qDebug() << "sim/flightmodel/engine/ENGN_running" << XPLMGetDataRefTypes(XPdata::ref("sim/flightmodel/engine/ENGN_running")->getRef()); //int[8]
        qDebug() << "sim/aircraft/parts/acf_Rafl0" << XPLMGetDataRefTypes(XPdata::ref("sim/aircraft/parts/acf_Rafl0")->getRef()); //byte
        //qDebug() << "" << XPLMGetDataRefTypes(XPdata::ref("")->getRef());
        */
        freeze(true);
    }

    XPdata::inc();
    qDebug() << "Freezed: Vel = " << getGLvel() << "Accel =" << getOpenGLacc();

    if (count > 10) {
        count = 0;
        return 0.0f;
    }
    return -1.0f;
    //Testing end

    if (!m_bFltRunning)
        return 0.0f;

    if (m_pCurrent && checkConditions()) {
        if (processCurrent()) {
            m_cReady.append(m_pCurrent);
            m_pCurrent = nullptr;
            emit outData();
        }
    }

    initializeTestpoint(false);

    return -1.0f;

    //if (m_cQueue.size())
    //    return -1.0f;

    //setConditions(false);
    //m_bFltRunning = false;
    //return 0.0f;
}

float PerfPlugin::startFlightLoop(float inElapsedSinceLastCall,
                             float inElapsedTimeSinceLastFlightLoop,
                             int inCounter,
                             void *inRefcon)
{
    Q_UNUSED(inElapsedSinceLastCall);
    Q_UNUSED(inElapsedTimeSinceLastFlightLoop);
    Q_UNUSED(inCounter);
    Q_UNUSED(inRefcon);

    qDebug() << "Startloop";

    //m_pControl = new PoM(0.0f, 20.0f, 1.0f, 0.06f, 0.1f, 2.0f, -2.0f);

    XPLMScheduleFlightLoop(preLoopId, -1.0, true);
    return 0.0f;



    XPdata::inc();
    //getFlaps();
    //setFlaps(0.0f);
    //getFlaps();
    XPdata *test1 = XPdata::ref("sim/aircraft/engine/acf_num_engines");
    XPdata *test2 = XPdata::ref("sim/aircraft/engine/acf_spooltime_jet");
    XPdata *test3 = XPdata::ref("sim/aircraft/engine/acf_spooltime_prop");
    XPdata *test4 = XPdata::ref("sim/aircraft/engine/acf_spooltime_turbine");

    test2->set(0.1f);
    test3->set(0.1f);
    test4->set(0.1f);

    setThrottle(1.0f);

    qDebug() << "engines" << test1->getf() << "spooljet" << test2->getf() << "spoolprop" << test3->getf() << "spoolturbine" << test4->getf();

    return 0.0f;




    //m_cSavedpos = setPos(QVectorD3D(60.0+19.0/60+2.0/3600, 24.0+57.0/60+48.0/3600, 200));
    //m_cSavedpos = setPos(QVectorD3D(0.0+0.0/60+0.0/3600, -40.0+57.0/60+48.0/3600, 1));
    //m_cSavedpos = setPos(QVectorD3D(45.0+0.0/60+0.0/3600, 50.0+0.0/60+0.0/3600, 200));



    m_cSavedpos = setPos(QVectorD3D(45.0, 50.0, 200));
    setISAweather(true);

    static XPdata *mu = XPdata::ref("sim/physics/earth_mu");
    static XPdata *rad = XPdata::ref("sim/physics/earth_radius_m");

    qDebug() << "MU" << mu->getf() << "Rad" << rad->getf();
    //mu->set(3.986004418.0f);
    rad->set(6375416.324f);
    XPdata::inc();

    //Coordinate pos = getCoordinates();
    //qDebug() << "New pos =" << getPos().toVector3D() << "Coordinates lat" << pos.getLat() << "lon" << pos.getLon() << "elev" << pos.getElev() << "MU" << mu->getf() << "Rad" << rad->getf();
    //m_cSavedpos = getGLpos();
    //m_cTestpos = m_cSavedpos;
    XPLMScheduleFlightLoop(preLoopId, -1.0, true);
    XPLMScheduleFlightLoop(afterLoopId, -1.0, true); //Unset! This is for testing!
    return 0.0;




    XPdata::inc();

    if (m_pCurrent && checkConditions()) {
        if (processCurrent()) {
            m_cReady.append(m_pCurrent);
            m_pCurrent = nullptr;
            emit outData();
        }
    }

    if (!m_pCurrent && m_cQueue.size()) {
        m_pCurrent = m_cQueue.first();
        m_cQueue.removeFirst();
    }

    if (m_pCurrent) {
        setConditions();

        return -1.0f;
    }

    if (m_cQueue.size())
        return -1.0f;

    m_bFltRunning = false;
    return 0.0f;
}

void PerfPlugin::pause()
{
    static XPLMCommandRef pause = XPLMFindCommand("sim/operation/pause_toggle");

    Q_ASSERT(pause);

    //XPLMCommandKeyStroke(xplm_key_pause);

    //if (pause)
    XPLMCommandOnce(pause);
}

bool PerfPlugin::isPaused()
{
    static XPdata *paused = XPdata::ref("sim/time/paused");

    return paused->geti();
}

void PerfPlugin::initTestpoint(TestPoint *m_pCurrent) //Set gear flaps etc
{
    Q_UNUSED(m_pCurrent);
    //setGear(m_pCurrent->gear);
    //setFlaps(m_pCurrent->flaps);

    //setThrottle(m_pCurrent->throttle);
    //setPitch(m_pCurrent->pitchTrim);
    //setRoll(m_pCurrent->rollTrim);
    //setYaw(m_pCurrent->yawTrim);
    //setAttitude(m_pCurrent->alpha);
    //m_fAlpha = m_pCurrent->alpha;
    //setTestAtt();
    //setTAS(m_pCurrent->speed, false);
    //setTestAltitude(m_pCurrent->alt);
    //createAutoPilot();
    //qDebug() << "Roll =" << getOpenGLatt().toEulerAngles() << "Alpha"  << m_pCurrent->alpha;
}

void PerfPlugin::createAutoPilot()
{
    if (!m_pAutoPilot) {

        /*
        Filter<float> *QradKdFilter = new EMA(3.0f, 99.0f);
        Filter<float> *RradKdFilter = new EMA(3.0f, 99.0f);
        Filter<float> *PradKdFilter = new EMA(3.0f, 99.0f);
        Filter<float> *ThetaKdFilter = new EMA(3.0f, 99.0f);
        Filter<float> *ThrKdFilter = new EMA(3.0f, 99.0f);
        //PID<float> *pitchAlpha = new PoM(0.0f, 0.12f, 0.05f, 0.05f, 0.1f, 2.0f, -2.0f);
        //PID<float> *pitchAlpha = new PoM(0.0f, 1.0f, 1.0f, 0.06f, 0.1f, 2.0f, -2.0f);
        //PID<float> *pitchAlpha = new PoM(0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 2.0f, -2.0f);
        PID<float> *QradPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 2.0f, -2.0f);
        PID<float> *RradPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 2.0f, -2.0f);
        PID<float> *PradPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 2.0f, -2.0f);
        PID<float> *ThetaPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.2f * getMaxAlpha(), -getMaxAlpha());
        PID<float> *ThrPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.0f, 0.0f);
        //AutopilotAxis<float> *pitch = new AutopilotAxis(getAngVelQ, setPitch, QradPid);
        //AutopilotAxis<float> *yaw = new AutopilotAxis(getAngVelR, setYaw, RradPid);
        //AutopilotAxis<float> *roll = new AutopilotAxis(getAngVelP, setRoll, PradPid);
        //AutopilotAxis<float> *pitch = new AutopilotAxis(getAngAccQdot, setPitch, QradPid);
        //AutopilotAxis<float> *yaw = new AutopilotAxis(getAngAccRdot, setYaw, RradPid);
        //AutopilotAxis<float> *roll = new AutopilotAxis(getAngAccPdot, setRoll, PradPid);
        //AutopilotAxis<float> *vpath = new AutopilotAxis(getVPath, setTestAlpha, ThetaPid);
        //AutopilotAxis<float> *throttle = new AutopilotAxis(getTotalForceZ, setThrottle, ThrPid);
        //m_pAutoPilot = new Autopilot<float>();
        //AutopilotAxis<float> *pitchAPalpha = new AutopilotAxis(getAlpha, setPitch, pitchAlpha);
        //this->setTestAlpha
        //PerfPlugin::setTestAlpha

        m_pControl = ThetaPid; //The one can be controlled with debug

        QradPid->KdFilter = QradKdFilter;
        RradPid->KdFilter = RradKdFilter;
        PradPid->KdFilter = PradKdFilter;
        ThetaPid->KdFilter = ThetaKdFilter;
        ThrPid->KdFilter = ThrKdFilter;
        //pitchAPalpha->setScale(getMaxAlpha() * 2.0f);
        //pitch->setScale(5000);
        //yaw->setScale(5000);
        //roll->setScale(5000);
        //throttle->setScale(1);

        //m_pAutoPilot->add(pitch);
        //m_pAutoPilot->add(yaw);
        //m_pAutoPilot->add(roll);
        //m_pAutoPilot->add(vpath);
        //m_pAutoPilot->add(throttle);
        */
    }
}

void PerfPlugin::controlPara1(double v)
{
    m_bPreDump = v;
    return;

    if (!m_pControl)
        return;

    PID<float> *p = (PID<float> *)m_pControl;
    p->Kp = v;
}

void PerfPlugin::controlPara2(double v)
{
    m_bAfterDump = v;
    return;

    if (!m_pControl)
        return;

    PID<float> *p = (PID<float> *)m_pControl;
    p->Ki = v;
}

void PerfPlugin::controlPara3(double v)
{

    return;

    if (!m_pControl)
        return;

    PID<float> *p = (PID<float> *)m_pControl;
    p->Kd = v;
}

void PerfPlugin::controlPara4(double v)
{

    return;

    if (!m_pControl)
        return;

    PID<float> *p = (PID<float> *)m_pControl;
    p->dt = v;
}

void PerfPlugin::controlPara5(double v)
{

    return;

    if (!m_pControl)
        return;

    PID<float> *p = (PID<float> *)m_pControl;
    p->sp = v;
}

void PerfPlugin::controlButton()
{

    return;

    qDebug() << "Reset request...";
    reset = true;
}

void PerfPlugin::showMessage(QString msg)
{
    if (m_pPerfApp)
        m_pPerfApp->statusBar()->showMessage("Ready");
}


void PerfPlugin::setApLevelFlight()
{
    setBankHold(0);
    setVerticalSpeed(0.0f);
    setApModes(0x14); //0x4 (roll hold), 0x10 (V/S select), 0x20 (Alt Arm), 0x40 (FLCH), 0x4000 (Alt hold), 0x40000 (VNAV engaged)
    //setApModes(0x54);
    setAutopilot(true);
}

void PerfPlugin::setAutopilot(bool on)
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/autopilot_mode");

    if (on)
        c->set(2);
    else
        c->set(0);
}

void PerfPlugin::setApModes(int flags)
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/autopilot_state");

    c->set(flags);
}

int PerfPlugin::getApModes()
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/autopilot_state");

    return c->geti();
}

void PerfPlugin::setBankHold(int mode)
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/heading_roll_mode");

    c->set(mode);
}

void PerfPlugin::setVerticalSpeed(float fmp)
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/vertical_velocity");

    c->set(fmp);
}

void PerfPlugin::setAltSelector(float ft)
{
    static XPdata *c = XPdata::ref("sim/cockpit/autopilot/vertical_velocity");

    c->set(ft);
}

void PerfPlugin::initializeTestpoint(bool set) //Set testspeed etc
{
    if (set) {
        //setspeed
        //setattitude
        //setaltitude
    } else {
        freeze(true);
    }
}

void PerfPlugin::setConditions(bool set)
{
    //setAltitude(2000);
    saveVelAttAng(set);
    setTestPos(set);
    setISAweather(set);
    setControls(set);
    //m_cSavedpos = getGLpos();
    //m_cTestpos = m_cSavedpos;
}

void PerfPlugin::setTestPos(bool set)
{
    static bool notSet = true;
    static QVectorD3D orgPos(0.0, 0.0, 0.0);

    if (set) { //Set position
        if (notSet) {
            orgPos = getPos(); //Position lat, lon, elev
            qDebug() << "Pos saved" << orgPos.toVector3D();
        }

        setPos(QVectorD3D(45.0, 50.0, 1000.0));

        notSet = false;
    } else if (!set && !notSet) { //Reset position
        qDebug() << "Reset Pos" << orgPos.toVector3D();
        setPos(orgPos);

        notSet = true;
    }
}

void PerfPlugin::setTestAtt()
{
    setAttitude(m_fAlpha, 0.0f, 0.0f);
    //setAttitude(m_pCurrent->alpha, 0.0f, 0.0f);
}

void PerfPlugin::setTestAlpha(float f)
{
    //m_pCurrent->alpha = f;
    m_fAlpha = f;
}

void PerfPlugin::saveVelAttAng(bool save)
{
    static bool notSaved = true;
    static QQuaternion orgAtt(1.0f, 0.0f, 0.0f, 0.0f);
    static QVector3D orgVel(0.0f, 0.0f, 0.0f);
    static QVector3D orgAngVel(0.0f, 0.0f, 0.0f);

    if (save && notSaved) { //Save velocities
        //orgAtt = getOpenGLatt(); //Quaternion position
        orgAtt = getGLq(); //Quaternion position
        orgVel = getGLvel(); //Velocities
        orgAngVel = getAngVel(); // Prad, Qrad, Rrad rad/sec

        //setOpenGLvel(QVector3D(0.0f, 0.0f, 0.0f));
        //setAngvel(QVector3D(0.0f, 0.0f, 0.0f));
        notSaved = false;
    } else if (!save && !notSaved) { //Reset velocities
        setGLq(orgAtt); //Quaternion position
        setOpenGLvel(orgVel);
        setAngVel(orgAngVel);

        notSaved = true;
    }
}

void PerfPlugin::freezeAccNrot()
{
    setAngVel(m_cZeroV);
    setAngAcc(m_cZeroV);
    setOpenGLacc(m_cZeroV);
    zeroForces();
}

void PerfPlugin::freeze(bool set)
{
    Q_UNUSED(set);

    //static const QVector3D zero(0.0f, 0.0f, 0.0f);
    setOpenGLvel(m_cZeroV);
    setAngVel(m_cZeroV);
    setAngAcc(m_cZeroV);
    setOpenGLacc(m_cZeroV);
    zeroForces();

    return;

    return zeroAcceleration();
    return zeroForces();




    //setOpenGLvel(QVector3D());
    //setOpenGLacc(QVector3D());
    //static bool notSet = true;
    static const QHash<QByteArray, QVariant> hash =
    {{"sim/flightmodel/position/local_vx", 0.0f}, //Set linear velocity to 0.
     {"sim/flightmodel/position/local_vy", 0.0f},
     {"sim/flightmodel/position/local_vz", 0.0f},
     {"sim/flightmodel/position/Prad", 0.0f}, //Set angular velocity to 0.
     {"sim/flightmodel/position/Qrad", 0.0f},
     {"sim/flightmodel/position/Rrad", 0.0f},
     {"sim/flightmodel/forces/g_nrml", 0.0f}, //Override gravity
     {"sim/flightmodel/forces/g_axil", 0.0f},
     {"sim/flightmodel/forces/g_side", 0.0f},
     {"sim/flightmodel/forces/fside_total", 0.0f}, //Override aerodynamic forces
     {"sim/flightmodel/forces/fnrml_total", 0.0f},
     {"sim/flightmodel/forces/faxil_total", 0.0f}};
    /*
     {"sim/operation/override/override_forces", 1}, //Override physics model
     {"sim/flightmodel/position/local_ax", 0.0f}, //Set linear acceleration to 0.
     {"sim/flightmodel/position/local_ay", 0.0f},
     {"sim/flightmodel/position/local_az", 0.0f},
     {"sim/flightmodel/forces/L_total", 0.0f}, //Override aerodynamic moments
     {"sim/flightmodel/forces/M_total", 0.0f},
     {"sim/flightmodel/forces/N_total", 0.0f},
     */

    /*
    static QHash<QByteArray, QVariant> saved = hash;
    static QVector3D savedVel;
    static XPdata *override = XPdata::ref("sim/operation/override/override_forces");
    static XPdata *side = XPdata::ref("sim/flightmodel/forces/fside_total");
    static XPdata *nrml = XPdata::ref("sim/flightmodel/forces/fnrml_total");
    static XPdata *axil = XPdata::ref("sim/flightmodel/forces/faxil_total");
    static XPdata *g_nrml = XPdata::ref("sim/flightmodel/forces/g_nrml");
    static XPdata *g_axil = XPdata::ref("sim/flightmodel/forces/g_axil");
    static XPdata *g_side = XPdata::ref("sim/flightmodel/forces/g_side");
    static XPdata *alt = XPdata::ref("sim/flightmodel/misc/h_ind");
    static XPdata *sig = XPdata::ref("sim/weather/sigma");
    static XPdata *rho = XPdata::ref("sim/weather/rho");
    static XPdata *temp = XPdata::ref("sim/weather/temperature_ambient_c");
    static XPdata *seaT = XPdata::ref("sim/weather/temperature_sealevel_c");
    static XPdata *baro = XPdata::ref("sim/weather/barometer_sealevel_inhg");
    static XPdata *baroC = XPdata::ref("sim/cockpit/misc/barometer_setting");
    static XPdata *pres = XPdata::ref("sim/weather/barometer_current_inhg");
    static XPdata *presE = XPdata::ref("sim/physics/earth_pressure_p");
    static XPdata *rhoE = XPdata::ref("sim/physics/rho_sea_level");
    static XPdata *g = XPdata::ref("sim/physics/g_sealevel");
    static XPdata *gcur = XPdata::ref("sim/weather/gravity_mss");
    static XPdata *mu = XPdata::ref("sim/physics/earth_mu");
    static XPdata *rad = XPdata::ref("sim/physics/earth_radius_m");
    */

    /*
     * sim/weather/
     * temperature_ambient_c //The air temperature outside the aircraft (at altitude).
     * temperature_sealevel_c //The air temperature outside the aircraft (at altitude).
     * temperature_le_c //The air temperature at the leading edge of the wings in degrees C.
     * sigma //The atmospheric density as a ratio compared to sea level.
     * rho //The density of the air in kg/cubic meters.
     * barometer_sealevel_inhg //+- …. The barometric pressure at sea level.
     * barometer_current_inhg //This is the barometric pressure at the point the current flight is at.
     * gravity_mss //This is the acceleration of gravity for the current planet.
     *
     * sim/physics/
     * earth_pressure_p //average pressure at sea level, current planet
     * rho_sea_level //rho at sea level, current planet
     *
     * sim/cockpit2/gauges/actuators/
     * barometer_setting_in_hg_pilot
     *
     * sim/cockpit2/gauges/indicators/
     * altitude_ft_pilot
     *
     * sim/cockpit/misc/
     * barometer_setting
     *
     * sim/flightmodel/misc/
     * h_ind
     * h_ind2
     * h_ind_copilot
     * h_ind_copilot2
     * machno
     * Qstatic
     * */

    /*
    QVector3D gravity = QVector3D(g_axil->getf(), g_nrml->getf(), g_side->getf());
    //qDebug() << "FREEZE: Override =" << override->getf() << ", Side =" << side->getf() << ", Normal =" << nrml->getf() << ", Axil = " << axil->getf() << "OpenGLacc" << getOpenGLacc() << "OpenGLpos" << getGLpos().toVector3D() << "Lat/Lon/elev" << getPos().toVector3D() << "Altitude GL =" << getAltitude().toStr() << "Altitude =" << getAlt().toStr() << "AGL = " << getAgl().toStr() << "Indicated" << alt->getf() << "Sigma" << sig->getf() << "Density" << rho->getf() << "Baro at sea" << baro->getf() << "Baro set" << baroC->getf() << "Pres at alt" << pres->getf() << "Pres Earth" << presE->getf() << "Density Earth" << rhoE->getf() << "Temp" << temp->getf() << "MSL Temp" << seaT->getf() << "Gravity = " << gravity << gravity.length() << g->getf() << gcur->getf() << "MU" << mu->getf() << "Rad" << rad->getf();

    //side->set(0.0f);
    //nrml->set(0.0f);
    //axil->set(0.0f);

    QVectorD3D pos = getGLpos();
    pos.setY(pos.y() - 0.1f);
    //setGLpos(pos);
    XPdata::inc();
    gravity = QVector3D(g_axil->getf(), g_nrml->getf(), g_side->getf());
    //qDebug() << "FREEZE: Override =" << override->getf() << ", Side =" << side->getf() << ", Normal =" << nrml->getf() << ", Axil = " << axil->getf() << "OpenGLacc" << getOpenGLacc() << "OpenGLpos" << getGLpos().toVector3D() << "Lat/Lon/elev" << getPos().toVector3D() << "Altitude GL =" << getAltitude().toStr() << "Altitude =" << getAlt().toStr() << "AGL = " << getAgl().toStr() << "Indicated" << alt->getf() << "Sigma" << sig->getf() << "Density" << rho->getf() << "Baro at sea" << baro->getf() << "Baro set" << baroC->getf() << "Pres at alt" << pres->getf() << "Pres Earth" << presE->getf() << "Density Earth" << rhoE->getf() << "Temp" << temp->getf() << "MSL Temp" << seaT->getf() << "Gravity = " << gravity << gravity.length() << g->getf() << gcur->getf() << "MU" << mu->getf() << "Rad" << rad->getf();

    //override->set(1);
    //side->set(0.0f);
    //nrml->set(0.0f);
    //axil->set(0.0f);
    pos.setY(pos.y() - 0.1f);
    //setGLpos(pos);
    //baro->set(29.85f);

    XPdata::inc();
    gravity = QVector3D(g_axil->getf(), g_nrml->getf(), g_side->getf());
    //qDebug() << "FREEZE: Override =" << override->getf() << ", Side =" << side->getf() << ", Normal =" << nrml->getf() << ", Axil = " << axil->getf() << "OpenGLacc" << getOpenGLacc() << "OpenGLpos" << getGLpos().toVector3D() << "Lat/Lon/elev" << getPos().toVector3D() << "Altitude GL =" << getAltitude().toStr() << "Altitude =" << getAlt().toStr() << "AGL = " << getAgl().toStr() << "Indicated" << alt->getf() << "Sigma" << sig->getf() << "Density" << rho->getf() << "Baro at sea" << baro->getf() << "Baro set" << baroC->getf() << "Pres at alt" << pres->getf() << "Pres Earth" << presE->getf() << "Density Earth" << rhoE->getf() << "Temp" << temp->getf() << "MSL Temp" << seaT->getf() << "Gravity = " << gravity << gravity.length() << g->getf() << gcur->getf() << "MU" << mu->getf() << "Rad" << rad->getf();

    if (set && notSet) {
        savedVel = getGLvel();
        XPdata::get(&saved);
        //override->set(1);
        //XPdata::set(&hash);
        //setOpenGLvel(QVector3D());
        //setOpenGLacc(QVector3D());
        //setGLpos(m_cTestpos);

        notSet = false;
        qDebug() << "Setting freeze hash" << hash;
    } else if (!set && !notSet) {
        XPdata::set(&saved);
        //override->set(0);
        //setOpenGLvel(savedVel);
        //setGLpos(m_cTestpos);

        notSet = true;
        qDebug() << "Setting saved" << saved;
    }
    gravity = QVector3D(g_axil->getf(), g_nrml->getf(), g_side->getf());
    //qDebug() << "FREEZE AFTER GLpos: Override =" << override->getf() << ", Side =" << side->getf() << ", Normal =" << nrml->getf() << ", Axil = " << axil->getf() << "OpenGLacc" << getOpenGLacc() << "OpenGLpos" << getGLpos().toVector3D() << "Lat/Lon/elev" << getPos().toVector3D() << "Altitude GL =" << getAltitude().toStr() << "Altitude =" << getAlt().toStr() << "AGL = " << getAgl().toStr() << "Indicated" << alt->getf() << "Sigma" << sig->getf() << "Density" << rho->getf() << "Baro at sea" << baro->getf() << "Baro set" << baroC->getf() << "Pres at alt" << pres->getf() << "Pres Earth" << presE->getf() << "Density Earth" << rhoE->getf() << "Temp" << temp->getf() << "MSL Temp" << seaT->getf() << "Gravity = " << gravity << gravity.length() << g->getf() << gcur->getf() << "MU" << mu->getf() << "Rad" << rad->getf();
    */
}

void PerfPlugin::zeroForces()
{
    /*
    static const QHash<QByteArray, QVariant> hash =
    {{"sim/flightmodel/forces/g_nrml", 0.0f}, //Override g-forces
     {"sim/flightmodel/forces/g_axil", 0.0f},
     {"sim/flightmodel/forces/g_side", 0.0f},
     {"sim/flightmodel/forces/L_total", 0.0f}, //Roll pitch yaw moment
     {"sim/flightmodel/forces/M_total", 0.0f},
     {"sim/flightmodel/forces/N_total", 0.0f},
     {"sim/flightmodel/forces/fside_total", 0.0f}, //Override aerodynamic forces
     {"sim/flightmodel/forces/fnrml_total", 0.0f},
     {"sim/flightmodel/forces/faxil_total", 0.0f}};

    XPdata::set(&hash);
    */
    setGForces(m_cZeroV);
    setTotalMoments(m_cZeroV);
    setTotalForces(m_cZeroV);
}



void PerfPlugin::zeroAcceleration()
{
    setOpenGLacc(m_cZeroV);
}


bool PerfPlugin::checkConditions()
{
    //setAltitude(3000);
    //getCoordinates();
    //qDebug() << "SetAlpha";
    //setAlpha(-10.0f);
    //qDebug() << "Read rotation!!!";
    //qDebug() << "ATT qua" << getOpenGLatt() << getOpenGLatt().toEulerAngles();
    //qDebug() << "ATT real" << getRealAtt() << getRealAtt().toEulerAngles();
    //qDebug() << "ATT openGL" << getOpenGLattDeg() << getOpenGLattDeg().toEulerAngles();

    //qDebug() << "Set rotation!!!" << m_pCurrent->pitchTrim << m_pCurrent->yawTrim << m_pCurrent->rollTrim;
    //QQuaternion q = QQuaternion::fromEulerAngles(m_pCurrent->pitch, m_pCurrent->yaw, m_pCurrent->rollTrim);
    //qDebug() << "Quaternion =" << q << q.toEulerAngles();
    //setGLq(QQuaternion::fromEulerAngles(m_pCurrent->pitch, m_pCurrent->yaw, m_pCurrent->roll));
    //setGLq(q);

    //setAlpha(m_pCurrent->alpha);

    //qDebug() << "ATT qua" << getOpenGLatt() << getOpenGLatt().toEulerAngles();
    //qDebug() << "ATT real" << getRealAtt() << getRealAtt().toEulerAngles();
    //qDebug() << "ATT openGL" << getOpenGLattDeg() << getOpenGLattDeg().toEulerAngles();
    //qDebug() << "Done rotation!!!";

    return true;
}

Atmosphere PerfPlugin::getEnvironment()
{
    Atmosphere state;
    state.pressure = getPressure();
    state.rho = getRho();
    state.c = getC();
    state.q = getQ();
    return state;
}

float PerfPlugin::getRho()
{
    //static XPdata *p = XPdata::ref("sim/physics/rho_sea_level"); // 1.225
    static XPdata *p = XPdata::ref("sim/weather/rho"); //at current level
    return p->getf();
}

float PerfPlugin::getSigma()
{
    static XPdata *p = XPdata::ref("sim/weather/sigma"); //at current level
    return p->getf();
}

float PerfPlugin::getPressure()
{
    //static XPdata *p = XPdata::ref("sim/weather/barometer_sealevel_inhg"); // 29.92 inHg / 101325.0 Pa
    static XPdata *p = XPdata::ref("sim/weather/barometer_current_inhg"); //at current level
    return p->getf() * PHY_INHG_PA;
}

float PerfPlugin::getTat()
{
    static XPdata *p = XPdata::ref("sim/weather/temperature_ambient_c"); //at current level
    return p->getf() - PHY_K;
}

float PerfPlugin::getSat()
{
    static XPdata *p = XPdata::ref("sim/weather/temperature_le_c"); //at current level stagnation
    return p->getf() - PHY_K;
}

float PerfPlugin::getC()
{
    static XPdata *p = XPdata::ref("sim/weather/speed_sound_ms"); //at current level
    return p->getf();
}

float PerfPlugin::getQ()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/misc/Qstatic"); //dynamic pressure (no compressibility) Pa  q = 1/2 RHO Vt^2
    return p->getf();
}

float PerfPlugin::getG() //G at current level
{
    //static XPdata *p = XPdata::ref("sim/physics/g_sealevel"); //gravity at current level (Bug)
    static XPdata *p = XPdata::ref("sim/weather/gravity_mss"); //gravity at current level
    return p->getf();
}

void PerfPlugin::setAltitude(Length alt)
{
    //Coordinate coord = getCoordinates();
    //setCoordinates(coord.setElev(alt.value));

    static XPdata *y = XPdata::ref("sim/flightmodel/position/local_y");
    qDebug() << "Get altitude" << y->get() << getGLpos().toVector3D();
    y->set(alt.raw());
    XPdata::inc();
    qDebug() << "Set altitude" << y->get() << getGLpos().toVector3D();
}

void PerfPlugin::setTestAltitude(Length alt)
{
    //static Length altFt(Unit("ft", 0.3048));
    static double delta;
    static double agl;

    qDebug() << "setTestAltitude" << m_cTestGLpos.y();

    agl = getAglm();
    delta = alt.raw() - getAltm();
    delta = (agl + delta - m_dMinAlt) < 0 ? (-agl + m_dMinAlt) : delta; //minimum altitude

    //altFt = alt;
    //setAltSelector(altFt.val());

    m_cTestGLpos.setY(m_cTestGLpos.y() + delta);
    qDebug() << "setTestAltitude: alt = " << alt.toStr() << alt.raw() << "AGL =" << agl << "delta =" << delta << "GL y =" << m_cTestGLpos.y();
}

void PerfPlugin::setTestPosition()
{
    setGLpos(m_cTestGLpos);
}

/*
void PerfPlugin::setCoordinates(Coordinate coord)
{
    XPdata *x = XPdata::ref("sim/flightmodel/position/local_x");
    XPdata *y = XPdata::ref("sim/flightmodel/position/local_y");
    XPdata *z = XPdata::ref("sim/flightmodel/position/local_z");
    XPdata *lat = XPdata::ref("sim/flightmodel/position/latitude");
    XPdata *lon = XPdata::ref("sim/flightmodel/position/longitude");

    double lx, ly, lz;
    XPLMWorldToLocal(coord.getLat(), coord.getLon(), coord.getElev(), &lx, &ly, &lz);

    x->set(lx);
    y->set(ly);
    z->set(lz);
    lat->set(coord.getLat());
    lon->set(coord.getLon());
}

Length PerfPlugin::getAltitude()
{
    XPdata *z = XPdata::ref("sim/flightmodel/position/local_y");

    return Length(z->get().toDouble());
}

Coordinate PerfPlugin::getCoordinates()
{
    XPdata *x = XPdata::ref("sim/flightmodel/position/local_x");
    XPdata *y = XPdata::ref("sim/flightmodel/position/local_y");
    XPdata *z = XPdata::ref("sim/flightmodel/position/local_z");
    double lat, lon, elev;

    XPLMLocalToWorld(x->getd(), y->getd(), z->getd(), &lat, &lon, &elev);
    qDebug() << x->getd() <<  y->getd() << z->getd() << lat << lon << elev;

    return Coordinate(lat, lon, elev);
}
*/

QVectorD3D PerfPlugin::setPos(QVectorD3D v)
{
    //XPdata *x = XPdata::ref("sim/flightmodel/position/local_x");
    //XPdata *y = XPdata::ref("sim/flightmodel/position/local_y");
    //XPdata *z = XPdata::ref("sim/flightmodel/position/local_z");
    //XPdata *lat = XPdata::ref("sim/flightmodel/position/latitude");
    //XPdata *lon = XPdata::ref("sim/flightmodel/position/longitude");

    double lx, ly, lz;
    XPLMWorldToLocal(v.x(), v.y(), v.z(), &lx, &ly, &lz);

    QVectorD3D pos(lx, ly, lz);
    setGLpos(pos);
    return pos;

    //lat->set(v.x());
    //lon->set(v.y());
    //lat->set(v.x());
    //lon->set(v.y());
    //x->set(lx);
    //y->set(ly);
    //z->set(lz);
    //lat->set(coord.getLat());
    //lon->set(coord.getLon());

    //return setGLpos(pos);
}

double PerfPlugin::getLat()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/latitude");
    return p->getd();
}

double PerfPlugin::getLon()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/longitude");
    return p->getd();
}

double PerfPlugin::getGLposX()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_x");
    return p->getd();
}

double PerfPlugin::getGLposY()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_y");
    return p->getd();
}

double PerfPlugin::getGLposZ()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_z");
    return p->getd();
}

void PerfPlugin::setGLpos(QVectorD3D v, int mask)
//void PerfPlugin::setGLpos(QVectorD3D v, Axis mask)
{
    if (mask & X)
        setGLposX(v.x());
    if (mask & Y)
        setGLposY(v.y());
    if (mask & Z)
        setGLposZ(v.z());
    //qDebug() << "OpenGL pos set," << v.toVector3D() << (int)mask << (int)X << (int)Y << (int)Z << ((mask & X) ? "true" : "false") << ((mask & Y) ? "true" : "false") << ((mask & Z) ? "true" : "false");
    //qDebug() << "OpenGL pos set," << v.toVector3D() << "mask" << mask;
}

void PerfPlugin::setGLposX(double d)
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_x");
    p->set(d);
}

void PerfPlugin::setGLposY(double d)
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_y");
    p->set(d);
}

void PerfPlugin::setGLposZ(double d)
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_z");
    p->set(d);
}

void PerfPlugin::setGLq(QQuaternion q)
{
//    XPdata *pitch = XPdata::ref("sim/flightmodel/position/theta");
//    XPdata *roll = XPdata::ref("sim/flightmodel/position/phi");
//    XPdata *heading = XPdata::ref("sim/flightmodel/position/psi");
    XPdata *XPq = XPdata::ref("sim/flightmodel/position/q"); // { 1, i, j, k}; i=roll=-z, j=pitch=x, k=heading=-y
    //QQuaternion roll = z, pitch=x, yaw=y

//    QList<float> buffer = XPq->getfList();

    QList<float> buffer(4);
    //qDebug() << "List size" << buffer.size();
    buffer[0] = q.scalar();
    buffer[1] = q.z();
    buffer[2] = q.x();
    buffer[3] = q.y();

    //qDebug() << "setGLq" << q.toEulerAngles() << buffer << getGLq() << getGLq().toEulerAngles();

    /*
    float pitch, roll, yaw;
    q.getEulerAngles(&pitch, &yaw, &roll);
    QQuaternion a = QQuaternion::fromEulerAngles(pitch, yaw, roll);
    QQuaternion b = QQuaternion::fromEulerAngles(QVector3D(pitch, yaw, roll));
    */
    //qDebug() << "SET GL:" << q << buffer << q.toEulerAngles() << "Pitch =" << pitch << "Roll =" << roll << "Yaw =" << yaw << a.toEulerAngles() << b.toEulerAngles();

    XPq->set(buffer);

    //qDebug() << "setGLq" << q.toEulerAngles() << buffer << getGLq() << getGLq().toEulerAngles();
    //return q;
}


//QQuaternion PerfPlugin::getOpenGLatt()
QQuaternion PerfPlugin::getGLq()
{
    //qDebug() << "GetOpenGLatt";
    QQuaternion q;
    XPdata *XPq = XPdata::ref("sim/flightmodel/position/q"); // { 1, k, j, i}; i=roll=-z, j=pitch=x, k=heading=-y

    QList<float> buffer = XPq->getfList();
    //qDebug() << "List size" << buffer.size();
    q.setScalar(buffer.at(0));
    q.setZ(buffer.at(1));
    q.setX(buffer.at(2));
    q.setY(buffer.at(3));

    /*)
    float pitch, roll, yaw;
    q.getEulerAngles(&pitch, &yaw, &roll);
    qDebug() << "GET GL:" << q << buffer << q.toEulerAngles() << "Pitch =" << pitch << "Roll =" << roll << "Yaw =" << yaw;
    */
    return q;
}


/*
QQuaternion PerfPlugin::getOpenGLattDeg()
{
    //qDebug() << "GetOpenGLattDeg";
    XPdata *pitch = XPdata::ref("sim/flightmodel/position/theta");
    XPdata *roll = XPdata::ref("sim/flightmodel/position/phi");
    XPdata *heading = XPdata::ref("sim/flightmodel/position/psi");

    //qDebug() << "GetOpenGLattDeg:" << "OpenGL Pitch, roll, heading" << pitch->getf() << roll->getf() << heading->getf();

    QQuaternion cur = QQuaternion::fromEulerAngles(pitch->getf(), heading->getf(), roll->getf());

    return cur;
}
*/

inline void PerfPlugin::setAttitude(float pitch, float heading, float roll)
{
    setGLq(QQuaternion::fromEulerAngles(pitch, heading, roll));
}


void PerfPlugin::setRealAtt(QQuaternion q)
{
    return setGLq(getTrueAtt() * q);
    //return setGLq(getRealAtt() * q);
}

/*
QQuaternion PerfPlugin::getRealAtt()
{
    static XPdata *pitch = XPdata::ref("sim/flightmodel/position/true_theta");
    static XPdata *roll = XPdata::ref("sim/flightmodel/position/true_phi");
    static XPdata *heading = XPdata::ref("sim/flightmodel/position/true_psi");

    //qDebug() << "Pitch, roll, heading" << pitch->getf() << roll->getf() << heading->getf();

    QQuaternion cur = QQuaternion::fromEulerAngles(pitch->getf(), heading->getf(), roll->getf());

    return cur;
}
*/

QVector3D PerfPlugin::setOpenGLvel(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/local_vx");
    static XPdata *y = XPdata::ref("sim/flightmodel/position/local_vy"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/position/local_vz");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());

    return v;
}

QVector3D PerfPlugin::setOpenGLacc(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/local_ax");
    static XPdata *y = XPdata::ref("sim/flightmodel/position/local_ay"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/position/local_az");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());

    return v;
}

void PerfPlugin::setAeroForces(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_aero");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_aero"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_aero");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

void PerfPlugin::setEngineForces(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_prop");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_prop"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_prop");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

void PerfPlugin::setGForces(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/g_side");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/g_nrml"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/g_axil");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

void PerfPlugin::setTotalForces(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_total");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_total"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_total");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

float PerfPlugin::getGLtheta() //Pitch
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/theta");
    return p->getf();
}
float PerfPlugin::getGLphi() //Roll
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/phi");
    return p->getf();
}
float PerfPlugin::getGLpsi() //Heading
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/psi");
    return p->getf();
}

float PerfPlugin::getTrueTheta() //Pitch
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/true_theta");
    return p->getf();
}
float PerfPlugin::getTruePhi() //Roll
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/true_phi");
    return p->getf();
}
float PerfPlugin::getTruePsi() //Heading
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/true_psi");
    return p->getf();
}

float PerfPlugin::getAeroForceX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_aero");
    return x->getf();
}

float PerfPlugin::getAeroForceY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_aero"); //UP!
    return y->getf();
}

float PerfPlugin::getAeroForceZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_aero");
    return z->getf();
}

float PerfPlugin::getEngineForceX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_prop");
    return x->getf();
}

float PerfPlugin::getEngineForceY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_prop"); //UP!
    return y->getf();
}

float PerfPlugin::getEngineForceZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_prop");
    return z->getf();
}

float PerfPlugin::getGearForceX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_gear");
    return x->getf();
}

float PerfPlugin::getGearForceY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_gear"); //UP!
    return y->getf();
}

float PerfPlugin::getGearForceZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_gear");
    return z->getf();
}

float PerfPlugin::getGforceTotal()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/misc/g_total");
    return p->getf();
}

float PerfPlugin::getGforceX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/g_side");
    return x->getf();
}

float PerfPlugin::getGforceY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/g_nrml"); //UP!
    return y->getf();
}

float PerfPlugin::getGforceZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/g_axil");
    return z->getf();
}

float PerfPlugin::getTotalForceX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/fside_total");
    return x->getf();
}
float PerfPlugin::getTotalForceY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/fnrml_total"); //UP!
    return y->getf();
}
float PerfPlugin::getTotalForceZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/faxil_total");
    return z->getf();
}

float PerfPlugin::getFlightPathForcesSide()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/forces/side_path_axis");
    return p->getf();
}

float PerfPlugin::getFlightPathForcesLift()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/forces/lift_path_axis");
    return p->getf();
}

float PerfPlugin::getFlightPathForcesDrag()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/forces/drag_path_axis");
    return p->getf();
}

float PerfPlugin::getAeroMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_aero"); //pitch
    return x->getf();
}

float PerfPlugin::getAeroMomentN()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_aero"); //yaw
    return y->getf();
}

float PerfPlugin::getAeroMomentL()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_aero"); //roll
    return z->getf();
}

void PerfPlugin::setAeroMoments(QVector3D v) //Roll = L = z, Pitch = M = x, Yaw = N = y
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_aero");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_aero"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_aero");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

float PerfPlugin::getEngineMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_prop"); //pitch
    return x->getf();
}

float PerfPlugin::getEngineMomentN()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_prop"); //yaw
    return y->getf();
}

float PerfPlugin::getEngineMomentL()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_prop"); //roll
    return z->getf();
}

void PerfPlugin::setEngineMoments(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_prop");
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_prop"); //UP!
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_prop");

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

float PerfPlugin::getGearMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_gear"); //pitch
    return x->getf();
}

float PerfPlugin::getGearMomentN()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_gear"); //yaw
    return y->getf();
}

float PerfPlugin::getGearMomentL()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_gear"); //roll
    return z->getf();
}

float PerfPlugin::getMassMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_mass"); //pitch
    return x->getf();
}

float PerfPlugin::getMassMomentN()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_mass"); //yaw
    return y->getf();
}

float PerfPlugin::getMassMomentL()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_mass"); //roll
    return z->getf();
}

float PerfPlugin::getTotalMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_total"); //pitch
    return x->getf();
}

float PerfPlugin::getTotalMomentN()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/N_total"); //yaw
    return x->getf();
}

float PerfPlugin::getTotalMomentL()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/L_total"); //roll
    return x->getf();
}

void PerfPlugin::setTotalMoments(QVector3D v)
{
    static XPdata *x = XPdata::ref("sim/flightmodel/forces/M_total"); //pitch
    static XPdata *y = XPdata::ref("sim/flightmodel/forces/N_total"); //yaw
    static XPdata *z = XPdata::ref("sim/flightmodel/forces/L_total"); //roll

    x->set(v.x());
    y->set(v.y());
    z->set(v.z());
}

float PerfPlugin::getPosMomentM()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/M"); //pitch
    return x->getf();
}

float PerfPlugin::getPosMomentN()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/N"); //yaw
    return x->getf();
}

float PerfPlugin::getPosMomentL()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/L"); //roll
    return x->getf();
}

QQuaternion PerfPlugin::setAlpha(QQuaternion q)
{
    QVector3D v = q.toEulerAngles();

    //float delta = setAlpha(v.y());
    setAlpha(v.y());

    return q;
}

float PerfPlugin::setAlpha(float a)
{
    //QVector3D relv = getOpenGLatt() * getGLvel();
    //angle between x-plane a relv
    //angle between y-plane a relv
    //quaternion
    float delta = a - getAlpha();

    QQuaternion diff = QQuaternion::fromEulerAngles(delta, 0, 0);
    qDebug() << "multiply";
    //setGLq(diff * getOpenGLatt());
    //setGLq(getOpenGLatt() * diff);
    setGLq(getGLq() * diff);
    qDebug() << "multiply done";

    return a;
}

QQuaternion PerfPlugin::setBeta(QQuaternion q)
{
    QVector3D v = q.toEulerAngles();

    //float delta = setAlpha(v.y());
    setAlpha(v.z());

    return q;
}

float PerfPlugin::setBeta(float b)
{
    float delta = b - getAlpha();

    QQuaternion diff = QQuaternion::fromEulerAngles(0, delta, 0);
    //setGLq(diff * getOpenGLatt());
    setGLq(diff * getGLq());

    return b;

}

float PerfPlugin::getMaxAlpha()
{
    static XPdata *alpha = XPdata::ref("sim/aircraft/overflow/acf_stall_warn_alpha");
    return alpha->getf();
}

float PerfPlugin::getAlpha()
{
    static XPdata *alpha = XPdata::ref("sim/flightmodel/position/alpha");
    return alpha->getf();
}

float PerfPlugin::getBeta()
{
    XPdata *beta = XPdata::ref("sim/flightmodel/position/beta");
    return beta->getf();
}

//QQuaternion PerfPlugin::setVPath(QQuaternion q);
//QQuaternion PerfPlugin::setHPath(QQuaternion q);

float PerfPlugin::getVPath()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/vpath");
    return v->getf();
}

float PerfPlugin::getHPath()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/hpath");
    return v->getf();
}

//QVector3D PerfPlugin::setMom(QVector3D v)

/*
QVector3D PerfPlugin::getMom()
{
    static XPdata *M = XPdata::ref("sim/flightmodel/position/M");
    static XPdata *N = XPdata::ref("sim/flightmodel/position/N");
    static XPdata *L = XPdata::ref("sim/flightmodel/position/L");
    QVector3D v;

    v.setX(M->getf());
    v.setY(N->getf());
    v.setZ(L->getf());

    return v;
}
*/

QVector3D PerfPlugin::setAngVel(QVector3D v)
{
    static XPdata *P = XPdata::ref("sim/flightmodel/position/Prad"); //Roll = Z
    static XPdata *Q = XPdata::ref("sim/flightmodel/position/Qrad"); //Pitch = X
    static XPdata *R = XPdata::ref("sim/flightmodel/position/Rrad"); //Yaw = Y

    P->set(v.z());
    Q->set(v.x());
    R->set(v.y());

    return v;
}

QVector3D PerfPlugin::setAngAcc(QVector3D v)
{
    static XPdata *P = XPdata::ref("sim/flightmodel/position/P_dot"); //Roll = Z
    static XPdata *Q = XPdata::ref("sim/flightmodel/position/Q_dot"); //Pitch = X
    static XPdata *R = XPdata::ref("sim/flightmodel/position/R_dot"); //Yaw = Y

    P->set(v.z());
    Q->set(v.x());
    R->set(v.y());

    return v;
}

float PerfPlugin::getGLvelX()
{
    static XPdata *x = XPdata::ref("sim/flightmodel/position/local_vx");
    return x->getf();
}
float PerfPlugin::getGLvelY()
{
    static XPdata *y = XPdata::ref("sim/flightmodel/position/local_vy");
    return y->getf();
}
float PerfPlugin::getGLvelZ()
{
    static XPdata *z = XPdata::ref("sim/flightmodel/position/local_vz");
    return z->getf();
}

float PerfPlugin::getAngVelQ() //Q = Pitch = X
{
    static XPdata *q = XPdata::ref("sim/flightmodel/position/Qrad");
    return q->getf();
}

float PerfPlugin::getAngVelR() //R = Yaw = Y
{
    static XPdata *r = XPdata::ref("sim/flightmodel/position/Rrad");
    return r->getf();
}

float PerfPlugin::getAngVelP() //P = Roll = Z
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/Prad");
    return p->getf();
}

float PerfPlugin::getAngVelDegQ() //Q = Pitch = X
{
    static XPdata *q = XPdata::ref("sim/flightmodel/position/Q");
    return q->getf();
}
float PerfPlugin::getAngVelDegR() //R = Yaw = Y
{
    static XPdata *r = XPdata::ref("sim/flightmodel/position/R");
    return r->getf();
}
float PerfPlugin::getAngVelDegP() //P = Roll = Z
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/P");
    return p->getf();
}

QVector3D PerfPlugin::getOpenGLacc()
{
    QVector3D v;
    //static XPdata *x = XPdata::ref("sim/flightmodel/position/local_ax");
    //static XPdata *y = XPdata::ref("sim/flightmodel/position/local_ay"); //UP!
    //static XPdata *z = XPdata::ref("sim/flightmodel/position/local_az");

    //v.setX(x->getf());
    //v.setY(y->getf());
    //v.setZ(z->getf());

    v.setX(getOpenGLaccX());
    v.setY(getOpenGLaccY());
    v.setZ(getOpenGLaccZ());

    return v;
}

float PerfPlugin::getOpenGLaccX()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_ax");
    return p->getf();
}

float PerfPlugin::getOpenGLaccY()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_ay"); //UP!
    return p->getf();
}

float PerfPlugin::getOpenGLaccZ()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/local_az");
    return p->getf();
}

float PerfPlugin::getTestDeceleration()
{
    return -getTotalForceZ();
    //return -getOpenGLaccZ();
}


float PerfPlugin::getAngAccQdot() //Q = Pitch = X
{
    static XPdata *q = XPdata::ref("sim/flightmodel/position/Q_dot");
    return q->getf();
}

float PerfPlugin::getAngAccRdot() //R = Yaw = Y
{
    static XPdata *r = XPdata::ref("sim/flightmodel/position/R_dot");
    return r->getf();
}

float PerfPlugin::getAngAccPdot() //P = Roll = Z
{
    static XPdata *p = XPdata::ref("sim/flightmodel/position/P_dot");
    return p->getf();
}

Quantity PerfPlugin::setIAS(Quantity q)
{
    static XPdata *ias = XPdata::ref("sim/flightmodel/position/indicated_airspeed"); //kias

    Quantity ias_kt(Unit("kt", 1.852f/3.6f));
    ias_kt = q;

    ias->set(ias_kt.val());

    return ias_kt;
}

float PerfPlugin::getIASkt()
{
    //static XPdata *ias = XPdata::ref("sim/flightmodel/position/indicated_airspeed"); //kias
    static XPdata *ias = XPdata::ref("sim/flightmodel/position/indicated_airspeed2"); //kias
    return ias->getf();
}

float PerfPlugin::getTASms()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/true_airspeed"); // m/s
    return v->getf();
}
float PerfPlugin::getGSms()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/groundspeed"); // m/s
    return v->getf();
}

float PerfPlugin::getMachNo()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/misc/machno"); // ratio
    return v->getf();
}


Quantity PerfPlugin::getIAS(Unit u)
{
    Quantity ias_kt(Unit("kt", 1.852/3.6));
    ias_kt.set(getIASkt());

    return ias_kt.setUnit(u);
}

Quantity PerfPlugin::getCAS(Unit u)
{
    return Aerodynamics::tasToCas(getTAS(), getEnvironment());
}

Quantity PerfPlugin::getTAS(Unit u)
{
    Quantity tas_ms(Unit("m/s", 1.0));
    tas_ms.set(getTASms());

    return tas_ms.setUnit(u);
}

Quantity PerfPlugin::getGS(Unit u)
{
    Quantity gs_ms(Unit("m/s", 1.0));
    gs_ms.set(getTASms());

    return gs_ms.setUnit(u);
}

/*
Quantity PerfPlugin::setGS(Quantity q)
{
    XPdata *v = XPdata::ref("sim/flightmodel/position/groundspeed"); //kias

    v->set(q.raw());

    return q;
}
*/

//Quantity PerfPlugin::setAlt(Quantity q);
double PerfPlugin::getAltm()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/elevation"); //m
    return v->getd();
}

//Quantity PerfPlugin::setAgl(Quantity q);
float PerfPlugin::getAglm()
{
    static XPdata *v = XPdata::ref("sim/flightmodel/position/y_agl"); //m
    return v->getf();
}

float PerfPlugin::getCL()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/misc/cl_overall"); //Cl

    return p->get().toFloat();
}

float PerfPlugin::getCD()
{
    static XPdata *p = XPdata::ref("sim/flightmodel/misc/cd_overall"); //Cd

    return p->get().toFloat();
}

void PerfPlugin::dumpData(QString title)
{
    //static const int titleW = 10, w = 10;
    static int w = 22;
    static PID<float> *p = nullptr;
    p = (PID<float> *)m_pControl;
    if (p)
        w = p->Kp;

    qDebug() << "Dump: --------------------------------------------" << title << "-------------------------------------";


    if (m_bDumpDebug) {
        QVectorD3D v = getGLpos();
        qDebug() << QString("Dump PosGL:\tx = %1m\ty = %2m\tz = %3m\tALT = %4m").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(getAltm(), w).toUtf8().constData();
        v = m_cTestGLpos;
        qDebug() << QString("Dump TestPos:\tx = %1m\ty = %2m\tz = %3m\tAGL = %4m").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(getAglm(), w).toUtf8().constData();
        v = getGLq().toEulerAngles();
        qDebug() << QString("Dump GL Q:\tTheta = %1deg\tPhi = %2deg\tPsi = %3deg").arg(v.x(), w).arg(v.z(), w).arg(v.y(), w).toUtf8().constData();
        qDebug() << QString("Dump True Att:\tTheta = %1deg\tPhi = %2deg\tPsi = %3deg").arg(getTrueTheta(), w).arg(getTruePhi(), w).arg(getTruePsi(), w).toUtf8().constData();
        qDebug() << QString("Dump Att:\tAlpha = %1deg\tBeta = %2deg\tVpath = %3deg\tHpath = %4").arg(getAlpha(), w).arg(getBeta(), w).arg(getVPath(), w).arg(getHPath(), w).toUtf8().constData();
        v = getGLvel();
        qDebug() << QString("Dump Vel:\tx = %1m/s\ty = %2m/s\tz = %3m/s\tLen = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(v.length(), w).toUtf8().constData();
        v = getAngVel();
        qDebug() << QString("Dump Ang Vel:\tQ = %1r/s\tR = %2r/s\tP = %3r/s\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getOpenGLacc();
        qDebug() << QString("Dump Acc:\tx = %1m/s2\ty = %2m/s2\tz = %3m/s2\tLen = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(v.length(), w).toUtf8().constData();
        v = getAngAcc();
        qDebug() << QString("Dump Ang Acc:\tQ' = %1d/s2\tR' = %2d/s2\tP' = %3d/s2\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getTotalForces();
        qDebug() << QString("Dump Total Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getAeroForces();
        qDebug() << QString("Dump Aero Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getEngineForces();
        qDebug() << QString("Dump Engine Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getTotalMoments();
        qDebug() << QString("Dump Total Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getAeroMoments();
        qDebug() << QString("Dump Aero Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getEngineMoments();
        qDebug() << QString("Dump Engine Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        if (m_pCurrentCfg && m_pCurrentCfg->getTestPoint())
            qDebug() << QString("Dump Engine:\tN1 = %1%\tN2 = %2%\tEPR = %3\tThrust = %4N\tTP = %5%").arg(getN1mean(), w).arg(getN2mean(), w).arg(getEPRmean(), w).arg(getThrustTotal(), w).arg(m_pCurrentCfg->getTestPoint()->throttle * 100.0f, w).toUtf8().constData();
        else
            qDebug() << QString("Dump Engine:\tN1 = %1%\tN2 = %2%\tEPR = %3\tThrust = %4N\tTP = N/A").arg(getN1mean(), w).arg(getN2mean(), w).arg(getEPRmean(), w).arg(getThrustTotal(), w).toUtf8().constData();
        v = getMassMoments();
        qDebug() << QString("Dump Mass Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        qDebug() << QString("Dump Yoke:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getYokePitch(), w).arg(getYokeYaw(), w).arg(getYokeRoll(), w).toUtf8().constData();
        qDebug() << QString("Dump Trim:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getTrimPitch(), w).arg(getTrimYaw(), w).arg(getTrimRoll(), w).toUtf8().constData();
        qDebug() << QStringLiteral("Dump Throttles:\t").toUtf8().constData() << getThrottle();

        qDebug() << QString("Dump Yoke:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getYokePitch(), w).arg(getYokeYaw(), w).arg(getYokeRoll(), w).toUtf8().constData();
        qDebug() << QString("Dump Trim:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getTrimPitch(), w).arg(getTrimYaw(), w).arg(getTrimRoll(), w).toUtf8().constData();
        return;
    }

    if (m_bDumpPos) {
        QVectorD3D v = getGLpos();
        qDebug() << QString("Dump PosGL:\tx = %1m\ty = %2m\tz = %3m\tALT = %4m").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(getAltm(), w).toUtf8().constData();
        v = getPos();
        qDebug() << QString("Dump Pos:\tLat = %1deg\tLon = %2deg\tElev = %3m\tAGL = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(getAglm(), w).toUtf8().constData();
        v = m_cTestGLpos;
        qDebug() << QString("Dump TestPos:\tx = %1m\ty = %2m\tz = %3m").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
    }
    if (m_bDumpAttitude) {
        QVector3D v = getGLq().toEulerAngles();
        qDebug() << QString("Dump GL Q:\tTheta = %1deg\tPhi = %2deg\tPsi = %3deg").arg(v.x(), w).arg(v.z(), w).arg(v.y(), w).toUtf8().constData();
        //qDebug() << QString("Dump GL Att:\tTheta = %1deg\tPhi = %2deg\tPsi = %3deg").arg(getGLtheta(), w).arg(getGLphi(), w).arg(getGLpsi(), w).toUtf8().constData();
        qDebug() << QString("Dump True Att:\tTheta = %1deg\tPhi = %2deg\tPsi = %3deg").arg(getTrueTheta(), w).arg(getTruePhi(), w).arg(getTruePsi(), w).toUtf8().constData();
        qDebug() << QString("Dump Att:\tAlpha = %1deg\tBeta = %2deg\tVpath = %3deg\tHpath = %4").arg(getAlpha(), w).arg(getBeta(), w).arg(getVPath(), w).arg(getHPath(), w).toUtf8().constData();
    }
    if (m_bDumpLinVel) {
        QVector3D v = getGLvel();
        qDebug() << QString("Dump Vel:\tx = %1m/s\ty = %2m/s\tz = %3m/s\tLen = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(v.length(), w).toUtf8().constData();
        qDebug() << QString("Dump Vel:\tIAS = %1m/s\tTAS = %2m/s\tGS = %3m/s").arg(getIASms(), w).arg(getTASms(), w).arg(getGSms(), w).toUtf8().constData();
        //qDebug() << QString("Dump Vel:\tIAS = %1kt\tTAS = %2kt\tGS = %3kt").arg(getIASkt(), w).arg(getTASkt(), w).arg(getGSkt(), w).toUtf8().constData();
    }
    if (m_bDumpAngVel) {
        QVector3D v = getAngVel();
        qDebug() << QString("Dump Ang Vel:\tQ = %1r/s\tR = %2r/s\tP = %3r/s\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getAngVelDeg();
        qDebug() << QString("Dump Ang Vel:\tQ = %1deg\tR = %2deg\tP = %3deg\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
    }
    if (m_bDumpLinAcc) {
        QVector3D v = getOpenGLacc();
        qDebug() << QString("Dump Acc:\tx = %1m/s2\ty = %2m/s2\tz = %3m/s2\tLen = %4").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).arg(v.length(), w).toUtf8().constData();
    }
    if (m_bDumpAngAcc) {
        QVector3D v = getAngAcc();
        qDebug() << QString("Dump Ang Acc:\tQ' = %1d/s2\tR' = %2d/s2\tP' = %3d/s2\tPitch, Yaw, Roll\t").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
    }
    if (m_bDumpForces) {
        QVector3D v = getTotalForces();
        qDebug() << QString("Dump Total Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getAeroForces();
        qDebug() << QString("Dump Aero Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getEngineForces();
        qDebug() << QString("Dump Engine Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        //v = getGearForces();
        //qDebug() << QString("Dump Gear Forces:\tSide = %1N\tNormal = %2N\tAxil = %3N").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getGforces();
        qDebug() << QString("Dump G Forces:\tSide = %1G's\tNormal = %2G's\tAxil = %3G's").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        if (m_pCurrentCfg && m_pCurrentCfg->getTestPoint())
            qDebug() << QString("Dump Engine:\tN1 = %1%\tN2 = %2%\tEPR = %3\tThrust = %4N\tTP = %5%").arg(getN1mean(), w).arg(getN2mean(), w).arg(getEPRmean(), w).arg(getThrustTotal(), w).arg(m_pCurrentCfg->getTestPoint()->throttle * 100.0f, w).toUtf8().constData();
        else
            qDebug() << QString("Dump Engine:\tN1 = %1%\tN2 = %2%\tEPR = %3\tThrust = %4N\tTP = N/A").arg(getN1mean(), w).arg(getN2mean(), w).arg(getEPRmean(), w).arg(getThrustTotal(), w).toUtf8().constData();
    }
    if (m_bDumpMoments) {
        QVector3D v = getTotalMoments();
        qDebug() << QString("Dump Total Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getAeroMoments();
        qDebug() << QString("Dump Aero Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getEngineMoments();
        qDebug() << QString("Dump Engine Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        //v = getGearMoments();
        //qDebug() << QString("Dump Gear Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
        v = getMassMoments();
        qDebug() << QString("Dump Mass Mom.:\tPitch = %1Nm\tYaw = %2Nm\tRoll = %3Nm").arg(v.x(), w).arg(v.y(), w).arg(v.z(), w).toUtf8().constData();
    }
    if (m_bDumpControls) {
        qDebug() << QString("Dump Yoke:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getYokePitch(), w).arg(getYokeYaw(), w).arg(getYokeRoll(), w).toUtf8().constData();
        qDebug() << QString("Dump Trim:\tPitch = %1\tYaw = %2\tRoll = %3").arg(getTrimPitch(), w).arg(getTrimYaw(), w).arg(getTrimRoll(), w).toUtf8().constData();
        static XPdata *l1 = XPdata::ref("sim/flightmodel2/wing/elevator1_deg"); //write //These give you the most specific access to control surfaces
        static XPdata *l2 = XPdata::ref("sim/flightmodel2/wing/elevator2_deg");
        //static XPdata *l3 = XPdata::ref("sim/flightmodel2/controls/pitch_ratio"); //write //These are the actual control movements
        static XPdata *p3 = XPdata::ref("sim/flightmodel/controls/wing1l_elv1def");
        static XPdata *p4 = XPdata::ref("sim/flightmodel/controls/wing1l_elv2def");
        static XPdata *p5 = XPdata::ref("sim/flightmodel/controls/wing1r_elv1def");
        static XPdata *p6 = XPdata::ref("sim/flightmodel/controls/wing1r_elv2def");
        static XPdata *p7 = XPdata::ref("sim/flightmodel/controls/wing3l_elv1def");
        static XPdata *p8 = XPdata::ref("sim/flightmodel/controls/wing3l_elv2def");
        static XPdata *p9 = XPdata::ref("sim/flightmodel/controls/wing3r_elv1def");
        static XPdata *p10 = XPdata::ref("sim/flightmodel/controls/wing3r_elv2def");
        static XPdata *p11 = XPdata::ref("sim/flightmodel/controls/wing4l_elv1def");
        static XPdata *p12 = XPdata::ref("sim/flightmodel/controls/wing4l_elv2def");
        static XPdata *p13 = XPdata::ref("sim/flightmodel/controls/wing4r_elv1def");
        static XPdata *p14 = XPdata::ref("sim/flightmodel/controls/wing4r_elv2def");
        static XPdata *p15 = XPdata::ref("sim/flightmodel/controls/mwing03_elv1def");
        static XPdata *p16 = XPdata::ref("sim/flightmodel/controls/mwing03_elv2def");
        static XPdata *p17 = XPdata::ref("sim/flightmodel/controls/mwing04_elv1def");
        static XPdata *p18 = XPdata::ref("sim/flightmodel/controls/mwing04_elv2def");
        static XPdata *p19 = XPdata::ref("sim/flightmodel/controls/hstab1_elv1def");
        static XPdata *p20 = XPdata::ref("sim/flightmodel/controls/hstab1_elv2def");
        static XPdata *p21 = XPdata::ref("sim/flightmodel/controls/hstab2_elv1def");
        static XPdata *p22 = XPdata::ref("sim/flightmodel/controls/hstab2_elv2def");
        static XPdata *l23 = XPdata::ref("sim/flightmodel/controls/elv1_def"); //write elevators
        static XPdata *l24 = XPdata::ref("sim/flightmodel/controls/elv2_def");
        static XPdata *l25 = XPdata::ref("sim/flightmodel/parts/elev_cont_def"); // ro
        static XPdata *l26 = XPdata::ref("sim/flightmodel/parts/elev_trim_def"); // ro
        qDebug() << "Dump Elev:\telv1_deg = %1" << l1->getfList();
        qDebug() << "Dump Elev:\telv2_deg = %1" << l2->getfList();
        qDebug() << QString("Dump Elev:\tW1l1 = %1\tW1l2 = %2\tW1r1 = %3\tW1r2 = %4").arg(p3->getf(), w).arg(p4->getf(), w).arg(p5->getf(), w).arg(p6->getf(), w).toUtf8().constData();
        qDebug() << QString("Dump Elev:\tW3l1 = %1\tW3l2 = %2\tW3r1 = %3\tW3r2 = %4").arg(p7->getf(), w).arg(p8->getf(), w).arg(p9->getf(), w).arg(p10->getf(), w).toUtf8().constData();
        qDebug() << QString("Dump Elev:\tW4l1 = %1\tW4l2 = %2\tW4r1 = %3\tW4r2 = %4").arg(p11->getf(), w).arg(p12->getf(), w).arg(p13->getf(), w).arg(p14->getf(), w).toUtf8().constData();
        qDebug() << QString("Dump Elev:\tmW3_1 = %1\tmW3_2 = %2\tmW4_1 = %3\tmW4_2 = %4").arg(p15->getf(), w).arg(p16->getf(), w).arg(p17->getf(), w).arg(p18->getf(), w).toUtf8().constData();
        qDebug() << QString("Dump Elev:\tmHS11 = %1\tHS12 = %2\tHS21 = %3\tHS22 = %4").arg(p19->getf(), w).arg(p20->getf(), w).arg(p21->getf(), w).arg(p22->getf(), w).toUtf8().constData();
        qDebug() << "Dump Elev:\telv1_def = %1" << l23->getfList();
        qDebug() << "Dump Elev:\telv2_def = %1" << l24->getfList();
        qDebug() << "Dump Elev:\telev_cont = %1" << l25->getfList();
        qDebug() << "Dump Elev:\telev_trim = %1" << l26->getfList();
    }
    if (m_bDumpThrottles) {
        qDebug() << QStringLiteral("Dump Throttles:\t").toUtf8().constData() << getThrottle();
    }
}

void PerfPlugin::setControls(bool set)
{
    static bool notSet = true;
    static const QHash<QByteArray, QVariant> hash =
    {{"sim/operation/override/override_throttles", 1}, //Use ENGN_thro_use to control them.
     {"sim/operation/override/override_joystick", 1},
     //{"sim/operation/override/override_planepath", QVariant(QList<QVariant>{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1})}, //disables physics
     //{"sim/operation/override/override_groundplane", 1 },
     {"sim/joystick/yoke_pitch_ratio", 0.0f}, //override_joystick
     {"sim/joystick/yoke_roll_ratio", 0.0f}, //override_joystick
     {"sim/joystick/yoke_heading_ratio", 0.0f}, //override_joystick
     {"sim/aircraft/engine/acf_spooltime_jet", 0.5f}, //spool time makes engine fluctuate
     {"sim/aircraft/engine/acf_spooltime_prop", 0.5f},
     {"sim/aircraft/engine/acf_spooltime_turbine", 0.5f},
     //{"sim/flightmodel/engine/", 1},
     //{"sim/aircraft/engine/acf_num_engines", 1},
     {"sim/flightmodel/controls/sbrkrqst", 0.0f}, //Speed brake request
     //{"sim/flightmodel/controls/sbrkrat", 0.0f}, //Speed brake actual
     {"sim/cockpit2/controls/flap_ratio", 0.0f},
     {"sim/flightmodel/controls/flaprqst", 0.0f}, //Flap request
     //{"sim/flightmodel/controls/flaprat", 0.0f}, //Flap actual
     //{"sim/flightmodel/controls/flap2rat", 0.0f}, //Flap actual
     //{"sim/flightmodel/controls/slatrqst", 0.0f}, //Slat request //Not found!
     //{"sim/flightmodel/controls/slatrat", 0.0f}, //Slat actual
     {"sim/flightmodel/controls/ail_trim", 0.0f},
     {"sim/flightmodel/controls/elv_trim", 0.0f},
     {"sim/flightmodel/controls/rud_trim", 0.0f},

     {"sim/aircraft/controls/acf_elev_trim_speedrat", 200.0f}, // 1.0 = 20s, speed = 20s/speedrat
     {"sim/aircraft/controls/acf_ailn_trim_speedrat", 200.0f}, // 1.0 = 20s, speed = 20s/speedrat
     {"sim/aircraft/controls/acf_rudd_trim_speedrat", 200.0f}, // 1.0 = 20s, speed = 20s/speedrat
     {"sim/aircraft/controls/acf_elev_def_time", 0.0f}, // sec deflection time
     {"sim/aircraft/controls/acf_ailn_def_time", 0.0f}, // sec deflection time
     {"sim/aircraft/controls/acf_rudd_def_time", 0.0f}, // sec deflection time
     {"sim/aircraft/controls/acf_elev_trim_time", 0.0f}, // sec deflection time
     {"sim/aircraft/controls/acf_ailn_trim_time", 0.0f}, // sec deflection time
     {"sim/aircraft/controls/acf_rudd_trim_time", 0.0f}, // sec deflection time

     {"sim/flightmodel/misc/cgz_ref_to_default", 0.0f}, //Set cg to default

     //Artstability

     {"sim/operation/override/override_flightcontrol", 1}, // Flight control computer = CAS + stability -> off
     {"sim/operation/override/override_artstab", 1}, // ??? -> off
     {"sim/aircraft/artstability/acf_AShiV", 1.0f},
     {"sim/aircraft/artstability/acf_ASloV", 0.0f},
     {"sim/aircraft/artstability/acf_ASp_hi_pos", 0.0f},
     {"sim/aircraft/artstability/acf_ASp_lo_rate", 0.0f},
     {"sim/aircraft/artstability/acf_ASh_hi_pos", 0.0f},
     {"sim/aircraft/artstability/acf_ASh_lo_rate", 0.0f},
     {"sim/aircraft/artstability/acf_ASr_hi_rate", 0.0f},
     {"sim/aircraft/artstability/acf_ASr_lo_rate", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxp_hi", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxp_lo", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxh_hi", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxh_lo", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxr_hi", 0.0f},
     {"sim/aircraft/artstability/acf_ASmaxr_lo", 0.0f},
     {"sim/aircraft/artstability/acf_AShiV", 0.0f},
     {"sim/aircraft/artstability/acf_AShiV", 0.0f},
     {"sim/aircraft/artstability/acf_AShiV", 0.0f},
     {"sim/aircraft/artstability/acf_AShiV", 0.0f},
     //{"sim/aircraft/artstability/acf_AShi_roll_rat", 1.0f},


     {"sim/flightmodel/controls/tailhook_ratio", 0.0f},
     {"sim/flightmodel/controls/canopy_ratio", 0.0f},
     {"sim/cockpit/switches/canopy_req", 0},
     {"sim/cockpit/switches/arresting_gear", 0},
     {"sim/cockpit/switches/dumping_fuel", 0},
     //{"sim/flightmodel/controls/incid_rqst", 0.0f}, //Requested incidence
     //{"sim/flightmodel/controls/dihed_rqst", 0.0f}, //Requested dihedral
     //{"sim/flightmodel2/gear/deploy_ratio", QVariant({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f})},
     //{"sim/aircraft/parts/acf_gear_def", 0.0f}, //Not Found!
     {"sim/cockpit/autopilot/autopilot_mode", 0}, // Autopilot off = 0, FD = 1, on = 2!
     {"sim/cockpit/autopilot/heading_roll_mode", 0},
     {"sim/cockpit/autopilot/vertical_velocity", 0.0f},
     {"sim/cockpit/autopilot/autopilot_state", 0x0}, // Autopilot engaged modes bitfield 0x4 (roll hold), 0x10 (V/S select), 0x20 (Alt Arm), 0x40 (FLCH), 0x4000 (Alt hold), 0x40000 (VNAV engaged)
     {"sim/cockpit/switches/gear_handle_status", 0.0f},
     {"sim/aircraft/parts/acf_gear_deploy", QVariant({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f})},
     //{"sim/joystick/artstab_pitch_ratio", 0.0f}, //override_artstab
     //{"sim/joystick/artstab_roll_ratio", 0.0f}, //override_artstab
     //{"sim/joystick/artstab_heading_ratio", 0.0f}, //override_artstab
     //{"sim/time/zulu_time_sec", 12.0f * 3600}}; //Seconds since midnight Zulu
     {"sim/flightmodel/engine/ENGN_thro", QVariant({0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f})}}; //Save user throttle
    static QHash<QByteArray, QVariant> saved = hash;

    if (set) {
        if (notSet) {
            qDebug() << "Save Controls";
            XPdata::get(&saved);
        }
        qDebug() << "Set Controls";
        XPdata::set(&hash);
        notSet = false;
        qDebug() << "Done Controls";
    } else if (!set && !notSet) {
        qDebug() << "Restore Controls";
        XPdata::set(&saved);
        notSet = true;
        qDebug() << "Done Controls";
    }

    //XPdata::set(&hash);
    /*
    QHash<QByteArray, QVariant>::const_iterator i = hash.constBegin();
    while (i != hash.constEnd()) {
        XPdata::ref(i.key())->set(i.value());

        //XPdata *d = XPdata::ref(i.key());
        //if (d)
        //    d->set(i.value());

        ++i;
    }
    */
}

Aircraft* PerfPlugin::getAircraft()
{
    return m_pCurAC;
}

QString PerfPlugin::getCurrentAC()
{
    char path[XPLM_PATH_BUFFER + 1];
    char filename[XPLM_PATH_BUFFER + 1];

    XPLMGetNthAircraftModel(0, filename, path);
    qDebug() << "Found path" << path << "and filename" << filename;

    return QString(path);
}

void PerfPlugin::setCurrentAC(Aircraft *ac)
{
    setCurrentAC(ac->path().toUtf8());
}

void PerfPlugin::setCurrentAC(QByteArray file)
{
    XPLMSetUsersAircraft(file.constData());
}

void PerfPlugin::setPitch(float f) //-2 ... 2
{
    float positive = 1.0f;
    float abs = qAbs(f);
    if (f < 0)
        positive = -1.0f;

    //qDebug() << "SetPitch, f =" << f << "abs =" << abs << "positive = " << positive << "Yoke =" << (abs > 1.0f ? positive * (abs - 1.0f) : 0.0f);
    setTrimPitch(f);
    if (abs > 1.0f)
        setYokePitch(positive * (abs - 1.0f));
    else
        setYokePitch(0.0f);
}

void PerfPlugin::setRoll(float f)
{
    float positive = 1.0f;
    float abs = qAbs(f);
    if (f < 0)
        positive = -1.0f;

    setTrimRoll(f);
    if (abs > 1.0f)
        setYokeRoll(positive * (abs - 1.0f));
    else
        setYokeRoll(0.0f);
}

void PerfPlugin::setYaw(float f)
{
    float positive = 1.0f;
    float abs = qAbs(f);
    if (f < 0)
        positive = -1.0f;

    setTrimYaw(f);
    if (abs > 1.0f)
        setYokeYaw(positive * (abs - 1.0f));
    else
        setYokeYaw(0.0f);
}

void PerfPlugin::setYokePitch(float f) //-1 ... 1
{
    //static XPdata* const p = XPdata::ref("sim/joystick/yoke_pitch_ratio");
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/pitch_ratio");

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getYokePitch() //-1 ... 1
{
    //static XPdata *trim = XPdata::ref("sim/joystick/yoke_pitch_ratio");
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/pitch_ratio");

    return trim->getf();
}

void PerfPlugin::setYokeRoll(float f)
{
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/roll_ratio");
    //static XPdata* const p = XPdata::ref("sim/joystick/yoke_roll_ratio");

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getYokeRoll() //-1 ... 1
{
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/roll_ratio");
    //static XPdata *trim = XPdata::ref("sim/joystick/yoke_roll_ratio");

    return trim->getf();
}

void PerfPlugin::setYokeYaw(float f)
{
    //static XPdata* const p = XPdata::ref("sim/joystick/yoke_heading_ratio");
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/heading_ratio");

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getYokeYaw() //-1 ... 1
{
    //static XPdata *trim = XPdata::ref("sim/joystick/yoke_heading_ratio");
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/heading_ratio");

    return trim->getf();
}

void PerfPlugin::setTrimPitch(float f) //-1 ... 1
{
    //static XPdata* const p = XPdata::ref("sim/flightmodel/controls/elv_trim");
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/elevator_trim");

    //qDebug() << "PerfPlugin::setTrimPitch" << f;

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getTrimPitch() //-1 ... 1
{
    //static XPdata *trim = XPdata::ref("sim/flightmodel/controls/elv_trim");
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/elevator_trim");

    return trim->getf();
}

void PerfPlugin::setTrimRoll(float f)
{
    //static XPdata* const p = XPdata::ref("sim/flightmodel/controls/ail_trim");
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/aileron_trim");

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getTrimRoll() //-1 ... 1
{
    //static XPdata *trim = XPdata::ref("sim/flightmodel/controls/ail_trim");
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/aileron_trim");

    return trim->getf();
}

void PerfPlugin::setTrimYaw(float f)
{
    //static XPdata* const p = XPdata::ref("sim/flightmodel/controls/rud_trim");
    static XPdata* const p = XPdata::ref("sim/flightmodel2/controls/rudder_trim");

    if (f > 1.0f)
        p->set(1.0f);
    else if (f < -1.0f)
        p->set(-1.0f);
    else
        p->set(f);
}

float PerfPlugin::getTrimYaw() //-1 ... 1
{
    //static XPdata *trim = XPdata::ref("sim/flightmodel/controls/rud_trim");
    static XPdata* const trim = XPdata::ref("sim/flightmodel2/controls/rudder_trim");

    return trim->getf();
}

void PerfPlugin::setControlsP(QList<float> v) //get first and then modify
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/elevator1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/elevator2_deg");

    p1->set(v.first(v.size() / 2.0f));
    p2->set(v.last(v.size() / 2.0f));

    //PerfPlugin::get()->dumpData("Controls...");
}

void PerfPlugin::setControlsR(QList<float> v)
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/aileron1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/aileron2_deg");

    p1->set(v.first(v.size() / 2.0f));
    p2->set(v.last(v.size() / 2.0f));
}

void PerfPlugin::setControlsY(QList<float> v)
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/rudder1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/rudder2_deg");

    p1->set(v.first(v.size() / 2.0f));
    p2->set(v.last(v.size() / 2.0f));
}

QList<float> PerfPlugin::getControlsP() //get first and then modify
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/elevator1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/elevator2_deg");

    QList<float> l = p1->getfList();
    l.append(p2->getfList());

    return l;
}

QList<float> PerfPlugin::getControlsR()
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/aileron1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/aileron2_deg");

    QList<float> l = p1->getfList();
    l.append(p2->getfList());

    return l;
}

QList<float> PerfPlugin::getControlsY()
{
    static XPdata *p1 = XPdata::ref("sim/flightmodel2/wing/rudder1_deg");
    static XPdata *p2 = XPdata::ref("sim/flightmodel2/wing/rudder2_deg");

    QList<float> l = p1->getfList();
    l.append(p2->getfList());

    return l;
}

int PerfPlugin::getNumEngines() //sim/aircraft/engine/acf_num_engines
{
    static XPdata *p = XPdata::ref("sim/aircraft/engine/acf_num_engines");
    return p->geti();
}

float PerfPlugin::getMaxThrottle() //sim/aircraft/engine/acf_throtmax_FWD
{
    static XPdata *p = XPdata::ref("sim/aircraft/engine/acf_throtmax_FWD");
    return p->getf();
}

float PerfPlugin::getMinThrottle() //sim/aircraft/engine/acf_throtmax_REV
{
    static XPdata *p = XPdata::ref("sim/aircraft/engine/acf_throtmax_REV");
    return p->getf();
}

QList<float> PerfPlugin::getThrottle()
{
    static XPdata* throttle = XPdata::ref("sim/flightmodel/engine/ENGN_thro_use");
    return throttle->getfList();
}

QList<float> PerfPlugin::getFF() // sim/flightmodel/engine/ENGN_FF_
{
    static XPdata* p = XPdata::ref("sim/flightmodel/engine/ENGN_FF_");
    return p->getfList();
}

QList<float> PerfPlugin::getN1() // sim/flightmodel/engine/ENGN_N1_
{
    static XPdata* p = XPdata::ref("sim/flightmodel/engine/ENGN_N1_");
    return p->getfList();
}

QList<float> PerfPlugin::getN2() // sim/flightmodel/engine/ENGN_N2_
{
    static XPdata* p = XPdata::ref("sim/flightmodel/engine/ENGN_N2_");
    return p->getfList();
}

QList<float> PerfPlugin::getEPR() // sim/flightmodel/engine/ENGN_EPR
{
    static XPdata* p = XPdata::ref("sim/flightmodel/engine/ENGN_EPR");
    return p->getfList();
}

QList<float> PerfPlugin::getThrust() // sim/flightmodel/engine/POINT_thrust
{
    static XPdata* p = XPdata::ref("sim/flightmodel/engine/POINT_thrust");
    return p->getfList();
}

float PerfPlugin::getFFtotal() // sim/flightmodel/engine/ENGN_FF_
{
    static QList<float> p;
    static float f;
    p = getFF();
    f = 0.0f;
    for (qsizetype i = 0; i < p.size(); ++i)
        f += p.at(i);
    return f;
}

float PerfPlugin::getN1mean() // sim/flightmodel/engine/ENGN_N1_
{
    static QList<float> p;
    static float f, size;
    p = getN1();
    f = 0.0f;
    size = qMin(p.size(), getNumEngines());

    for (qsizetype i = 0; i < size; ++i)
        f += p.at(i);

    return f / getNumEngines();
}

float PerfPlugin::getN2mean() // sim/flightmodel/engine/ENGN_N2_
{
    static QList<float> p;
    static float f, size;
    p = getN2();
    f = 0.0f;
    size = qMin(p.size(), getNumEngines());

    for (qsizetype i = 0; i < size; ++i)
        f += p.at(i);

    return f / getNumEngines();
}

float PerfPlugin::getEPRmean() // sim/flightmodel/engine/ENGN_EPR
{
    static QList<float> p;
    static float f;
    p = getEPR();
    f = 0.0f;

    for (qsizetype i = 0; i < p.size(); ++i)
        f += p.at(i);

    return f / getNumEngines();
}

float PerfPlugin::getThrustTotal() // sim/flightmodel/engine/POINT_thrust
{
    static QList<float> p;
    static float f;
    p = getThrust();
    f = 0;
    for (qsizetype i = 0; i < p.size(); ++i)
        f += p.at(i);
    return f;
}

//{"sim/flightmodel/engine/ENGN_thro_use", QVariant({1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f})}};
void PerfPlugin::setThrottle(float pos)
{
    static XPdata* throttle = XPdata::ref("sim/flightmodel/engine/ENGN_thro_use");
    //static XPdata* throttle = XPdata::ref("sim/flightmodel/engine/ENGN_thro");
    //static XPdata* override = XPdata::ref("sim/operation/override/override_throttles");
    QList<float> throttles = throttle->getfList();
    //override->set(1);

    for (int i = 0; i < throttles.size(); ++i)
        throttles[i] = pos;

    throttle->set(throttles);
}

//Aircraft weight
float PerfPlugin::getCG() // sim/flightmodel/misc/cgz_ref_to_default
{
    static XPdata* p = XPdata::ref("sim/flightmodel/misc/cgz_ref_to_default");
    return p->getf();
}

float PerfPlugin::getCGfwd() // sim/aircraft/overflow/acf_cgZ_fwd //meters
{
    static XPdata* p = XPdata::ref("sim/aircraft/overflow/acf_cgZ_fwd");
    return p->getf();
}

float PerfPlugin::getCGaft() // sim/aircraft/overflow/acf_cgZ_aft //meters
{
    static XPdata* p = XPdata::ref("sim/aircraft/overflow/acf_cgZ_aft");
    return p->getf();
}

float PerfPlugin::getGrossMass() // sim/flightmodel/weight/m_total
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_total");
    return p->getf();
}

float PerfPlugin::getTrafficLoad() // sim/flightmodel/weight/m_fixed //Payload
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_fixed");
    return p->getf();
}

float PerfPlugin::getTotalFuel() // sim/flightmodel/weight/m_fuel_total
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_fuel_total");
    return p->getf();
}

int PerfPlugin::getNumTanks() //sim/aircraft/overflow/acf_num_tanks
{
    static XPdata* p = XPdata::ref("sim/aircraft/overflow/acf_num_tanks");
    return p->geti();
}

float PerfPlugin::getTankFuel(unsigned int n) // sim/flightmodel/weight/m_fuel[9]
{
    QList<float> p = getTankFuel();
    return (p.size() > --n) ? p.at(n) : 0.0f;
}

QList<float> PerfPlugin::getTankFuel() // sim/flightmodel/weight/m_fuel[9]
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_fuel");
    return p->getfList();
}

QList<float> PerfPlugin::getTankSize() //sim/aircraft/overflow/acf_tank_rat // ratio of total tank capacity (Sum should be 1)
{
    static XPdata* r = XPdata::ref("sim/aircraft/overflow/acf_tank_rat");
    QList<float> p =  r->getfList();
    float f = 0;
    for (qsizetype i = 0; i < p.size(); ++i)
        f += p.at(i);
    if (f != 1.0f) {
        for (qsizetype i = 0; i < p.size(); ++i)
            p[i] /= f;
    }
    return p;
}

float PerfPlugin::getDOM() //sim/aircraft/weight/acf_m_empty // DOM = BEM(includes oil etc) + crew + catering
{
    static XPdata* p = XPdata::ref("sim/aircraft/weight/acf_m_empty");
    return p->getf();
}

float PerfPlugin::getMaxFuel() // sim/aircraft/weight/acf_m_fuel_tot
{
    static XPdata* p = XPdata::ref("sim/aircraft/weight/acf_m_fuel_tot");
    return p->getf();
}

float PerfPlugin::getMTOM() //sim/aircraft/weight/acf_m_max
{
    static XPdata* p = XPdata::ref("sim/aircraft/weight/acf_m_max");
    return p->getf();
}

void PerfPlugin::setCG(float f) // sim/flightmodel/misc/cgz_ref_to_default
{
    static XPdata* p = XPdata::ref("sim/flightmodel/misc/cgz_ref_to_default");
    p->set(f);
}

void PerfPlugin::setTrafficLoad(float f) // sim/flightmodel/weight/m_fixed
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_fixed");
    p->set(f);
}

void PerfPlugin::setTankFuel(int n, float f) // sim/flightmodel/weight/m_fuel[9]
{
    QList<float> p = getTankFuel();
    if (p.size() > --n) {
        p[n] = f;
    }
}

void PerfPlugin::setTankFuel(QList<float> f) // sim/flightmodel/weight/m_fuel[9]
{
    static XPdata* p = XPdata::ref("sim/flightmodel/weight/m_fuel");
    p->set(f);
}

//Aircraft speeds
Quantity PerfPlugin::getVMO() // sim/aircraft/view/acf_Vne
{
    static XPdata* p = XPdata::ref("sim/aircraft/view/acf_Vne");
    qDebug() << "GetVMO()" << p->getf();
    return Quantity(Unit("kt", 1.852f/3.6f), p->getf());
    //return p->getf();
}

float PerfPlugin::getMMO() // sim/aircraft/view/acf_Mmo
{
    static XPdata* p = XPdata::ref("sim/aircraft/view/acf_Mmo");
    return p->getf();
}


void PerfPlugin::setGear(bool pos) // 0.0f is up, 1.0f is down
{
    //static XPdata* gear = XPdata::ref("sim/flightmodel2/gear/deploy_ratio");
    static XPdata* gear = XPdata::ref("sim/aircraft/parts/acf_gear_deploy");
    static XPdata* handle = XPdata::ref("sim/cockpit/switches/gear_handle_status");
    static QList<float> gears = gear->getfList();

    for (int i = 0; i < gears.size(); ++i)
        gears[i] = pos;

    qDebug() << "Set Gears" << gears;

    gear->set(gears);
    handle->set(pos);
}

bool PerfPlugin::getGear()
{
    static XPdata* gear = XPdata::ref("sim/aircraft/parts/acf_gear_deploy");
    static XPdata* handle = XPdata::ref("sim/cockpit/switches/gear_handle_status");

    //static XPdata* gear = XPdata::ref("sim/flightmodel2/gear/deploy_ratio");
    //static XPdata* def = XPdata::ref("sim/aircraft/parts/acf_gear_def"); //not found
    //static XPdata* handle2 = XPdata::ref("sim/cockpit2/controls//gear_handle_down"); //not found

    qDebug() << "Gear =" << gear->getfList() << "handle" << handle->get();

    return handle->getf();
    //return gear->getf();
}

void PerfPlugin::setFlaps(float pos)
{
    static XPdata* flap = XPdata::ref("sim/flightmodel/controls/flaprqst");

    flap->set(pos);
    /*
    static XPdata* flap1 = XPdata::ref("sim/flightmodel/controls/flaprat");
    static XPdata* flap2 = XPdata::ref("sim/flightmodel/controls/flap2rat");
    static XPdata* flapdef1 = XPdata::ref("sim/flightmodel/controls/fla1_def");
    static XPdata* flapdef2 = XPdata::ref("sim/flightmodel/controls/fla2_def");
    static QList<float> flapdef = flapdef1->getfList();

    flap1->set(pos);
    flap2->set(pos);

    if (!pos) {
        for (int i = 0; i < flapdef.size(); ++i)
            flapdef[i] = 0.0f;
        flapdef1->set(flapdef);
        flapdef2->set(flapdef);
    }
    */
}

float PerfPlugin::getFlaps()
{
    static XPdata* flap1 = XPdata::ref("sim/flightmodel/controls/flaprat");

    /*
     * (static XPdata* flap = XPdata::ref("sim/flightmodel/controls/flaprqst");

    //static XPdata* slat = XPdata::ref("sim/flightmodel/controls/slatrqst"); //Not found
    static XPdata* flap1 = XPdata::ref("sim/flightmodel/controls/flaprat");
    static XPdata* flap2 = XPdata::ref("sim/flightmodel/controls/flap2rat");
    static XPdata* slatA = XPdata::ref("sim/flightmodel/controls/slatrat");
    static XPdata* flapdef1 = XPdata::ref("sim/flightmodel/controls/fla1_def");
    static XPdata* flapdef2 = XPdata::ref("sim/flightmodel/controls/fla2_def");

    XPdata::inc();
    qDebug() << "Flap =" << flap->getf() << "Actual" << flap1->getf() << flap2->getf() << "Slat" << slatA ->getf() << flapdef1->getfList() << flapdef2->getfList();
    */

    return flap1->getf();
}

/*
void PerfPlugin::setPitchTrim(float val)
{
    static XPdata *trim = XPdata::ref("sim/flightmodel/controls/elv_trim");

    qDebug() << "setPitchTrim" << val;

    trim->set(val);
}

void PerfPlugin::setRollTrim(float val)
{
    static XPdata *trim = XPdata::ref("sim/flightmodel/controls/ail_trim");

    trim->set(val);
}

void PerfPlugin::setYawTrim(float val)
{
    static XPdata *trim = XPdata::ref("sim/flightmodel/controls/rud_trim");

    trim->set(val);
}
*/

void PerfPlugin::setCAS(Quantity val, bool rotate)
{
    setTAS(Aerodynamics::casToTas(val, getEnvironment()), rotate);
}

void PerfPlugin::setTAS(Quantity val, bool rotate)
{
    QVector3D vel(0.0f, 0.0f, -val.raw());

    if (rotate)
        vel = getGLq() * vel;
        //vel = getOpenGLatt() * vel;

    //qDebug() << "Set TAS =" << val.toStr() << vel;
    setOpenGLvel(vel);
}

void PerfPlugin::setISAweather(bool set) // true to set and false to restore
{
    static bool notSet = true;
    static const QHash<QByteArray, QVariant> hash =
    {{"sim/weather/cloud_type[0]", 0},
     {"sim/weather/cloud_type[1]", 0},
     {"sim/weather/cloud_type[2]", 0},
     {"sim/weather/visibility_reported_m", 20000},
     {"sim/weather/rain_percent", 0},
     {"sim/weather/thunderstorm_percent", 0},
     {"sim/weather/wind_turbulence_percent", 0},
     {"sim/weather/barometer_sealevel_inhg", 29.92126f},
//     {"sim/weather/barometer_sealevel_inhg", 24.0f},
//     {"sim/cockpit/misc/barometer_setting", 24.0f},
     {"sim/cockpit/misc/barometer_setting", 29.92126f},
     {"sim/weather/use_real_weather_bool", 0},
     {"sim/weather/wind_altitude_msl_m[0]", 5000},
     {"sim/weather/wind_altitude_msl_m[1]", 10000},
     {"sim/weather/wind_altitude_msl_m[2]", 15000},
     {"sim/weather/wind_speed_kt[0]", 0},
     {"sim/weather/wind_speed_kt[1]", 0},
     {"sim/weather/wind_speed_kt[2]", 0},
     {"sim/weather/shear_speed_kt[0]", 0},
     {"sim/weather/shear_speed_kt[1]", 0},
     {"sim/weather/shear_speed_kt[2]", 0},
     {"sim/weather/turbulence[0]", 0},
     {"sim/weather/turbulence[1]", 0},
     {"sim/weather/turbulence[2]", 0},
     {"sim/weather/wave_amplitude", 0},
     {"sim/weather/wave_speed", 0},
     {"sim/weather/temperature_sealevel_c", 15},
     {"sim/weather/temperature_tropo_c", -56.5},
     {"sim/weather/tropo_alt_mtr", 11000},
     {"sim/weather/dewpoi_sealevel_c", -200}, //below -60 C air is nearly dry
     {"sim/weather/thermal_percent", 0},
     {"sim/weather/thermal_rate_ms", 0},
     {"sim/physics/metric_temp", 1},
     {"sim/physics/metric_press", 1},
     {"sim/time/zulu_time_sec", 8.0f * 3600 + 40.0f * 60}}; //Seconds since midnight Zulu
    static QHash<QByteArray, QVariant> saved = hash;
    //saved["test"] = "Create copy";
    //saved.remove("test");

    //{"sim/weather/rate_change_percent", 0.0f}, //Fake
    //{"sim/weather/microburst_probability", 0.0f}, //Fake
    //acf_num_engines //num_engines

    if (set) {
        if (notSet) {
            qDebug() << "Save Weather";
            XPdata::get(&saved);
        }
        qDebug() << "Set ISA";
        XPdata::set(&hash);
        qDebug() << "Done.";
        notSet = false;
    } else if (!set && !notSet) {
        qDebug() << "Restore ISA";
        XPdata::set(&saved);
        notSet = true;
    }
}


bool PerfPlugin::processCurrent()
{
    //XPdata* joo = XPdata::ref("sim/flightmodel/forces/L_aero");
    m_pCurrent->fnrml_aero = XPdata::get("sim/flightmodel/forces/fnrml_aero").toFloat();

    return true;
}

void PerfPlugin::newPoint(TestPoint * pNewPoint)
{
    m_cQueue.append(pNewPoint);
    sortQueue();
    emit inData();
}

void PerfPlugin::newPoint(TestPointList cNewPoints)
{
    m_cQueue.append(cNewPoints);
    sortQueue();
    emit inData();
}

void PerfPlugin::newConfig(Config *pNewCfg)
{
    m_cCfgQueue.append(pNewCfg);
    sortQueue();
    emit inData();
}

void PerfPlugin::newConfig(Aircraft *pAC)
{
    m_cCfgQueue.append(pAC->getCfgs());
    sortQueue();
    emit inData();
}

void PerfPlugin::sortQueue()
{
    //std::sort(m_cCfgQueue.begin(), m_cCfgQueue.end(), this->compareCfg);
    Config *tmp = nullptr;
    qsizetype i = 0, k = 0;
    bool result = true;

    for(k = m_cCfgQueue.size() - 1; k > 0; ++k) {
        result = true;
        for(i = 0; i < k; ++i) {
            if (!compareCfg(m_cCfgQueue.at(i), m_cCfgQueue.at(i+1))) {
                result = false;
                tmp = m_cCfgQueue[i];
                m_cCfgQueue[i] = m_cCfgQueue[i+1];
                m_cCfgQueue[i+1] = tmp;
            }
        }
        if (result)
            break;
    }

    return;
}

bool PerfPlugin::compareCfg(const Config *a, const Config *b) const
{
    if (a == m_pCurrentCfg)
        return true;
    if (b == m_pCurrentCfg)
        return false;
    return a < b;
}

void PerfPlugin::hardReset()
{
    pluginDisable();

    m_eComState = off;
    m_eCurState = off;
    m_pCurAC = Aircraft::addAircraft(getCurrentAC());
    m_pCurAC->readXPdata();

    pluginEnable();
}


int PerfPlugin::pluginStart(char * outName,
                            char * outSig,
                            char *outDesc)
{
    XPLMMenuID	id;
    int			item;

    INFO << "Plugin started";
    qstrncpy(outName, pluginName.constData(), XPLM_BUFFER_SIZE);
    qstrncpy(outSig, "ilmavoimat.mny.perfplugin", XPLM_BUFFER_SIZE);
    qstrncpy(outDesc, "Aircraft performance analyzer", XPLM_BUFFER_SIZE);

    setlocale(LC_NUMERIC, "C"); // See http://stackoverflow.com/questions/25661295/why-does-qcoreapplication-call-setlocalelc-all-by-default-on-unix-linux

    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS",1);

    item = XPLMAppendMenuItem(XPLMFindPluginsMenu(), pluginName.constData(), NULL, 1);
    id = XPLMCreateMenu(pluginName.constData(), XPLMFindPluginsMenu(), item, XPluginMenuHandler, NULL);
    XPLMAppendMenuItem(id, pluginName.constData(), (void *)m_menuStart.constData(), 1);
    XPLMAppendMenuItem(id, m_menuReset.constData(), (void *)m_menuReset.constData(), 1);

    return 1;
}

void PerfPlugin::pluginStop()
{
    CONSOLE("Delete PerfApp");
    XPdata::clearAll();
    delete m_pPerfApp;
    m_pPerfApp = nullptr;
}

int PerfPlugin::pluginEnable()
{
    createCallBacks();

    return 1;
}

void PerfPlugin::pluginDisable()
{
    destroyCallBacks();
}

void PerfPlugin::receivedMessage(XPLMPluginID inFromWho, long inMessage, void *inParam)
{
    Q_UNUSED(inFromWho);
    Q_UNUSED(inMessage);
    Q_UNUSED(inParam);

    if (inMessage == XPLM_MSG_PLANE_CRASHED) {
        return hardReset();
    } else if (inMessage == XPLM_MSG_PLANE_LOADED) {
        return hardReset();
    } else if (inMessage == XPLM_MSG_AIRPORT_LOADED) {
        return hardReset();
    }
}

void PerfPlugin::createCallBacks()
{
    XPLMCreateFlightLoop_t preLoop;
    XPLMCreateFlightLoop_t afterLoop;

    preLoop.structSize = sizeof(XPLMCreateFlightLoop_t);
    afterLoop.structSize = sizeof(XPLMCreateFlightLoop_t);

    preLoop.phase = xplm_FlightLoop_Phase_BeforeFlightModel;
    afterLoop.phase = xplm_FlightLoop_Phase_AfterFlightModel;

    preLoop.callbackFunc = PrePerfFlightLoopCallback;
    afterLoop.callbackFunc = AfterPerfFlightLoopCallback;

    preLoop.refcon = nullptr;
    afterLoop.refcon = nullptr;

    preLoopId = XPLMCreateFlightLoop(&preLoop);
    afterLoopId = XPLMCreateFlightLoop(&afterLoop);

    XPLMRegisterFlightLoopCallback(StartPerfFlightLoopCallback, 0.0f, NULL);
    //XPLMScheduleFlightLoop(preLoopId, -1.0, true);
    //XPLMScheduleFlightLoop(afterLoopId, -1.0, true);

    m_bFltRunning = false;
}

void PerfPlugin::destroyCallBacks()
{
    if (preLoopId) {
        XPLMDestroyFlightLoop(preLoopId);
        preLoopId = nullptr;
    }

    if (afterLoopId) {
        XPLMDestroyFlightLoop(afterLoopId);
        afterLoopId = nullptr;
    }

    XPLMUnregisterFlightLoopCallback(StartPerfFlightLoopCallback, 0);

    //m_bFltRunning = false;
}

void PerfPlugin::scheduleFL()
{
    XPLMSetFlightLoopCallbackInterval(StartPerfFlightLoopCallback, -1.0f, 0, NULL);
    qDebug() << "Scheduled";

    //qDebug() << "Scheduling" << preLoopId << afterLoopId;

//    if (m_bFltRunning)
//        return;

//    if (preLoopId)
//        XPLMScheduleFlightLoop(preLoopId, -1.0f, false);
//    XPLMScheduleFlightLoop(preLoopId, 1.0f, false);
    //XPLMScheduleFlightLoop(preLoopId, 1.0, true);
    //XPLMScheduleFlightLoop(afterLoopId, 1.0, true);
//    XPLMRegisterFlightLoopCallback(PrePerfFlightLoopCallback, -1.0f, NULL);
//    XPLMSetFlightLoopCallbackInterval(StartPerfFlightLoopCallback, -1.0f, 0, NULL);

//    if (afterLoopId)
//        XPLMScheduleFlightLoop(afterLoopId, -1.0f, false);

}

/*
void PerfPlugin::registerFL()
{
    XPLMRegisterFlightLoopCallback(PerfFlightLoopCallback, -1.0f, NULL);
    m_bFltRunning = true;
}

void PerfPlugin::unregisterFL()
{
    if (!m_bFltRunning)
        XPLMUnregisterFlightLoopCallback(PerfFlightLoopCallback, 0);
    m_bFltRunning = false;
}
*/

void PerfPlugin::menuHandler(void *mRef, char *iRef)
{
    Q_UNUSED(mRef);
    Q_UNUSED(iRef);

    CONSOLE("PerfPlugin menuhandler");

    if (m_menuStart == iRef && !m_pPerfApp) {
        //handle menu press
        //CONSOLE("Creating a new thread");

        char path[XPLM_BUFFER_SIZE + 1];
        XPLMPluginID id = XPLMGetMyID();
        XPLMGetPluginInfo(id, nullptr, path, nullptr, nullptr);
        char *file = XPLMExtractFileAndPath(path);
        m_sQtPluginPath = path;
        Q_UNUSED(file);

        //m_pPerfApp = new PerfThread(m_sPluginPath);

        if (!qApp) {
            CONSOLE("qApp == null");
            int argc = 0;
            char *argv = nullptr;

            //qputenv("QT_PLUGIN_PATH", m_sQtPluginPath.toUtf8());
            QCoreApplication::addLibraryPath(m_sQtPluginPath);

            CONSOLE("New QApplication");

            //QApplication may not be deleted. It leads to a crash (static elements and the Windows event loop). Once X-Plane exits QApplication gets deleted.
            new QApplication(argc, &argv);
            QCoreApplication::setOrganizationName(pluginOrg);
            QCoreApplication::setApplicationName(pluginName);
            QCoreApplication::setApplicationVersion(QT_VERSION_STR);

            CONSOLE("App created");
        }
        CONSOLE("Create windows");

        m_pPerfApp = new PerfAnal();

        if (checkGravity()) {
            m_pPerfApp->show();
        } else {
            m_pPerfApp->hide();

            QMessageBox msg;
            msg.setText("Gravity is set incorrectly! Standard ISA conditions can't be met.");

            XPdata* mu = XPdata::ref("sim/physics/earth_mu");
            XPdata* rad = XPdata::ref("sim/physics/earth_radius_m");

            msg.setInformativeText(QString("Gravity does not meet ISA conditions. Please set under 'Special'->'Set Environment Properties' the planet mu to 3.983 and radius to 6 373km. This brings g close to standard value 9.80665 at MSL. Current mu is %1 and radius %2. To use %3 with current values relaunch the plugin.").arg(mu->getf()).arg(rad->getf()).arg(pluginName));
            msg.exec();
        }

        int Vers = 0, DLLvers = 0;
        XPLMHostApplicationID HostId;
        XPLMGetVersions(&Vers, &DLLvers, &HostId);

        //Only debug:
        qDebug() << "X-Plane version =" << Vers << ", XPLM.dll = " << DLLvers << ", Host id =" << HostId << ".";
        m_pCurAC = Aircraft::addAircraft(getCurrentAC());

        void *res = XPLMFindSymbol("XPLMCreateFlightLoop");
        if (!res)
            qDebug() << "XPLMCreateFlightLoop not found!";

        res = XPLMFindSymbol("XPLMScheduleFlightLoop");
        if (!res)
            qDebug() << "XPLMScheduleFlightLoop not found!";

    } else if (m_menuReset == iRef) {
        CONSOLE("Reload");
        XPLMReloadPlugins();
    } else if (m_pPerfApp) {
        m_pPerfApp->show();
    }

    showMessage("Ready");
}

bool PerfPlugin::checkGravity()
{
    XPdata *mu = XPdata::ref("sim/physics/earth_mu");
    XPdata *rad = XPdata::ref("sim/physics/earth_radius_m");

    if (mu->getf() != 3.983E+14f)
        return false;
    if (rad->getf() != 6373000.0f)
        return false;

    return true;
}

void PerfPlugin::exportDirectory(QString dir)
{
    //m_pCurAC->exportToDir(dir);
    m_pExportPath = dir;

    foreach(Aircraft *a, Aircraft::AircraftList) {
        a->exportToDir(m_pExportPath);
    }
}

