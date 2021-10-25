#ifndef PERFPLUGIN_H
#define PERFPLUGIN_H

#include <QObject>

#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#define XPLM_BUFFER_SIZE 255
#define XPLM_PATH_BUFFER (XPLM_BUFFER_SIZE * 2)

#include <Physics>

//#include "perfthread.h"
#include "perfanalyzer.h"
#include "testpoint.h"
#include "qvectord3d.h"

class PerfAnalyzer;
class PerfPlugin;
//template<typename T>
//class Autopilot;
//template<typename T>
//class Aircraft;

#include "aircraft.h"
#include "autopilot.h"

enum PerfState {
    off = 0,
    init,
    initAC,
    check,
    stabilize,
    stable,
    skip
};

class PerfPlugin : public QObject
{
    Q_OBJECT
public:
    static PerfPlugin *get();
    ~PerfPlugin();

    // X-Plane API
    float startFlightLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
    float preFlightLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
    float afterFlightLoop(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop, int inCounter, void *inRefcon);
    int pluginStart(char *outName, char *outSig, char *outDesc);
    void pluginStop();
    int pluginEnable();
    void pluginDisable();
    void receivedMessage(XPLMPluginID inFromWho, long inMessage, void *inParam);
    void menuHandler(void *mRef, char *iRef);
    void newPoint(TestPoint * pNewPoint);
    void newPoint(TestPointList cNewPoints);
    void newConfig(Config *pNewCfg);
    void newConfig(Aircraft *pNewAC);
    static TestPointList *getData();
    static float getAccuracy()
    { return m_fAccuracy; }

    static void inc();

    static void setAltitude(Length alt);
    void setCoordinates(Coordinate coord);

    static bool isPaused();
    static void pause();

    Length getAltitude();
    Coordinate getCoordinates();

    void setApLevelFlight();
    void setAutopilot(bool on = true);
    void setApModes(int flags);
    void setBankHold(int mode = 0);
    void setVerticalSpeed(float fmp = 0);
    void setAltSelector(float ft);

    int getApModes();

    void dumpData(QString title = "");

    bool m_bPreDump = true;
    bool m_bAfterDump = true;
    bool m_bDumpDebug = true;
    bool m_bDumpPos = true;
    bool m_bDumpAttitude = true;
    bool m_bDumpLinVel = true;
    bool m_bDumpAngVel = true;
    bool m_bDumpLinAcc = true;
    bool m_bDumpAngAcc = true;
    bool m_bDumpForces = true;
    bool m_bDumpMoments = true;
    bool m_bDumpAeroData = true;
    bool m_bDumpControls = true;
    bool m_bDumpThrottles = true;

    //Environment:

    static Atmosphere getEnvironment();
    static float getRho();
    static float getSigma();
    static float getTat();
    static float getSat();
    static float getPressure();
    static float getC();
    static float getQ();
    static float getG(); //G at current level

    //Position:
    enum Axis : int {
        X = 0x1,
        Y = 0x2,
        Z = 0x4,
        Q = 0x1,
        R = 0x2,
        P = 0x4,
        M = 0x1,
        N = 0x2,
        L = 0x4
    };

    static QVectorD3D setPos(QVectorD3D v);
    //QVectorD3D setOpenGLpos(QVectorD3D v);
    static void setTestAltitude(Length alt);
    static void setTestPosition();


    static QVectorD3D getPos()
    { return QVectorD3D(getLat(), getLon(), getAltm()); }
    static double getLat();
    static double getLon();
    //static double getElev();
    static double getAltm();
    static float getAglm();

    static Length getAlt()
    { return Length(getAltm()); }
    static Length getAgl()
    { return Length(getAglm()); }

    static QVectorD3D getGLpos()
    { return QVectorD3D(getGLposX(), getGLposY(), getGLposZ()); }
    static double getGLposX();
    static double getGLposY();
    static double getGLposZ();

    static void setGLpos(QVectorD3D v, int mask = -1);
    static void setGLposX(double f);
    static void setGLposY(double f);
    static void setGLposZ(double f);

    //Attitude
    //QQuaternion getOpenGLatt();
    //QQuaternion getOpenGLattDeg();
    //QQuaternion getRealAtt();
    static void setGLq(QQuaternion q);
    static void setRealAtt(QQuaternion q);

    static void setAttitude(float pitch = 0.0f, float heading = 0.0f, float roll = 0.0f);
    static void setTestAtt();
    static void setTestAtt(float alpha)
    { m_fAlpha = alpha; return setTestAtt(); }

    static QQuaternion getGLq();
    static float getGLqTheta() //Pitch
    { return getGLq().toEulerAngles().x(); } //Pitch
    static float getGLqPhi() //Roll
    { return getGLq().toEulerAngles().z(); } //Roll
    static float getGLqPsi() //Heading
    { return getGLq().toEulerAngles().y(); } //Heading

    static void setGLqTheta(float p) //Pitch
    { QVector3D v = getGLq().toEulerAngles(); v.setX(p); setGLq(QQuaternion::fromEulerAngles(v)); } //Pitch

    static QQuaternion getGLattDeg()
    { return QQuaternion::fromEulerAngles(getGLtheta(), getGLpsi(), getGLphi()); }
    static float getGLtheta(); //Pitch
    static float getGLphi(); //Roll
    static float getGLpsi(); //Heading

    static QQuaternion getTrueAtt()
    { return QQuaternion::fromEulerAngles(getTrueTheta(), getTruePsi(), getTruePhi()); }
    static float getTrueTheta(); //Pitch
    static float getTruePhi(); //Roll
    static float getTruePsi(); //Heading

    static float getMaxAlpha();
    static float getAlpha();
    static float getBeta();
    static float getVPath();
    static float getHPath();

    static QQuaternion getAlphaQ()
    { return QQuaternion::fromEulerAngles(getAlpha(), 0.0f, 0.0f); }
    static QQuaternion getBetaQ()
    { return QQuaternion::fromEulerAngles(0.0f, getBeta(), 0.0f); }
    static QQuaternion getVPathQ()
    { return QQuaternion::fromEulerAngles(getVPath(), 0.0f, 0.0f); }
    static QQuaternion getHPathQ()
    { return QQuaternion::fromEulerAngles(0.0f, getHPath(), 0.0f); }

    //Velocity
    static QVector3D setOpenGLvel(QVector3D v);
    static QVector3D setAngVel(QVector3D v);
    static void setTAS(Quantity val, bool rotate = true);
    static void setCAS(Quantity val, bool rotate = true);

    static QVector3D getGLvel()
    { return QVector3D(getGLvelX(), getGLvelY(), getGLvelZ()); }
    static float getGLvelX();
    static float getGLvelY();
    static float getGLvelZ();

    static QVector3D getAngVel()
    { return QVector3D(getAngVelQ(), getAngVelR(), getAngVelP()); }
    static float getAngVelQ(); //Pitch
    static float getAngVelR(); //Yaw
    static float getAngVelP(); //Roll

    static QVector3D getAngVelDeg()
    { return QVector3D(getAngVelDegQ(), getAngVelDegR(), getAngVelDegP()); }
    static float getAngVelDegQ(); //Pitch
    static float getAngVelDegR(); //Yaw
    static float getAngVelDegP(); //Roll

    static float getIASkt();
    static float getTASms();
    static float getGSms();
    static float getMachNo();

    static Quantity getIAS(Unit u = Unit("m/s", 1.0));
    static Quantity getCAS(Unit u = Unit("m/s", 1.0));
    static Quantity getTAS(Unit u = Unit("m/s", 1.0));
    static Quantity getGS(Unit u = Unit("m/s", 1.0));

    static double getIASms()
    { return getIAS(Unit("m/s", 1.0)).val(); }
    static double getTASkt()
    { return getTAS(Unit("kt", 1.852/3.6)).val(); }
    static double getGSkt()
    { return getGS(Unit("kt", 1.852/3.6)).val(); }

    static const QVector3D m_cZeroV;

    //Acceleration
    static void freezeAccNrot();

    static QVector3D getOpenGLacc();
    static float getOpenGLaccX();
    static float getOpenGLaccY();
    static float getOpenGLaccZ();
    static float getTestDeceleration();

    static QVector3D setOpenGLacc(QVector3D v);

    static QVector3D getAngAcc()
    { return QVector3D(getAngAccQdot(), getAngAccRdot(), getAngAccPdot()); }
    static float getAngAccQdot(); //Pitch
    static float getAngAccRdot(); //Yaw
    static float getAngAccPdot(); //Roll

    static QVector3D setAngAcc(QVector3D v);

    //Forces
    //QVector3D setForces(QVector3D v);
    //QVector3D getForces();
    static QVector3D getAeroForces()
    { return QVector3D(getAeroForceX(), getAeroForceY(), getAeroForceZ()); }
    static float getAeroForceX();
    static float getAeroForceY();
    static float getAeroForceZ();

    static QVector3D getEngineForces()
    { return QVector3D(getEngineForceX(), getEngineForceY(), getEngineForceZ()); }
    static float getEngineForceX();
    static float getEngineForceY();
    static float getEngineForceZ();

    static QVector3D getGearForces()
    { return QVector3D(getGearForceX(), getGearForceY(), getGearForceZ()); }
    static float getGearForceX();
    static float getGearForceY();
    static float getGearForceZ();

    static QVector3D getGforces()
    { return QVector3D(getGforceX(), getGforceY(), getGforceZ()); }
    static float getGforceX();
    static float getGforceY();
    static float getGforceZ();
    static float getGforceTotal(); //eq to lenght of getGforces vector

    static QVector3D getTotalForces()
    { return QVector3D(getTotalForceX(), getTotalForceY(), getTotalForceZ()); }
    static float getTotalForceX();
    static float getTotalForceY();
    static float getTotalForceZ();

    static QVector3D getFlightPathForces()
    { return QVector3D(getFlightPathForcesSide(), getFlightPathForcesLift(), getFlightPathForcesDrag()); }
    static float getFlightPathForcesSide();
    static float getFlightPathForcesLift();
    static float getFlightPathForcesDrag();

    static float getCL();
    static float getCD();

    static void setAeroForces(QVector3D v);
    static void setEngineForces(QVector3D v);
    static void setGForces(QVector3D v);
    static void setTotalForces(QVector3D v);

    static void zeroForces();

    //Moments
    //QVector3D getMom();

    static QVector3D getPosMoments()
    { return QVector3D(getPosMomentM(), getPosMomentN(), getPosMomentL()); }
    static float getPosMomentM();
    static float getPosMomentN();
    static float getPosMomentL();

    static QVector3D getAeroMoments()
    { return QVector3D(getAeroMomentM(), getAeroMomentN(), getAeroMomentL()); }
    static float getAeroMomentM();
    static float getAeroMomentN();
    static float getAeroMomentL();

    static QVector3D getEngineMoments()
    { return QVector3D(getEngineMomentM(), getEngineMomentN(), getEngineMomentL()); }
    static float getEngineMomentM();
    static float getEngineMomentN();
    static float getEngineMomentL();

    static QVector3D getGearMoments()
    { return QVector3D(getGearMomentM(), getGearMomentN(), getGearMomentL()); }
    static float getGearMomentM();
    static float getGearMomentN();
    static float getGearMomentL();

    static QVector3D getMassMoments()
    { return QVector3D(getMassMomentM(), getMassMomentN(), getMassMomentL()); }
    static float getMassMomentM();
    static float getMassMomentN();
    static float getMassMomentL();

    static QVector3D getTotalMoments()
    { return QVector3D(getTotalMomentM(), getTotalMomentN(), getTotalMomentL()); }
    static float getTotalMomentM();
    static float getTotalMomentN();
    static float getTotalMomentL();

    static void setAeroMoments(QVector3D v);
    static void setEngineMoments(QVector3D v);
    static void setTotalMoments(QVector3D v);

    static void setTestAlpha(float f);

    //Aircraft loaded
    static Aircraft* getAircraft();
    static QString getCurrentAC();
    static void setCurrentAC(Aircraft *ac);
    static void setCurrentAC(QByteArray file);

    //Aircraft controls
    static void setPitch(float f); //-2 ... 2
    static void setRoll(float f);
    static void setYaw(float f);
    static void setYokePitch(float f); //-1 ... 1
    static void setYokeRoll(float f);
    static void setYokeYaw(float f);
    static void setTrimPitch(float f); //-1 ... 1
    static void setTrimRoll(float f);
    static void setTrimYaw(float f);

    static float getYokePitch(); //-1 ... 1
    static float getYokeRoll(); //-1 ... 1
    static float getYokeYaw(); //-1 ... 1
    static float getTrimPitch(); //-1 ... 1
    static float getTrimRoll(); //-1 ... 1
    static float getTrimYaw(); //-1 ... 1

    static void setControlsP(QList<float> v); //get first and then modify
    static void setControlsR(QList<float> v);
    static void setControlsY(QList<float> v);
    static QList<float> getControlsP(); //get first and then modify
    static QList<float> getControlsR();
    static QList<float> getControlsY();

    //Engines
    static int getNumEngines(); //sim/aircraft/engine/acf_num_engines
    static float getMaxThrottle(); //sim/aircraft/engine/acf_throtmax_FWD
    static float getMinThrottle(); //sim/aircraft/engine/acf_throtmax_REV
    static QList<float> getThrottle(); // sim/flightmodel/engine/
    static QList<float> getFF(); // sim/flightmodel/engine/ENGN_FF_
    static QList<float> getN1(); // sim/flightmodel/engine/ENGN_N1_
    static QList<float> getN2(); // sim/flightmodel/engine/ENGN_N2_
    static QList<float> getEPR(); // sim/flightmodel/engine/ENGN_EPR
    static QList<float> getThrust(); // sim/flightmodel/engine/POINT_thrust

    static float getFFtotal(); // sim/flightmodel/engine/ENGN_FF_
    static float getN1mean(); // sim/flightmodel/engine/ENGN_N1_
    static float getN2mean(); // sim/flightmodel/engine/ENGN_N2_
    static float getEPRmean(); // sim/flightmodel/engine/ENGN_EPR
    static float getThrustTotal(); // sim/flightmodel/engine/POINT_thrust

    static void setThrottle(float pos);

    //Aircraft weight
    static float getCG(); // sim/flightmodel/misc/cgz_ref_to_default //meters 0.0f is center
    static float getCGfwd(); // sim/aircraft/overflow/acf_cgZ_fwd //meters
    static float getCGaft(); // sim/aircraft/overflow/acf_cgZ_aft //meters
    static float getGrossMass(); // sim/flightmodel/weight/m_total
    static float getTrafficLoad(); // sim/flightmodel/weight/m_fixed //Payload
    static float getTotalFuel(); // sim/flightmodel/weight/m_fuel_total
    static int   getNumTanks(); //sim/aircraft/overflow/acf_num_tanks
    static float getTankFuel(unsigned int n); // sim/flightmodel/weight/m_fuel[9]
    static QList<float> getTankFuel(); // sim/flightmodel/weight/m_fuel[9]
    static QList<float> getTankSize(); //sim/aircraft/overflow/acf_tank_rat // ratio of total tank capacity (Sum should be 1)
    static float getDOM(); //sim/aircraft/weight/acf_m_empty // DOM = BEM(includes oil etc) + crew + catering
    static float getMaxFuel(); // sim/aircraft/weight/acf_m_fuel_tot
    static float getMTOM(); //sim/aircraft/weight/acf_m_max

    static void setCG(float f); // sim/flightmodel/misc/cgz_ref_to_default
    static void setGrossMass(float f); // sim/flightmodel/weight/m_total (read _only)
    static void setTrafficLoad(float f); // sim/flightmodel/weight/m_fixed
    static void setTotalFuel(float f); // sim/flightmodel/weight/m_fuel[9]
    static void setTankFuel(int n, float f); // sim/flightmodel/weight/m_fuel[9]
    static void setTankFuel(QList<float> f); // sim/flightmodel/weight/m_fuel[9]

    //Aircraft speeds
    static Quantity getVMO(); // sim/aircraft/view/acf_Vne
    static float getMMO(); // sim/aircraft/view/acf_Mmo

    //Aircraft config
    static bool getGear();
    static float getFlaps();

    static void setGear(bool pos);
    static void setFlaps(float pos);

    static QString getExportDir()
    { return m_pExportPath; }

public slots:
    void controlPara1(double v);
    void controlPara2(double v);
    void controlPara3(double v);
    void controlPara4(double v);
    void controlPara5(double v);
    void controlButton();
    static void showMessage(QString msg);
    static void exportDirectory(QString dir);

signals:
    void inData();
    void outData();

protected:
    void createAutoPilot();

    QQuaternion setAlpha(QQuaternion q);
    float setAlpha(float a);
    QQuaternion setBeta(QQuaternion q);
    float setBeta(float b);
    //QQuaternion setVPath(QQuaternion q);
    //QQuaternion setHPath(QQuaternion q);
    //QVector3D setMom(QVector3D v);
    Quantity setIAS(Quantity q);
    //Quantity setGS(Quantity q);
    //Quantity setTAS(Quantity q);
    //Quantity setAlt(Quantity q);
    //Quantity setAgl(Quantity q);
    void setControls(bool set = true);
    void setISAweather(bool set = true);

    //static void setPitchTrim(float val);
    //static void setRollTrim(float val);
    //static void setYawTrim(float val);

private slots:
//    void registerFL();

private:
    explicit PerfPlugin(QObject *parent = nullptr);

    void hardReset();

    static PerfPlugin* m_pPlugin;
    static PerfAnalyzer *m_pPerfApp;
    static Aircraft *m_pCurAC;
public:
    static const QByteArray pluginOrg;
    static const QByteArray pluginName;
    static const QByteArray m_menuStart;
    static const QByteArray m_menuReset;
private:
    static QList<Config*> m_cCfgQueue;
    static TestPointList m_cQueue;
    static TestPointList m_cReady;
    static QVectorD3D m_cTestGLpos;
    static QVectorD3D m_cSavedpos;
    static QString m_sQtPluginPath;
    static QString m_pExportPath;
    static double m_dMinAlt;

    static float m_fAlpha;
    static int m_iNumSamples;
    static float m_fAccuracy;

    void *m_pControl = nullptr;
    bool reset = false;

    void createCallBacks();
    void destroyCallBacks();
//    void unregisterFL();
    void scheduleFL();
    void setConditions(bool set = true); //Apply overrides save pos, att, and speeds
    void setTestPos(bool set = true);
    void saveVelAttAng(bool save = true);
    void initializeTestpoint(bool set = true); //Set testspeed etc
    void initTestpoint(TestPoint *m_pCurrent); //Set gear flaps etc
    bool checkConditions();
    bool processCurrent();
    void freeze(bool set = true);
    void zeroAcceleration();
    void sortQueue();
    bool checkGravity();

    bool compareCfg(const Config *a, const Config *b) const;

    Autopilot<float> *m_pAutoPilot = nullptr;

    XPLMFlightLoopID preLoopId = nullptr;
    XPLMFlightLoopID afterLoopId = nullptr;

    bool m_bFltRunning = false;
    bool m_bPreRunning = false;
    bool m_bAftRunning = false;
    bool m_bTestConditions = false;

    PerfState m_eComState = PerfState::off;
    PerfState m_eCurState = PerfState::off;
    TestPoint *m_pCurrent = nullptr;
    Config *m_pCurrentCfg = nullptr;
    Profile *m_pProfile = nullptr;
};

#endif // PERFPLUGIN_H
