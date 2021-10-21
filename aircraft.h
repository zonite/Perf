#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include "autopilot.h"

#include <QDateTime>
#include <QFile>

//template<typename T>
//class AutopilotAxis;
//template<typename T>
//class Autopilot;

#include "Physics"

class Aircraft;
class Config;
//class Profile;
//class TestPoint;

#include "testpoint.h"

class Aircraft : public QObject
{
    Q_OBJECT

public:
    //Aircraft(Autopilot<T> *ap = nullptr)
    //{ setAutopilot(ap); }
    ~Aircraft()
    { reset(); }

    enum Controls {
        neutral = 0,
        fullUp,
        fullDown,
        fullLeft,
        fullRight,
        fullLeftUp,
        fullLeftDown,
        fullRightUp,
        fullRightDown,
        fullLeftRudder,
        fullRightRudder,
        fullLeftDownRudder,
        fullRightUpRudder
    };

    static Aircraft *addAircraft(QString path);
    static QHash<QByteArray, Aircraft*> AircraftList;

    bool set();
    void reset()
    { if (m_pAutoPilot) { delete m_pAutoPilot; m_pAutoPilot = nullptr; } }
    bool isStable();

    bool check(bool set = true);

    void setAutopilot(Autopilot<float> *ap)
    { reset(); if (ap) m_pAutoPilot = ap; else createAutopilot(); }

    Config *addConfig(float flaps, bool gear, float mass, float cg);
    Config *addConfig() { readXPdata(); return addConfig(0.0f, 0.0f, m_fGrossMass, m_fCG); }

    void initTest();
    TestPoint *curTP();
    //TestPoint *nextTP();
    //TestPoint *prevTP();
    TestPoint *checkTP(TestPoint *tp);
    void initTP(TestPoint *tp);

    void captureData();
    void recordThrust();

    void setMaxAlt(Length v) { m_cMaxAlt = v; }
    void setMinAlt(Length v) { m_cMinAlt = v; }
    void setAltInc(Length v) { m_cAltInc = v; }
    void setSpeedInc(Quantity v) { m_cSpeedInc = v; }

    const QList<float> *getTestThrottles() const { return &m_cTestThrottles; }
    QList<Config *> getCfgs() { return m_cTestConfigs.values(); }
    Length getMaxAlt() { return m_cMaxAlt; }
    Length getMinAlt() { return m_cMinAlt; }
    Length getAltInc() { return m_cAltInc; }
    Quantity getSpeedInc() { return m_cSpeedInc; }
    Quantity getVMO();
    float getMMO() { return m_fMMO; }

    //bool checkModified();
    //void setCurrent();

    void setThrottle(float f) { if (f > m_fMaxThrottle) m_fThrottle = m_fMaxThrottle; else m_fThrottle = f; setThrottle(); }
    void setThrottle(); //Set Throttle in PerfPlugin
    float getThrottle()
    { return m_fThrottle; }
    float getMaxThrust()
    { return m_fMaxThrust; }
    float getMinThrust()
    { return m_fMinThrust; }

    void setPitch(float f) { m_fPitch = f; setPitch(); }
    void setPitch(); //Set Pitch in PerfPlugin
    void setRoll(float f) { m_fRoll = f; setRoll(); }
    void setRoll(); //Set Roll in PerfPlugin
    void setYaw(float f) { m_fYaw = f; setYaw(); }
    void setYaw(); //Set Yaw in PerfPlugin
    void setAlpha(float f) { m_fAlpha = f; setAlpha(); }
    void setAlpha(); //Set Yaw in PerfPlugin

    void setControlsP(float v);
    void setControlsR(float v);
    void setControlsY(float v);
    QList<float> toList(float v, QList<float> *l);

    QString path() { return m_sPath; }
    void readXPdata();
    void setXPdata();
    void setGrossMass(float m);
    void resetFuel();
    void setTotalFuel(float m);
    void setMinFuel();
    void setFlaps(float v) { m_fFlaps = v; }
    void setGear(bool v) { m_bGear = v; }
    void setCG(float v) { m_fCG = v; }

    void setTrafficLoad(float f)
    { m_fTrafficLoad = f; }

    void exportToDir(QString dir);

public slots:

signals:

private:
    Aircraft(QString path);

    static QByteArray calcHash(QString path);
    static QByteArray calcHash(QFile &f);
    void setPath(QString path);
    QByteArray getHash();
    QByteArray calcHash();
    void createAutopilot();

    Autopilot<float> *m_pAutoPilot = nullptr;
    QDateTime m_cModified;
    QString m_sPath = QString();
    QString m_sName = QString();
    QByteArray m_sHash = QByteArray();

    void data(QTextStream &p);

    QMap<QString, Config*> m_cTestConfigs;
    QList<float> m_cTestThrottles = { 0.0f, 1.0f, -1.0f};

    TestPoint *m_pCurTP = nullptr;
    Config *m_pCurCfg = nullptr;

    Length m_cMaxAlt = Length(Unit("ft", 0.3048f), 40000.0f);
    Length m_cMinAlt = Length(Unit("ft", 0.3048f), 100.0f);
    Length m_cAltInc = Length(Unit("ft", 0.3048f), 1000.0f);
    Quantity m_cSpeedInc = Quantity(Unit("kt", 1.852f/3.6f), 10.0f);

    Quantity m_fVMO = Quantity(Unit("kt", 1.852f/3.6f), 0.0f);
    float m_fMMO = 0.0f;
    float m_fFlaps = 0.0f;
    bool m_bGear = false;
    float m_fThrottle = 0.0f;
    float m_fPitch = 0.0f;
    float m_fRoll = 0.0f;
    float m_fYaw = 0.0f;
    float m_fAlpha = 0.0f;
    bool m_bStable = false;
    bool m_pChecked = false;
    enum Controls yoke = neutral;
    float m_fMaxAlpha = 0.0f;
    float m_fMinAlpha = 0.0f;
    float m_fMaxThrottle = 1.0f;
    float m_fMinThrottle = 0.0f;
    float m_fMaxThrust = -1.0f;
    float m_fMinThrust = -1.0f;

    float m_fCG = 0.0f;
    float m_fCGfwd = 0.0f;
    float m_fCGaft = 0.0f;
    int m_iNumTanks = 0;
    int m_iNumEngines = 0;
    float m_fTrafficLoad = 0.0f;
    float m_fTotalFuel = 0.0f;
    float m_fGrossMass = 0.0f;
    float m_fDOM = 0.0f;
    float m_fMaxFuel = 0.0f;
    float m_fMTOM = 0.0f;
    QList<float> m_cTankFuel;
    QList<float> m_cTankSize;

    static QList<float> m_cElevUp;
    static QList<float> m_cElevDown;
    static QList<float> m_cAileLeft;
    static QList<float> m_cAileRight;
    static QList<float> m_cRuddLeft;
    static QList<float> m_cRuddRight;
};

class Config
{
public:
    Config(Aircraft *ac);
    ~Config();
    void addProfile(float throttle);

    Aircraft *getAC() { return m_pAC; }

    TestPoint *initTest();
    void runTest();

    bool set();
    bool isStable()
    { return m_pAC->isStable(); }

    Profile *getCurProfile()
    { return m_pCurProfile; }
    Profile *getNextProfile();
    TestPoint* checkTP();
    TestPoint* getTestPoint()
    { return m_pCurTP; }
    TestPoint* nextTestPoint();
    void captureData();
    QMap<float, Profile*> getProfiles()
    { return m_pProfile; }

    void data(QTextStream &p);

    float getGear() { return m_bGear; }
    float getFlaps() { return m_fFlaps; }
    float getGrossMass() { return m_fGrossMass; }
    float getCG() { return m_fCG; }

    float m_fFlaps = 0.0f;
    bool m_bGear = false;
    float m_fGrossMass = 0.0f;
    float m_fCG = 0.0f;

private:
    qsizetype m_iIterator = 0;
    Profile *m_pCurProfile = nullptr;
    Aircraft *m_pAC = nullptr;
    TestPoint *m_pCurTP = nullptr;
    QList<float> m_cThrottles;
    QMap<float, Profile*> m_pProfile; // power setting + profile (contains the same throttle as profile)
};

#endif // AIRCRAFT_H
