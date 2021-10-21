#ifndef AIRCRAFT_H
#define AIRCRAFT_H

#include "autopilot.h"

#include <QDateTime>
#include <QFile>

template<typename T>
class AutopilotAxis;
template<typename T>
class Autopilot;

template<typename T = float>
class Aircraft
//class Aircraft : public QObject
{
//    Q_OBJECT

public:
    //Aircraft(Autopilot<T> *ap = nullptr)
    //{ setAutopilot(ap); }
    ~Aircraft()
    { reset(); }

    static Aircraft<T> *addAircraft(QString path);
    static QHash<QByteArray, Aircraft<T>*> AircraftList;

    void reset()
    { if (m_pAutoPilot) { delete m_pAutoPilot; m_pAutoPilot = nullptr; } }

    void setAutopilot(Autopilot<T> *ap)
    { reset(); if (ap) m_pAutoPilot = ap; else createAutopilot(); }

    //bool checkModified();
    //void setCurrent();

    void readXPdata();
    void setXPdata();
    void setGrossMass(T m);
    void resetFuel();
    void setTotalFuel(T m);
    void setMinFuel();

    void setTrafficLoad(T f)
    { m_fTrafficLoad = f; }


private:
    Aircraft(QString path);

    static QByteArray calcHash(QString path);
    static QByteArray calcHash(QFile &f);
    void setPath(QString path);
    QByteArray getHash();
    QByteArray calcHash();
    void createAutopilot();

    Autopilot<T> *m_pAutoPilot = nullptr;
    QDateTime m_cModified;
    QString m_sPath = QString();
    QString m_sName = QString();
    QByteArray m_sHash = QByteArray();

    T m_fCG = 0;
    T m_fCGfwd = 0;
    T m_fCGaft = 0;
    int m_iNumTanks = 0;
    int m_iNumEngines = 0;
    T m_fTrafficLoad = 0;
    T m_fTotalFuel = 0;
    T m_fGrossMass = 0;
    T m_fDOM = 0;
    T m_fMaxFuel = 0;
    T m_fMTOM = 0;
    QList<T> m_cTankFuel;
    QList<T> m_cTankSize;
};



#endif // AIRCRAFT_H
