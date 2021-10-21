#include "aircraft.h"

#include "perfplugin.h"
#include "autopilot.h"

#include <QCryptographicHash>
#include <QFileInfo>

template class Aircraft<float>;
//template class Aircraft<double>;

template<typename T>
QHash<QByteArray, Aircraft<T>*> Aircraft<T>::AircraftList;

//QHash<QByteArray, Aircraft<float>*> Aircraft<float>::AircraftList;

template<typename T>
Aircraft<T> *Aircraft<T>::addAircraft(QString path)
{
    Aircraft<T> *newAC = new Aircraft(path);
    Aircraft<T> *ret = nullptr;

    if (AircraftList.contains(newAC->m_sHash)) {
        ret = AircraftList[newAC->m_sHash];
        delete newAC;
    } else {
        AircraftList[newAC->m_sHash] = newAC;
        ret = newAC;
    }

    ret->readXPdata();

    return ret;
}


template<typename T>
Aircraft<T>::Aircraft(QString path)
{
    setPath(path);

    if (m_sHash.isEmpty())
        qDebug() << "Aircraft file not found!" << m_sPath;

    createAutopilot();
}

template<typename T>
void Aircraft<T>::setPath(QString path)
{
    m_sPath = path;

    QFile f(m_sPath);
    if (f.exists()) {
        QFileInfo fi(f);
        m_sName = fi.completeBaseName();
        m_cModified = fi.lastModified();
        m_sHash = calcHash(f);
    }
}

template<typename T>
inline QByteArray Aircraft<T>::getHash()
{
    return m_sHash;
}

template<typename T>
QByteArray Aircraft<T>::calcHash()
{
    QFile f(m_sPath);

//    if (m_sName.isEmpty())
//        m_sName = QFileInfo(f).completeBaseName();

    if (f.exists())
        return calcHash(f);

    return QByteArray();
}

template<typename T>
QByteArray Aircraft<T>::calcHash(QString path)
{
    QFile f(path);
    return calcHash(f);
    //return calcHash(QFile(path));
}

template<typename T>
QByteArray Aircraft<T>::calcHash(QFile &f)
{
    if (f.open(QFile::ReadOnly)) {
        QCryptographicHash hash(QCryptographicHash::Sha256);

        if (hash.addData(&f)) {
            return hash.result();
        }
    }
    return QByteArray();
}

template<typename T>
void Aircraft<T>::readXPdata()
{
    //m_fCG = PerfPlugin::getCG();
    m_fCGfwd = PerfPlugin::getCGfwd();
    m_fCGaft = PerfPlugin::getCGaft();
    m_iNumTanks = PerfPlugin::getNumTanks();
    //m_iNumEngines = PerfPlugin:: ();
    m_fDOM = PerfPlugin::getDOM();
    m_fMaxFuel = PerfPlugin::getMaxFuel();
    m_fMTOM = PerfPlugin::getMTOM();
    m_fTrafficLoad = PerfPlugin::getTrafficLoad();
    m_fGrossMass = PerfPlugin::getGrossMass();
    m_cTankFuel = PerfPlugin::getTankFuel();
    m_cTankSize = PerfPlugin::getTankSize();
}

template<typename T>
void Aircraft<T>::setXPdata()
{
    PerfPlugin::setCG(m_fCG);
    PerfPlugin::setTrafficLoad(m_fTrafficLoad);
    PerfPlugin::setTankFuel(m_cTankFuel);
}

template<typename T>
void Aircraft<T>::setGrossMass(T m)
{
    T load = m - m_fDOM;

    if (load > m_fMaxFuel) {
        setTotalFuel(m_fMaxFuel);
        setTrafficLoad(load - m_fMaxFuel);
    } else if (load > 0.0) {
        setTrafficLoad(0.0);
        setTotalFuel(load);
    } else {
        setTrafficLoad(0.0);
        setMinFuel();
    }

    m_fGrossMass = m_fDOM + m_fTrafficLoad + m_fTotalFuel;
}

template<typename T>
void Aircraft<T>::resetFuel()
{
    PerfPlugin::setTankFuel(m_cTankFuel);
}

template<typename T>
void Aircraft<T>::setTotalFuel(T m)
{
    for (qsizetype i = 0; i < m_cTankFuel.size(); ++i)
        m_cTankFuel[i] = m_cTankSize.at(i) * m;
    m_fTotalFuel = m;
}

template<typename T>
void Aircraft<T>::setMinFuel()
{
    setTotalFuel(PerfPlugin::getFFtotal());
}


template<typename T>
void Aircraft<T>::createAutopilot()
{
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
    PID<float> *ThetaPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.2f * PerfPlugin::getMaxAlpha(), -PerfPlugin::getMaxAlpha());
    PID<float> *ThrPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.0f, 0.0f);
    //AutopilotAxis<float> *pitch = new AutopilotAxis(getAngVelQ, setPitch, QradPid);
    //AutopilotAxis<float> *yaw = new AutopilotAxis(getAngVelR, setYaw, RradPid);
    //AutopilotAxis<float> *roll = new AutopilotAxis(getAngVelP, setRoll, PradPid);
    AutopilotAxis<float> *pitch = new AutopilotAxis(PerfPlugin::getAngAccQdot, PerfPlugin::setPitch, QradPid);
    AutopilotAxis<float> *yaw = new AutopilotAxis(PerfPlugin::getAngAccRdot, PerfPlugin::setYaw, RradPid);
    AutopilotAxis<float> *roll = new AutopilotAxis(PerfPlugin::getAngAccPdot, PerfPlugin::setRoll, PradPid);
    AutopilotAxis<float> *vpath = new AutopilotAxis(PerfPlugin::getVPath, PerfPlugin::setTestAlpha, ThetaPid);
    AutopilotAxis<float> *throttle = new AutopilotAxis(PerfPlugin::getTotalForceZ, PerfPlugin::setThrottle, ThrPid);
    m_pAutoPilot = new Autopilot<T>();
    //AutopilotAxis<float> *pitchAPalpha = new AutopilotAxis(getAlpha, setPitch, pitchAlpha);
    //this->setTestAlpha
    //PerfPlugin::setTestAlpha

    //m_pControl = ThetaPid; //The one can be controlled with debug

    QradPid->KdFilter = QradKdFilter;
    RradPid->KdFilter = RradKdFilter;
    PradPid->KdFilter = PradKdFilter;
    ThetaPid->KdFilter = ThetaKdFilter;
    ThrPid->KdFilter = ThrKdFilter;
    //pitchAPalpha->setScale(getMaxAlpha() * 2.0f);
    pitch->setScale(5000);
    yaw->setScale(5000);
    roll->setScale(5000);
    throttle->setScale(1);

    m_pAutoPilot->add(pitch);
    m_pAutoPilot->add(yaw);
    m_pAutoPilot->add(roll);
    m_pAutoPilot->add(vpath);
    //m_pAutoPilot->add(throttle);
}
