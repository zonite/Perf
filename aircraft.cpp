#include "aircraft.h"

#include "perfplugin.h"

#include <QCryptographicHash>
#include <QFileInfo>
#include <QDir>

//template class Aircraft<float>;
//template class Aircraft<double>;

QList<float> Aircraft::m_cElevUp;
QList<float> Aircraft::m_cElevDown;
QList<float> Aircraft::m_cAileLeft;
QList<float> Aircraft::m_cAileRight;
QList<float> Aircraft::m_cRuddLeft;
QList<float> Aircraft::m_cRuddRight;

QHash<QByteArray, Aircraft*> Aircraft::AircraftList;

//QHash<QByteArray, Aircraft<float>*> Aircraft<float>::AircraftList;

Aircraft *Aircraft::addAircraft(QString path)
{
    Aircraft *newAC = new Aircraft(path);
    Aircraft *ret = nullptr;

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

Config *Aircraft::addConfig(float flaps, bool gear, float mass, float cg)
{
    QString key = QString("%1:%2:%3:%4").arg(mass).arg(cg).arg(flaps).arg(gear);

    if (!m_cTestConfigs.contains(key)) {
        Config *cfg = new Config(this);
        cfg->m_fFlaps = flaps;
        cfg->m_bGear = gear;
        cfg->m_fGrossMass = mass;
        cfg->m_fCG = cg;
        m_cTestConfigs[key] = cfg;

        return cfg;
    }

    return m_cTestConfigs[key];
}


Aircraft::Aircraft(QString path):
    m_cMaxAlt(Unit("ft", 0.3048f)),
    m_cMinAlt(Unit("ft", 0.3048f)),
    m_cAltInc(Unit("ft", 0.3048f)),
    m_cSpeedInc(Unit("kt", 1.852f/3.6f))
{
    setPath(path);

    if (m_sHash.isEmpty())
        qDebug() << "Aircraft file not found!" << m_sPath;
}

void Aircraft::setPath(QString path)
{
    m_sPath = path;

    QFile f(m_sPath);
    if (f.exists()) {
        QFileInfo fi(f);
        m_sName = fi.completeBaseName();
        m_cModified = fi.lastModified();
        m_sHash = calcHash(f);
    }

    qDebug() << "Read Aircraft-file" << m_sPath << m_sHash.toHex();
}

inline QByteArray Aircraft::getHash()
{
    return m_sHash;
}

QByteArray Aircraft::calcHash()
{
    QFile f(m_sPath);

//    if (m_sName.isEmpty())
//        m_sName = QFileInfo(f).completeBaseName();

    if (f.exists())
        return calcHash(f);

    return QByteArray();
}

QByteArray Aircraft::calcHash(QString path)
{
    QFile f(path);
    return calcHash(f);
    //return calcHash(QFile(path));
}

QByteArray Aircraft::calcHash(QFile &f)
{
    if (f.open(QFile::ReadOnly)) {
        QCryptographicHash hash(QCryptographicHash::Sha256);

        if (hash.addData(&f)) {
            return hash.result();
        }
    }
    return QByteArray();
}

void Aircraft::readXPdata()
{
    PerfPlugin::inc();

    m_fVMO = PerfPlugin::getVMO();
    m_fMMO = PerfPlugin::getMMO();
    m_fFlaps = PerfPlugin::getFlaps();
    m_bGear = PerfPlugin::getGear();

    m_fCG = PerfPlugin::getCG();
    m_fCGfwd = PerfPlugin::getCGfwd();
    m_fCGaft = PerfPlugin::getCGaft();
    m_iNumTanks = PerfPlugin::getNumTanks();
    m_iNumEngines = PerfPlugin::getNumEngines();
    m_fDOM = PerfPlugin::getDOM();
    m_fMaxFuel = PerfPlugin::getMaxFuel();
    m_fMTOM = PerfPlugin::getMTOM();
    m_fTrafficLoad = PerfPlugin::getTrafficLoad();
    m_fGrossMass = PerfPlugin::getGrossMass();
    m_cTankFuel = PerfPlugin::getTankFuel();
    m_cTankSize = PerfPlugin::getTankSize();
    m_fMaxThrottle = PerfPlugin::getMaxThrottle();
    m_fMinThrottle = PerfPlugin::getMinThrottle();

    if (m_fMaxThrottle != 1.0f) {
        m_cTestThrottles.insert(1, m_fMaxThrottle);
    }

    createAutopilot();

    qDebug() << "Read XP data!!!";
    qDebug() << "VMO" << m_fVMO.raw() << "Gross Mass" << m_fGrossMass << "GC" << m_fCG;
}

void Aircraft::setXPdata()
{
    PerfPlugin::setFlaps(m_fFlaps);
    PerfPlugin::setGear(m_bGear);
    PerfPlugin::setTrafficLoad(m_fTrafficLoad);
    resetFuel();
}

bool Aircraft::check(bool set)
{
    if (m_pChecked)
        return true;

    if (set) {
        switch(yoke) {
        case neutral:
            PerfPlugin::setYokePitch(0.0f);
            PerfPlugin::setYokeRoll(0.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullUp:
            PerfPlugin::setYokePitch(1.0f);
            PerfPlugin::setYokeRoll(0.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullDown:
            PerfPlugin::setYokePitch(-1.0f);
            PerfPlugin::setYokeRoll(0.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullLeft:
            PerfPlugin::setYokePitch(0.0f);
            PerfPlugin::setYokeRoll(-1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullRight:
            PerfPlugin::setYokePitch(0.0f);
            PerfPlugin::setYokeRoll(1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullLeftUp:
            PerfPlugin::setYokePitch(1.0f);
            PerfPlugin::setYokeRoll(-1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullLeftDown:
            PerfPlugin::setYokePitch(-1.0f);
            PerfPlugin::setYokeRoll(-1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullRightUp:
            PerfPlugin::setYokePitch(1.0f);
            PerfPlugin::setYokeRoll(1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullRightDown:
            PerfPlugin::setYokePitch(-1.0f);
            PerfPlugin::setYokeRoll(1.0f);
            PerfPlugin::setYokeYaw(0.0f);
            break;
        case fullLeftRudder:
            PerfPlugin::setYokePitch(0.0f);
            PerfPlugin::setYokeRoll(0.0f);
            PerfPlugin::setYokeYaw(-1.0f);
            break;
        case fullRightRudder:
            PerfPlugin::setYokePitch(0.0f);
            PerfPlugin::setYokeRoll(0.0f);
            PerfPlugin::setYokeYaw(1.0f);
            break;
        case fullLeftDownRudder:
            PerfPlugin::setYokePitch(-1.0f);
            PerfPlugin::setYokeRoll(-1.0f);
            PerfPlugin::setYokeYaw(-1.0f);
            break;
        case fullRightUpRudder:
            PerfPlugin::setYokePitch(1.0f);
            PerfPlugin::setYokeRoll(1.0f);
            PerfPlugin::setYokeYaw(1.0f);
            break;
        }
    } else {
        switch(yoke) {
        case neutral:
            yoke = fullUp;
            break;
        case fullUp:
            m_cElevUp = PerfPlugin::getControlsP();
            yoke = fullDown;
            break;
        case fullDown:
            m_cElevDown = PerfPlugin::getControlsP();
            yoke = fullLeft;
            break;
        case fullLeft:
            m_cAileLeft = PerfPlugin::getControlsR();
            yoke = fullRight;
            break;
        case fullRight:
            m_cAileRight = PerfPlugin::getControlsR();
            yoke = fullLeftRudder;
            break;
        case fullLeftUp:
            yoke = fullLeftDown;
            break;
        case fullLeftDown:
            yoke = fullRightUp;
            break;
        case fullRightUp:
            yoke = fullRightDown;
            break;
        case fullRightDown:
            yoke = fullLeftRudder;
            break;
        case fullLeftRudder:
            m_cRuddLeft = PerfPlugin::getControlsY();
            yoke = fullRightRudder;
            break;
        case fullRightRudder:
            m_cRuddRight = PerfPlugin::getControlsY();
            //yoke = fullLeftDownRudder;
            yoke = neutral;
            m_pChecked = true;
            break;
        case fullLeftDownRudder:
            yoke = fullRightUpRudder;
            break;
        case fullRightUpRudder:
            yoke = neutral;
            m_pChecked = true;
            //PerfPlugin::pause();
            break;
        }
    }
    return false;
}

void Aircraft::setControlsP(float v)
{
    static QList<float> *l = nullptr;

    qDebug() << "Set Controls Pitch raw" << v;

    if (v < 0.0f) {
        if (v < -1.0f)
            v = -1.0f;
        l = &m_cElevDown;
    } else {
        if (v > 1.0f)
            v = 1.0f;
        l = &m_cElevUp;
    }

    PerfPlugin::setControlsP(toList(v, l));
}

void Aircraft::setControlsR(float v)
{
    static QList<float> *l = nullptr;

    if (v < 0.0f) {
        if (v < -1.0f)
            v = -1.0f;
        l = &m_cAileLeft;
    } else {
        if (v > 1.0f)
            v = 1.0f;
        l = &m_cAileRight;
    }

    PerfPlugin::setControlsP(toList(v, l));
}

void Aircraft::setControlsY(float v)
{
    static QList<float> *l = nullptr;

    if (v < 0.0f) {
        if (v < -1.0f)
            v = -1.0f;
        l = &m_cRuddLeft;
    } else {
        if (v > 1.0f)
            v = 1.0f;
        l = &m_cRuddRight;
    }

    PerfPlugin::setControlsP(toList(v, l));
}

QList<float> Aircraft::toList(float v, QList<float> *l)
{
    QList<float> pos(l->size());

    for(qsizetype i = 0; i < l->size(); ++i) {
        pos[i] = l->at(i) * v;
    }

    return pos;
}

TestPoint *Aircraft::checkTP(TestPoint *tp) //For checking various limits to not overstress AC
{
    if (!tp)
        return nullptr;

    Quantity VMO = Aerodynamics::machToCas(m_fMMO, PerfPlugin::getEnvironment());
    //float VMO = Aerodynamics::machToCas(m_fMMO, Atmosphere()).raw();
    Quantity max = ( VMO > m_fVMO ? m_fVMO : VMO ) * 1.1f; //Allow 10% overspeed.
    float curAlt = tp->alt.raw();

    //qDebug() << "m_fVMO" << m_fVMO.raw() << "VMO" << VMO.raw() << "MMO" << m_fMMO << "max" << max.raw() << (m_fVMO * 1.1f).raw() << (VMO * 1.1f).raw();

    while (tp && tp->alt.raw() == curAlt) {
        //qDebug() << "CurTP" << (void *)tp << "alt" << tp->alt.raw() << "curAlt" << curAlt << "Speed & max" << tp->speed.raw() << max.raw();
        if (tp->state == completed) {
            tp = tp->next();
            continue;
        }
        if (tp->next() && tp->next()->state == low_speed && tp->speed < tp->next()->speed) {
            tp->state = low_speed;
            tp = tp->prev();
            continue;
        }
        if (tp->speed <= max) {
            return tp;
        }
        tp->state = overspd;
        tp = tp->prev();
    }

    return tp;
}

void Aircraft::initTP(TestPoint *tp) //Init a valid checked testpoint to run
{
    //Q_ASSERT(tp);
    if (tp->next() && tp->next()->state == completed) {
        if (tp->next()->alt.raw() == tp->alt.raw())
            tp->clone(tp->next());
        //else
            //tp->clone(tp->getProfile());
    }

    if (tp->type == fixedPower)
        setThrottle(tp->throttle);
    setPitch(tp->pitchTrim);
    setRoll(tp->rollTrim);
    setYaw(tp->yawTrim);
    //PerfPlugin::setAttitude(tp->alpha, 0.0f, 0.0f);
    //PerfPlugin::setTestAtt(tp->alpha);
    setAlpha(tp->alpha);
    //PerfPlugin::setTestAlpha(tp->alpha);
    PerfPlugin::setTestAltitude(tp->alt);
    //PerfPlugin::setTestPosition();

    m_pCurTP = tp;
    m_pCurTP->state = intest;
}

TestPoint *Aircraft::curTP()
{
    return m_pCurTP;
}

//TestPoint *Aircraft::nextTP()
//TestPoint *Aircraft::prevTP();


Quantity Aircraft::getVMO()
{
    return m_fVMO;
    //return Quantity(Unit("kt", 1.852f/3.6f), m_fVMO);
}

void Aircraft::setThrottle()
{
    /*
    if (m_fThrottle < 0) {
        //autothrottle
    } else {
        PerfPlugin::setThrottle(m_fThrottle);
    }
    */
    PerfPlugin::setThrottle(m_fThrottle);
}

void Aircraft::setPitch() //Set Pitch in PerfPlugin
{
    PerfPlugin::setPitch(m_fPitch);
}

void Aircraft::setRoll() //Set Roll in PerfPlugin
{
    PerfPlugin::setRoll(m_fRoll);
}

void Aircraft::setYaw() //Set Yaw in PerfPlugin
{
    PerfPlugin::setYaw(m_fYaw);
}

void Aircraft::setAlpha() //Set Yaw in PerfPlugin
{
    PerfPlugin::setTestAlpha(m_fAlpha);
}

void Aircraft::setGrossMass(float m)
{
    float load = m - m_fDOM;

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

void Aircraft::resetFuel()
{
    PerfPlugin::setTankFuel(m_cTankFuel);
}

void Aircraft::setTotalFuel(float m)
{
    for (qsizetype i = 0; i < m_cTankFuel.size(); ++i)
        m_cTankFuel[i] = m_cTankSize.at(i) * m;
    m_fTotalFuel = m;
}


void Aircraft::setMinFuel()
{
    setTotalFuel(PerfPlugin::getFFtotal());
}

bool Aircraft::set()
{
    //static bool bThr, bPitch, bRoll, bYaw, bAlpha, bAcc;
    //static float angacc = 0.0f;
    //float cur = PerfPlugin::getAngAccQdot();
    m_bStable = m_pAutoPilot->set();

    if (m_fAlpha >= m_fMaxAlpha && PerfPlugin::getVPath() < 0.0f)
        m_pCurTP->state = low_speed;

    if (m_bStable && m_pCurTP->state == intest)
        m_pCurTP->state = completed;

    if (m_bStable)
        return m_bStable;

    if (m_pCurTP->count > 1000)
        m_pCurTP->state = unstable;

    return m_pCurTP->state != intest;
    //m_bStable = ;
    //angacc = cur;
}

bool Aircraft::isStable()
{
    return m_bStable;
}

void Aircraft::recordThrust()
{
    float thrust = PerfPlugin::getThrustTotal();

    if (m_fMaxThrust < thrust)
        m_fMaxThrust = thrust;
    if (m_fMinThrust > thrust || m_fMinThrust < 0.0f)
        m_fMinThrust = thrust;

}

void Aircraft::createAutopilot()
{
    if (m_pAutoPilot) {
        delete m_pAutoPilot;
        m_pAutoPilot = nullptr;
    }

    //Filter<float> *QradKdFilter = new EMA(3.0f, 99.0f);
    //Filter<float> *RradKdFilter = new EMA(3.0f, 99.0f);
    //Filter<float> *PradKdFilter = new EMA(3.0f, 99.0f);
    //Filter<float> *ThetaKdFilter = new EMA(3.0f, 99.0f);
    //curTPFilter<float> *ThrKdFilter = new EMA(3.0f, 99.0f);
    //PID<float> *pitchAlpha = new PoM(0.0f, 0.12f, 0.05f, 0.05f, 0.1f, 2.0f, -2.0f);
    //PID<float> *pitchAlpha = new PoM(0.0f, 1.0f, 1.0f, 0.06f, 0.1f, 2.0f, -2.0f);
    //PID<float> *pitchAlpha = new PoM(0.0f, 1.0f, 1.0f, 1.0f, 1.0f, 2.0f, -2.0f);
    m_fMaxAlpha = 1.2f * PerfPlugin::getMaxAlpha();
    m_fMinAlpha = -PerfPlugin::getMaxAlpha();

    //Filter<float> *QradInFilter = new EMA(10.0f, 99.0f);
    //Filter<float> *QradOutFilter = new EMA(10.0f, 99.0f);

    Control<float> *QradPid = new MNY2(0.0f, 2.0f, -2.0f);
    Control<float> *RradPid = new MNY2(0.0f, 2.0f, -2.0f);
    Control<float> *PradPid = new MNY2(0.0f, 2.0f, -2.0f);
    Control<float> *ThetaPid = new MNY2(0.0f, m_fMaxAlpha, m_fMinAlpha);
    Control<float> *ThrPid = new MNY2(0.0f, m_fMaxThrottle, 0.0f);
    //PID<float> *QradPid = new PoM(0.0f, 0.25f, 2.0f, 0.005f, 0.1f, 2.0f, -2.0f);
    //PID<float> *RradPid = new PoM(0.0f, 0.25f, 2.0f, 0.005f, 0.1f, 1.0f, -1.0f);
    //PID<float> *PradPid = new PoM(0.0f, 0.25f, 2.0f, 0.005f, 0.1f, 1.0f, -1.0f);
    //PID<float> *ThetaPid = new PoM(0.0f, 0.25f, 2.0f, 0.005f, 0.1f, 1.2f * PerfPlugin::getMaxAlpha(), -PerfPlugin::getMaxAlpha());
    //PID<float> *ThetaPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.2f * PerfPlugin::getMaxAlpha(), -PerfPlugin::getMaxAlpha());
    //PID<float> *ThrPid = new PoM(0.0f, 0.8f, 0.8f, 0.8f, 1.0f, 1.0f, 0.0f);
    //AutopilotAxis<float> *pitch = new AutopilotAxis(getAngVelQ, setPitch, QradPid);
    //AutopilotAxis<float> *yaw = new AutopilotAxis(getAngVelR, setYaw, RradPid);
    //AutopilotAxis<float> *roll = new AutopilotAxis(getAngVelP, setRoll, PradPid);
    AutopilotAxis<float> *pitch = new AutopilotAxis(0.0001f, PerfPlugin::getAngAccQdot, this, &Aircraft::setPitch, QradPid, new Amean(3));
    AutopilotAxis<float> *yaw = new AutopilotAxis(0.001f, PerfPlugin::getAngAccRdot, this, &Aircraft::setYaw, RradPid, new Amean(3));
    AutopilotAxis<float> *roll = new AutopilotAxis(0.001f, PerfPlugin::getAngAccPdot, this, &Aircraft::setRoll, PradPid, new Amean(3));
    AutopilotAxis<float> *vpath = new AutopilotAxis(0.001f, PerfPlugin::getVPath, this, &Aircraft::setAlpha, ThetaPid, new Amean(3));
    AutopilotAxis<float> *throttle = new AutothrottleAxis(0.001f, PerfPlugin::getTestDeceleration, this, &Aircraft::setThrottle, ThrPid);
    //AutopilotAxis<float> *throttle = new AutothrottleAxis(0.001f, PerfPlugin::getTotalForceZ, this, &Aircraft::setThrottle, ThrPid);
    //AutopilotAxis<float> *pitch = new AutopilotAxis(PerfPlugin::getAngAccQdot, PerfPlugin::setPitch, QradPid);
    //AutopilotAxis<float> *yaw = new AutopilotAxis(PerfPlugin::getAngAccRdot, PerfPlugin::setYaw, RradPid);
    //AutopilotAxis<float> *roll = new AutopilotAxis(PerfPlugin::getAngAccPdot, PerfPlugin::setRoll, PradPid);
    //AutopilotAxis<float> *vpath = new AutopilotAxis(0.001f, PerfPlugin::getVPath, PerfPlugin::setTestAlpha, ThetaPid);
    //AutopilotAxis<float> *throttle = new AutopilotAxis(PerfPlugin::getTotalForceZ, PerfPlugin::setThrottle, ThrPid);
    m_pAutoPilot = new Autopilot<float>();
    //AutopilotAxis<float> *pitchAPalpha = new AutopilotAxis(getAlpha, setPitch, pitchAlpha);
    //this->setTestAlpha
    //PerfPlugin::setTestAlpha

    //m_pControl = ThetaPid; //The one can be controlled with debug

    //((MNY2<float>*)QradPid)->setInFilter(QradInFilter);
    //((MNY2<float>*)QradPid)->setOutFilter(QradOutFilter);


    //RradPid->KdFilter = RradKdFilter;
    //PradPid->KdFilter = PradKdFilter;
    //ThetaPid->KdFilter = ThetaKdFilter;
    //ThrPid->KdFilter = ThrKdFilter;
    //pitchAPalpha->setScale(getMaxAlpha() * 2.0f);
    //pitch->setScale(74);
    //yaw->setScale(110);
    //roll->setScale(5000);
    //throttle->setScale(1);

    m_pAutoPilot->add(pitch);
    m_pAutoPilot->add(yaw);
    m_pAutoPilot->add(roll);
    m_pAutoPilot->add(vpath);
    m_pAutoPilot->add(throttle);
}

Config::Config(Aircraft *ac) : m_pAC(ac)
{
    const QList<float> *t = ac->getTestThrottles();
    m_cThrottles = *t;
    m_pProfile.clear();

    for(qsizetype i = 0; i < t->size(); ++i) {
        Profile *p = new Profile(this, t->at(i));
        m_pProfile[t->at(i)] = p;
    }
    //m_cProfile[_p.getThrottle()] = _p;
    m_pCurProfile = m_pProfile[0];
}

Config::~Config()
{
    foreach(Profile *p, m_pProfile)
        delete p;
    m_pProfile.clear();
}

bool Config::set()
{
    m_pCurTP->inc();
    return m_pAC->set();
}

TestPoint* Config::nextTestPoint()
{
    TestPoint *tp = m_pCurTP->prev();
    //Q_ASSERT(tp);
    m_pCurTP = tp;
    //m_pAC->initTP(m_pCurTP);
    if (m_pCurTP && m_pCurTP->alt.raw() != m_pCurTP->next()->alt.raw()) {
        PerfPlugin::setTestAltitude(m_pCurTP->alt);
        PerfPlugin::setTestPosition();
    }
    return m_pCurTP;
}

TestPoint *Config::initTest()
{
    m_iIterator = 0;

    m_pAC->setFlaps(m_fFlaps);
    m_pAC->setGear(m_bGear);
    m_pAC->setGrossMass(m_fGrossMass);
    m_pAC->setCG(m_fCG);

    m_pAC->setXPdata();

    m_pCurTP = getCurProfile()->last();
    m_pAC->initTP(m_pCurTP);
    PerfPlugin::setTestPosition();
    return m_pCurTP;
}

void Config::runTest()
{
    //static int i = 0;
    //PerfPlugin::setTestAltitude(m_pCurTP->alt);
    //++i;
    //if (i < 1000) {
    //if (i < 200 || i > 400)
    PerfPlugin::setTestAtt();
    //if (i < 400 || i > 600)
    PerfPlugin::setTestPosition();
    //setGLpos(m_cTestGLpos);
    //if (i < 600 || i > 800)
    PerfPlugin::setCAS(m_pCurTP->speed, false);
    //PerfPlugin::setAngVel(PerfPlugin::m_cZeroV);
    //if (i < 800 || i > 1000)
    PerfPlugin::freezeAccNrot();
    m_pAC->resetFuel();
    //}
    //qDebug() << "Runtest i =" << i;
}

void Aircraft::exportToDir(QString dir)
{
    QString path = QString("%1/%2.%3").arg(dir, m_sName, m_sHash.toHex());
    QDir ACexport(path);

    if (!ACexport.exists() && !ACexport.mkpath(path)) {
        qDebug() << "Unable to create path" << path;
        return;
    }

    foreach(Config *p, m_cTestConfigs) {
        QString config = QString("%1/Config_%2_%3_%4_%5.txt").arg(path).arg(p->getGrossMass()).arg(p->getCG()).arg(p->getFlaps()).arg(p->getGear());
        QFile file(config);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODeviceBase::Truncate))
            continue;
        QTextStream out(&file);

        //out << "Test" << config;
        this->data(out);
        p->data(out);
        TestPoint::dataHeader(out);

        QMap<float, Profile*> profiles = p->getProfiles();

        foreach(Profile *i, profiles) {
            TestPoint *tp = i->first();
            while(tp) {
                tp->data(out);
                tp = tp->next();
            }

        }

        out.flush();
        file.close();
    }
}

void Aircraft::data(QTextStream &p)
{
    p << PerfPlugin::pluginName << "-plugin X-Plane Flight Test data report" << "\n";
    p << "\n";
    p << "Aircraft;" << m_sName << "\n";
    p << "Path;" << m_sPath << "\n";
    p << "Last modified;" << m_cModified.toString() << "\n";
    p << "Hash;" << m_sHash.toHex() << "\n";
    //p << "Test throttles;" << m_cTestThrottles << "\n";
    p << "VMO;" << m_fVMO.val() << m_fVMO.getUnit().sym().prepend(";") << "\n";
    p << "MMO;" << m_fMMO << ";Mach" << "\n";
    p << "Max alpha;" << m_fMaxAlpha << ";deg" << "\n";
    p << "Min alpha;" << m_fMinAlpha << ";deg" << "\n";
    p << "Max Power Lever Angle (PLA);" << m_fMaxThrottle * 100 << ";%" << "\n";
    p << "Min Power Lever Angle (PLA);" << m_fMinThrottle * 100 << ";%" << "\n";
    p << "Max Thrust;" << m_fMaxThrust << ";N" << "\n";
    p << "Min Thrust;" << m_fMinThrust << ";N" << "\n";
    p << "Dry operating mass;" << m_fDOM << ";kg" << "\n";
    p << "Fuel capasity;" << m_fMaxFuel << ";kg" << "\n";
    p << "Maximum Take Off Mass;" << m_fMTOM << ";kg" << "\n";
    p << "CG fwd limit from neutral;" << m_fCGfwd << ";m" << "\n";
    p << "CG aft limit from neutral;" << m_fCGaft << ";m" << "\n";
    p << "\n";
}

void Config::data(QTextStream &p)
{
    p << "Flight Test configuration";
    p << "\n";
    p << "Gross Mass;" << m_fGrossMass << ";kg" << "\n";
    p << "CG deviation from neutral;" << m_fCG << ";m" << "\n";
    p << "Flaps;" << m_fFlaps * 100.0f << ";%" << "\n";
    p << "Gears;" << (m_bGear ? "Up" : "Down") << "\n";
    p << "\n";
}

void Aircraft::captureData()
{
    m_pCurTP->throttle = m_fThrottle;
    m_pCurTP->pitchTrim = m_fPitch;
    m_pCurTP->rollTrim = m_fRoll;
    m_pCurTP->yawTrim = m_fYaw;
    m_pCurTP->flaps = PerfPlugin::getFlaps();
    m_pCurTP->gear = PerfPlugin::getGear();
    m_pCurTP->alpha = m_fAlpha;
    m_pCurTP->beta = PerfPlugin::getBeta();
    m_pCurTP->vpath = PerfPlugin::getVPath();
    m_pCurTP->Qdot = PerfPlugin::getAngAccQdot();
    m_pCurTP->Rdot = PerfPlugin::getAngAccRdot();
    m_pCurTP->Pdot = PerfPlugin::getAngAccPdot();
    m_pCurTP->TestDecel = PerfPlugin::getTestDeceleration();
    m_pCurTP->true_theta = PerfPlugin::getTrueTheta(); //Pitch
    m_pCurTP->true_phi = PerfPlugin::getTruePhi(); //Roll
    m_pCurTP->true_psi = PerfPlugin::getTruePsi(); //Heading
    m_pCurTP->Lift = PerfPlugin::getFlightPathForcesLift();
    m_pCurTP->Drag = PerfPlugin::getFlightPathForcesDrag();
    m_pCurTP->Side = PerfPlugin::getFlightPathForcesSide();
    m_pCurTP->CD = PerfPlugin::getCD();
    m_pCurTP->CL = PerfPlugin::getCL();
    m_pCurTP->a = PerfPlugin::getOpenGLacc().length();
    m_pCurTP->ax = PerfPlugin::getOpenGLaccX(); //acceleration linear
    m_pCurTP->ay = PerfPlugin::getOpenGLaccY();
    m_pCurTP->az = PerfPlugin::getOpenGLaccZ();
    m_pCurTP->g_total = PerfPlugin::getGforceTotal(); //"G-Force in aircraft"
    m_pCurTP->g_side = PerfPlugin::getGforceX(); //For a non accelerating condition total and normal should be 1 and others zero
    m_pCurTP->g_nrml = PerfPlugin::getGforceY();
    m_pCurTP->g_axil = PerfPlugin::getGforceZ();
    m_pCurTP->fside_aero = PerfPlugin::getAeroForceX();
    m_pCurTP->fnrml_aero = PerfPlugin::getAeroForceY();
    m_pCurTP->faxil_aero = PerfPlugin::getAeroForceZ();
    m_pCurTP->L_aero = PerfPlugin::getAeroMomentL();
    m_pCurTP->M_aero = PerfPlugin::getAeroMomentM();
    m_pCurTP->N_aero = PerfPlugin::getAeroMomentN();
    m_pCurTP->fside_prop = PerfPlugin::getEngineForceX();
    m_pCurTP->fnrml_prop = PerfPlugin::getEngineForceY();
    m_pCurTP->faxil_prop = PerfPlugin::getEngineForceZ();
    m_pCurTP->L_prop = PerfPlugin::getEngineMomentL();
    m_pCurTP->M_prop = PerfPlugin::getEngineMomentM();
    m_pCurTP->N_prop = PerfPlugin::getEngineMomentN();
    m_pCurTP->L_mass = PerfPlugin::getMassMomentL();
    m_pCurTP->M_mass = PerfPlugin::getMassMomentM();
    m_pCurTP->N_mass = PerfPlugin::getMassMomentN();
    m_pCurTP->fside_total = PerfPlugin::getTotalForceX();
    m_pCurTP->fnrml_total = PerfPlugin::getTotalForceY();
    m_pCurTP->faxil_total = PerfPlugin::getTotalForceZ();
    m_pCurTP->L_total = PerfPlugin::getTotalMomentL();
    m_pCurTP->M_total = PerfPlugin::getTotalMomentM();
    m_pCurTP->N_total = PerfPlugin::getTotalMomentN();
    m_pCurTP->GrossMass = PerfPlugin::getGrossMass();
    m_pCurTP->CG = PerfPlugin::getCG();
    m_pCurTP->recAlt = PerfPlugin::getAlt();
    m_pCurTP->recTAS = PerfPlugin::getTAS();
    m_pCurTP->recCAS = PerfPlugin::getCAS();
    m_pCurTP->recEAS = PerfPlugin::getIAS();
    m_pCurTP->Mach = PerfPlugin::getMachNo();
    m_pCurTP->local_c = PerfPlugin::getC();
    m_pCurTP->totalThrust = PerfPlugin::getThrustTotal();
    m_pCurTP->totalFF = PerfPlugin::getFFtotal();
    m_pCurTP->meanN1 = PerfPlugin::getN1mean();
    m_pCurTP->meanN2 = PerfPlugin::getN2mean();
    m_pCurTP->meanEPR = PerfPlugin::getEPRmean();
    m_pCurTP->pressure = PerfPlugin::getPressure();
    m_pCurTP->q = PerfPlugin::getQ();
    m_pCurTP->delta = m_pCurTP->pressure / PHY_P;
    m_pCurTP->rho = PerfPlugin::getRho();
    m_pCurTP->sigma = PerfPlugin::getSigma();
    m_pCurTP->tat = PerfPlugin::getTat(); //True air temperature
    m_pCurTP->sat = PerfPlugin::getSat(); //stagnation air temperature
    m_pCurTP->g = PerfPlugin::getG();
}

void Config::captureData()
{
    //m_pCurTP->state = completed;
    m_pAC->captureData();
}

TestPoint *Config::checkTP()
{
    //TestPoint *checked = nullptr;
    do {
        /*
        if (m_pCurTP)
            checked = m_pAC->checkTP(m_pCurTP);
        else {
            checked = nullptr;
            m_pCurTP = nullptr;
        }
        */
        TestPoint *checked = m_pAC->checkTP(m_pCurTP);
        //qDebug() << "Checked TP" << (void *) checked << "CurTP" << (void *) m_pCurTP;

        if (checked) {
            m_pCurTP = checked;
            m_pAC->initTP(m_pCurTP);
            return m_pCurTP;
        } else {
            getNextProfile();
            if (m_pCurProfile) {
               m_pCurTP = m_pCurProfile->last();
            }
        }
        //qDebug() << "CurTP" << (void *) m_pCurTP;
    } while(m_pCurProfile);

    return nullptr;
}


Profile *Config::getNextProfile()
{
    ++m_iIterator;
    if (m_iIterator < m_cThrottles.size())
        m_pCurProfile = m_pProfile[m_cThrottles.at(m_iIterator)];
    else
        m_pCurProfile = nullptr;

    return m_pCurProfile;
}
