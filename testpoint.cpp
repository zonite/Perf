#include "testpoint.h"

TestPoint *TestPoint::newPoint(Profile *parent)
{
    if (parent) {
        return new TestPoint(parent);
    }

    return nullptr;
}

TestPoint::~TestPoint()
{
    if (m_pNext)
        m_pNext->setPrev(m_pPrev);
    if (m_pPrev)
        m_pPrev->setNext(m_pNext);

    m_pNext = nullptr;
    m_pPrev = nullptr;
}

TestPoint::TestPoint(Profile *parent) : m_pParent(parent)
{
    //throttle = m_pParent->getThrottle();
}

QString TestPoint::statusToStr(status s)
{
    QString ret;
    switch(s) {
    case overspd:
        ret = "Over SPD";
        break;
    case unstable:
        ret = "Unstable";
        break;
    case low_speed:
        ret = "Stall";
        break;
    case low_thrust:
        ret = "Low Thrust";
        break;
    case completed:
        ret = "Completed";
        break;
    case intest:
        ret = "In-Test";
        break;
    case notrun:
        ret = "Not run";
        break;
    case rerun:
        ret = "Re run";
        break;
    }
    return ret;
}

QString TestPoint::typeToStr(pointType t)
{
    QString ret;
    switch(t) {
    case fixedPower:
        ret = "Fixed Thrust";
        break;
    case autoTHR:
        ret = "Auto Thrust";
        break;
    }
    return ret;
}

void TestPoint::dataHeader(QTextStream &out)
{
    out << "Test point data:" << "\n";
    out << "TP type;";
    out << "TP state;";
    out << "Runned count;";
    out << "Set Altitude;";
    out << "Recorded Altitude;";
    out << "Set Speed (CAS);";
    out << "Recorded CAS;";
    out << "Recorded EAS;";
    out << "Recorded TAS;";
    out << "Mach no;";
    out << "Flap position;";
    out << "Gear position;";
    out << "PLA (%);";
    out << "Pitch command;";
    out << "Roll command;";
    out << "Yaw command;";
    out << "Alpha (AOA deg);";
    out << "Beta (deg);";
    out << "Vertical path (deg);";
    out << "Theta (deg);";
    out << "Phi (deg);";
    out << "Psi (deg);";
    out << "Lift (N);";
    out << "Drag (N);";
    out << "Side force (N);";
    out << "CL;";
    out << "CD;";
    out << "Total linear acceleration (m/s2);";
    out << "X-axis acceleration (m/s2);";
    out << "Y-axis acceleration (m/s2);";
    out << "Z-axis acceleration (m/s2);";
    out << "Total G-force (acceleration/standard G);";
    out << "Side G-force;";
    out << "Normal G-force;";
    out << "Axil G-force;";
    out << "Aerodynamic side force (N);";
    out << "Aerodynamic normal force (N);";
    out << "Aerodynamic axil force (N);";
    out << "Aerodynamic L moment of force (Nm);";
    out << "Aerodynamic M moment of force (Nm);";
    out << "Aerodynamic N moment of force (Nm);";
    out << "Engine side force (N);";
    out << "Engine normal force (N);";
    out << "Engine axil force (N);";
    out << "Engine L moment of force (Nm);";
    out << "Engine M moment of force (Nm);";
    out << "Engine N moment of force (Nm);";
    out << "Mass L moment of force (Nm);";
    out << "Mass M moment of force (Nm);";
    out << "Mass N moment of force (Nm);";
    out << "Total side force (N);";
    out << "Total normal force (N);";
    out << "Total axil force (N);";
    out << "Total L moment of force (Nm);";
    out << "Total L moment of force (Nm);";
    out << "Total L moment of force (Nm);";
    out << "Gross mass (kg);";
    out << "Deviation of CG from neutral (m);";
    out << "Local speed of sound;";
    out << "Total Thrust (N);";
    out << "Total fuel feed (kg/s);";
    out << "Average N1 of all engines;";
    out << "Average N2 of all engines;";
    out << "Average EPR of all engines;";
    out << "Ambient pressure;";
    out << "Dynamic pressure;";
    out << "Pressure ratio (Delta);";
    out << "Density (kg/m3);";
    out << "Density ratio (Sigma);";
    out << "True air temperature (K);"; //True air temperature
    out << "Stagnation air temperature (K);"; //stagnation air temperature
    out << "Gravity at current altitude (m/s2);";
    out << "\n";
}

void TestPoint::data(QTextStream &out)
{
    out << typeToStr(type) << ";";
    out << statusToStr(state) << ";";
    out << count << ";";
    out << alt.val() << ";";
    out << recAlt.val() << ";";
    out << speed.val() << ";";
    out << recCAS.val() << ";";
    out << recEAS.val() << ";";
    out << recTAS.val() << ";";
    out << Mach << ";";
    out << flaps << ";";
    out << gear << ";";
    out << throttle * 100 << ";";
    out << pitchTrim << ";";
    out << rollTrim << ";";
    out << yawTrim << ";";
    out << alpha << ";";
    out << beta << ";";
    out << vpath << ";";
    out << true_theta << ";";
    out << true_phi << ";";
    out << true_psi << ";";
    out << Lift << ";";
    out << Drag << ";";
    out << Side << ";";
    out << CL << ";";
    out << CD << ";";
    out << a << ";";
    out << ax << ";";
    out << ay << ";";
    out << az << ";";
    out << g_total << ";";
    out << g_side << ";";
    out << g_nrml << ";";
    out << g_axil << ";";
    out << fside_aero << ";";
    out << fnrml_aero << ";";
    out << faxil_aero << ";";
    out << L_aero << ";";
    out << M_aero << ";";
    out << N_aero << ";";
    out << fside_prop << ";";
    out << fnrml_prop << ";";
    out << faxil_prop << ";";
    out << L_prop << ";";
    out << M_prop << ";";
    out << N_prop << ";";
    out << L_mass << ";";
    out << M_mass << ";";
    out << N_mass << ";";
    out << fside_total << ";";
    out << fnrml_total << ";";
    out << faxil_total << ";";
    out << L_total << ";";
    out << M_total << ";";
    out << N_total << ";";
    out << GrossMass << ";";
    out << CG << ";";
    out << local_c << ";";
    out << totalThrust << ";";
    out << totalFF << ";";
    out << meanN1 << ";";
    out << meanN2 << ";";
    out << meanEPR << ";";
    out << pressure << ";";
    out << q << ";";
    out << delta << ";";
    out << rho << ";";
    out << sigma << ";";
    out << tat << ";"; //True air temperature
    out << sat << ";"; //stagnation air temperature
    out << g << ";";
    out << "\n";
}

//void TestPoint::captureData()
//{
//    state = completed;
//    throttle = m_p
//}

void TestPoint::clone(const TestPoint *tp)
{
    if (type == autoTHR && tp->type == autoTHR)
        throttle = tp->throttle;

    pitchTrim = tp->pitchTrim;
    rollTrim = tp->rollTrim;
    yawTrim = tp->yawTrim;
    alpha = tp->alpha;
}

void TestPoint::inc()
{
    ++count;
}

TestPointList::TestPointList() : QList<TestPoint*>()
{

}

Profile::Profile(Config *config, float throttle) : m_pConfig(config), m_fThrottle(throttle)
{
    Aircraft *ac = m_pConfig->getAC();
    Length MaxAlt = ac->getMaxAlt();
    Length MinAlt = ac->getMinAlt();
    Length AltInc = ac->getAltInc();
    Quantity SpeedInc = ac->getSpeedInc();
    Quantity VMO = ac->getVMO();

    qsizetype alts, speeds, min;
    double max, inc;

    qDebug() << "New profile" << this;

    inc = AltInc.raw();
    max = MaxAlt.raw();
    min = MinAlt.raw() / inc;
    alts = fmod(max, inc) ? 2 : 1;
    alts += max / inc;
    alts -= min;

    qDebug() << "Values:" << MaxAlt.toStr() << MinAlt.toStr() << AltInc.toStr() << "alts" << alts;

    QList<float> indToAlt(alts);
    indToAlt[0] = MinAlt.raw();
    for(qsizetype i = 1; i < alts - 1; ++i)
        indToAlt[i] = inc * i + MinAlt.raw();
    qDebug() << "indToAlt" << indToAlt;

    indToAlt[alts - 1] = max;

    qDebug() << "indToAlt" << indToAlt;

    qDebug() << "Speeds";

    inc = SpeedInc.raw();
    max = VMO.raw();
    speeds = fmod(max, inc) ? 1 : 0;
    speeds += max / inc;

    QList<float> indToSpeed(speeds);
    for(qsizetype i = 0; i < speeds; ++i)
        indToSpeed[i] = inc * (i + 1);
    indToSpeed[speeds - 1] = max;

    qDebug() << "indToSpeed" << speeds << indToSpeed;

    m_cAltitudes.clear();
    //m_cAltitudes[indToAlt[0]] = 0;

    m_cSpeeds.clear();
    //m_cSpeeds[indToSpeed[0]] = 0;

    m_pTestpoints.resize(alts);

    TestPoint *prev = nullptr;
    TestPoint *cur = nullptr;

    qDebug() << "Create points";

    for(qsizetype i = 0; i < alts; ++i) {
        m_pTestpoints[i].resize(speeds);
        m_cAltitudes[indToAlt[i]] = i;
        for(qsizetype k = 0; k < speeds; ++k) {
            cur = TestPoint::newPoint(this);
            cur->speed = Quantity(cur->speed.getUnit(), indToSpeed[k], true);
            cur->alt = Length(AltInc.getUnit(), indToAlt[i], true);
            cur->throttle = m_fThrottle;
            if (m_fThrottle < 0.0f) {
                cur->throttle = 0.5f;
                cur->type = autoTHR;
            }

            cur->setPrev(prev);
            if (prev)
                prev->setNext(cur);

            prev = cur;

            //qDebug() << "Set next and prev" << (void*)cur;

            m_cSpeeds[indToSpeed[k]] = k;
            m_pTestpoints[i][k] = cur;

            qDebug() << "testpoint[" << i << "][" << k << "]: alt" << cur->alt.raw() << "speed" << cur->speed.raw() << "Set next and prev" << (void*)cur;
        }
    }

    qDebug() << "Speeds" << m_cSpeeds;
    qDebug() << "Alts" << m_cAltitudes;

    qDebug() << "Done";
}

float TestPoint::getThrottle() { return m_pParent->getThrottle(); }
float TestPoint::getGear() { return m_pParent->getGear(); }
float TestPoint::getFlaps() { return m_pParent->getFlaps(); }

float Profile::getGear() { return m_pConfig->getGear(); }
float Profile::getFlaps() { return m_pConfig->getFlaps(); }
float Profile::getGrossMass() { return m_pConfig->getGrossMass(); }
float Profile::getCG() { return m_pConfig->getCG(); }

Profile::~Profile()
{
    TestPoint *p = m_pTestpoints[0][0];
    while (p) {
        p = p->next();
        delete p->prev();
    }
}

void Profile::setThrottle()
{
    m_pConfig->getAC()->setThrottle(m_fThrottle);
}

