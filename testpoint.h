#ifndef TESTPOINT_H
#define TESTPOINT_H

#include <QObject>
#include <QList>
#include <QTextStream>

//#include "perfplugin.h"
#include "Physics"

class TestPoint;
class Profile;
//class Aircraft;
class Config;

#include "aircraft.h"

enum status {
    overspd = -4,
    unstable = -3,
    low_speed = -2,
    low_thrust = -1, //too little thrust
    completed = 0,
    intest = 1,
    notrun = 2,
    rerun = 3
};

enum pointType {
    fixedPower = 0,
    autoTHR
};

class TestPoint
{
//    Q_OBJECT
public:
    static TestPoint *newPoint(Profile *parent);
    TestPoint(const TestPoint &) = delete;
    TestPoint &operator=(const TestPoint &) = delete;

    static void dataHeader(QTextStream &out);
    void data(QTextStream &out);

    void clone(const TestPoint *tp);

    void captureData();
    QString statusToStr(status s);
    QString typeToStr(pointType t);

    Profile *getProfile()
    { return m_pParent; }

    TestPoint *next() { return m_pNext; }
    TestPoint *prev() { return m_pPrev; }
    void setNext(TestPoint *p) { m_pNext = p; }
    void setPrev(TestPoint *p) { m_pPrev = p; }
    void inc();

    float getThrottle();
    Quantity getSpeed() { return speed; }
    float getAlt() { return alt.raw(); }
    float getGear();
    float getFlaps();

    /*
    //Datarefs sim/aircraft/parts/
    float acf_e; //Oswald's E
    float acf_AR; //Aspect Ratio
    float acf_alpha_max; //Alpha max
    float acf_mac; //MAC
    //Datarefs sim/aircraft/weight/
    float acf_m_empty;
    float acf_m_displaced;
    float acf_m_max;
    float acf_m_fuel_tot; //in lbs
    //Datarefs sim/airfoils/
    float afl_cl;
    float afl_cd;
    float afl_cm;
    */

    //Datarefs sim/flightmodel/controls/
    //Datarefs sim/flightmodel/engine/
    //float POINT_thrust; //[8]
    //Datarefs sim/flightmodel/forces/
    //float fside_aero; //Aerodynamic forces – sideways – ACF X. Override with override_wing_forces
    //float fnrml_aero; //Aerodynamic forces – upward – ACF Y.
    //float faxil_aero; //Aerodynamic forces – backward – ACF Z
    //float L_aero; //The roll moment due to aerodynamic forces – positive = right roll. Override with Override with override_wing_Forces
    //float M_aero; //The pitch moment due to aerodynamic forces – positive = pitch up.
    //float N_aero; //The yaw moment due to aerodynamic forces – positive = yaw right/clockwise.

    enum status state = notrun;
    float throttle = 0.0f;
    float pitchTrim = 0.0f;
    float rollTrim = 0.0f;
    float yawTrim = 0.0f;
    float alpha = 0.0f;
    float beta = 0.0f;
    float vpath = 0.0f;
    float Qdot = 0.0f;
    float Rdot = 0.0f;
    float Pdot = 0.0f;
    float TestDecel = 0.0f;
    float true_theta = 0.0f; //Pitch
    float true_phi = 0.0f; //Roll
    float true_psi = 0.0f; //Heading
    float Lift = 0.0f; //N lift_path_axis
    float Drag = 0.0f; //N drag_path_axis
    float Side = 0.0f; //N side_path_axis
    float CD = 0.0f;
    float CL = 0.0f;
    float a = 0.0f; //total acceleration
    float ax = 0.0f; //acceleration linear
    float ay = 0.0f;
    float az = 0.0f;
    float g_total = 0.0f; //"G-Force in aircraft"
    float g_side = 0.0f; //For a non accelerating condition total and normal should be 1 and others zero
    float g_nrml = 0.0f;
    float g_axil = 0.0f;
    float fside_aero = 0.0f; //Aerodynamic forces – sideways – ACF X. Override with override_wing_forces
    float fnrml_aero = 0.0f; //Aerodynamic forces – upward – ACF Y.
    float faxil_aero = 0.0f; //Aerodynamic forces – backward – ACF Z
    float L_aero = 0.0f; //The roll moment due to aerodynamic forces – positive = right roll. Override with Override with override_wing_Forces
    float M_aero = 0.0f; //The pitch moment due to aerodynamic forces – positive = pitch up.
    float N_aero = 0.0f; //The yaw moment due to aerodynamic forces – positive = yaw right/clockwise.
    float fside_prop = 0.0f; //engine forces – sideways – ACF X. Override with override_wing_forces
    float fnrml_prop = 0.0f; //engine forces – upward – ACF Y.
    float faxil_prop = 0.0f; //engine forces – backward – ACF Z
    float L_prop = 0.0f; //The roll moment due to engine forces – positive = right roll. Override with Override with override_wing_Forces
    float M_prop = 0.0f; //The pitch moment due to engine forces – positive = pitch up.
    float N_prop = 0.0f; //The yaw moment due to engine forces – positive = yaw right/clockwise.
    float L_mass = 0.0f; //The roll moment due to mass forces – positive = right roll. Override with Override with override_wing_Forces
    float M_mass = 0.0f; //The pitch moment due to mass forces – positive = pitch up.
    float N_mass = 0.0f; //The yaw moment due to mass forces – positive = yaw right/clockwise.
    float fside_total = 0.0f; //Total forces – sideways – ACF X. Override with override_wing_forces
    float fnrml_total = 0.0f; //Total forces – upward – ACF Y.
    float faxil_total = 0.0f; //Total forces – backward – ACF Z
    float L_total = 0.0f; //The roll moment due to Total forces – positive = right roll. Override with Override with override_wing_Forces
    float M_total = 0.0f; //The pitch moment due to Total forces – positive = pitch up.
    float N_total = 0.0f; //The yaw moment due to Total forces – positive = yaw right/clockwise.
    float GrossMass = 0.0f;
    float CG = 0.0f; //center of gravity
    Length recAlt = Length(Unit("ft", 0.3048f), 0.0f);
    Quantity recTAS = Quantity(Unit("kt", 1.852f/3.6f), 0.0f);
    Quantity recCAS = Quantity(Unit("kt", 1.852f/3.6f), 0.0f);
    Quantity recEAS = Quantity(Unit("kt", 1.852f/3.6f), 0.0f);
    float Mach = 0.0f;
    float local_c = 0.0f; //speed_sound_ms
    float totalThrust = 0.0f;
    float totalFF = 0.0f;
    float meanN1 = 0.0f;
    float meanN2 = 0.0f;
    float meanEPR = 0.0f;
    float pressure = 0.0f;
    float q = 0.0f;
    float delta = 0.0f;
    float rho = 0.0f;
    float sigma = 0.0f;
    float tat = 0.0f;
    float sat = 0.0f;
    float g = 0.0f;
    QDateTime date;

    Quantity speed = Quantity(Unit("kt", 1.852f/3.6f), 0.0f);
    Length alt = Length(Unit("ft", 0.3048f), 1000.0f);
    float flaps = 0.0f;
    bool gear = false;
    enum pointType type = fixedPower;
    qsizetype count = 0;

    friend void initTestpoint(TestPoint *m_pCurrent);
    friend Profile;

//signals:

private:
    explicit TestPoint(Profile *parent);
    ~TestPoint();

    Profile *m_pParent = nullptr;
    TestPoint *m_pNext = nullptr;
    TestPoint *m_pPrev = nullptr;
};

class TestPointList : public QList<TestPoint*>
{
//    Q_OBJECT
public:
    explicit TestPointList();
};

class Profile
{
public:
    Profile(Config *config, float throttle);

    void setThrottle();
    float getThrottle()
    { return m_fThrottle; }

    TestPoint *last() { return m_pTestpoints.last().last(); }
    TestPoint *first() { return m_pTestpoints.first().first(); }

    float getGear();
    float getFlaps();
    float getGrossMass();
    float getCG();

    //friend Config::~Config();
    friend class Config;

private:
    ~Profile();

    Config *m_pConfig = nullptr;
    float m_fThrottle;
    //QMap<float, QMap<float, float>> m_cTestpoints;
    QMap<float, qsizetype> m_cSpeeds;
    QMap<float, qsizetype> m_cAltitudes;
    QList<QList<TestPoint*>> m_pTestpoints;
};


#endif // TESTPOINT_H
