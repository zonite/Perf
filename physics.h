#ifndef PHYSICS_H
#define PHYSICS_H

#include <QString>
#include <QQuaternion>
#include <QDataStream>

//inch of mercury is 0.00029529983071445 Pa

class Unit
{
public:
    Unit(QString sym, double mult = 1, double exp = 1);
    double mult() const
    { return multiplier; }
    QString sym() const
    { return symbol; }
private:
    QString symbol = "";
    double multiplier = 1;
    double exponent = 1;
};

class Physics
{
public:
    Physics();

};

class Quantity : public Physics
{
public:
    Quantity(Unit symbol, double val = 0, bool raw = false);
    //Quantity(QString symbol, double val = 0);
    Quantity& setUnit(Unit u)
    { unit = u; return *this; }
    Quantity& set(const Quantity& q)
    { value = q.value; return *this; }
    Quantity& set(const double& f)
    { value = f * unit.mult(); return *this; }
    Quantity add(const Quantity& q)
    { return Quantity(unit, value + q.value, true); }
    Quantity add(const double& f)
    { return Quantity(unit, value + f * unit.mult(), true); }
    Quantity subs(const Quantity& q)
    { return Quantity(unit, value - q.value, true); }
    Quantity subs(const double& f)
    { return Quantity(unit, value - f * unit.mult(), true); }
    Quantity mult(const Quantity& q)
    { return Quantity(unit, value * q.value, true); }
    Quantity mult(const double& f)
    { return Quantity(unit, value * f, true); }

    Unit getUnit()
    { return unit; }
    double val() const
    { return value / unit.mult(); }
    double raw() const
    { return value; }

    QString toStr();
    friend QDataStream& operator<<(QDataStream &stream, const Quantity &q);
    Quantity& operator=(const Quantity& q)
    { return set(q); }
    Quantity& operator=(const double& q)
    { return set(q); }
    Quantity operator+(const Quantity& q)
    { return add(q); }
    Quantity operator-(const Quantity& q)
    { return subs(q); }
    Quantity operator*(const Quantity& q)
    { return mult(q); }
    Quantity operator*(const double& q)
    { return mult(q); }
    bool operator<(const Quantity& q)
    { return value < q.value; }
    bool operator>(const Quantity& q)
    { return value > q.value; }
    bool operator<=(const Quantity& q)
    { return value <= q.value; }
    bool operator>=(const Quantity& q)
    { return value >= q.value; }
    bool operator==(const Quantity& q)
    { return value == q.value; }
private:
    double value = 0;
    Unit unit;
};

class Length : public Quantity
{
public:
    Length(double val = 0);
    Length(Unit symbol, double val = 0, bool raw = false);
};

class Mass : public Quantity
{
public:
    Mass();
};

class Time : public Quantity
{
public:
    Time();
};

/*
class Area : public Quantity
{
public:
    Area();
};

class Volyme : public Quantity
{
public:
    Volyme();
};

class Angle : public Quantity
{
public:
    Angle();
};
*/


#define PHY_R 8.314463f
#define PHY_MAIR 0.0289645f
#define PHY_RHO 1.225f
#define PHY_T 288.15f
#define PHY_P 101325.0f
#define PHY_GAMMA 1.4f
#define PHY_C (sqrt(PHY_GAMMA * PHY_T * PHY_R / PHY_MAIR))
#define PHY_K -273.15f
#define PHY_IN_MM 25.4f
#define PHY_HG_PA 133.322387415
#define PHY_INHG_PA (PHY_IN_MM * PHY_HG_PA)
#define PHY_KT_MS (1.852 / 3.6)
#define PHY_LBS_KG 0.45359237
#define PHY_FT_M 0.3048f
#define PHY_G 9.80665f
// Temperature sensor recovery factor (sensor dependant ~ 1 - 0.8)
#define PHY_Kt 1.0f

class Atmosphere : public Physics
{
public:
    Atmosphere() {};
    float pressure = PHY_P;
    float rho = PHY_RHO; //air density
    float tat = PHY_T; //true air temperature/static air temperature/ambient
    float sat = PHY_T; //stagnation air temperature/total air temperature
    float c = PHY_C; //speed of sound
    float q = 0.0f;
    float dp = 0.0f; //dewpoint
    float gamma = PHY_GAMMA;
};


class Aerodynamics : public Physics
{
public:
    Aerodynamics();
    //Quantity(Unit("kt", 1.852f/3.6f), , true)

    static float qToQc(float mach);

    static float casToQc(Quantity cas);
    static float qcToCasMS(float qc);
    static Quantity qcToCas(float qc);

    static float machToQc(float mach, Atmosphere state); // state.pressure
    static float qcToMach(float qc, Atmosphere state); // state.pressure

    static float tasToQ(Quantity tas, Atmosphere state); // state.rho
    static float tasToQc(Quantity tas, Atmosphere state); // state.pressure state.rho
    static float qcToTasMS(float qc, Atmosphere state); // state.pressure state.rho
    static Quantity qcToTas(float qc, Atmosphere state) // state.pressure state.rho
    { return Quantity(Unit("kt", 1.852f/3.6f), qcToTasMS(qc, state), true); }

    static float tasToMach(Quantity tas, Atmosphere state); // state.c
    static float machToTasMS(float mach, Atmosphere state); // state.c
    static Quantity machToTas(float mach, Atmosphere state) // state.c
    { return Quantity(Unit("kt", 1.852f/3.6f), machToTasMS(mach, state), true); }

    static Quantity machToCas(float mach, Atmosphere state);
    static float casToMach(Quantity cas, Atmosphere state)
    { return qcToMach(casToQc(cas), state); }
    static Quantity casToTas(Quantity cas, Atmosphere state)
    { return qcToTas(casToQc(cas), state).setUnit(cas.getUnit()); }
    static Quantity tasToCas(Quantity tas, Atmosphere state)
    { return qcToCas(tasToQc(tas, state)).setUnit(tas.getUnit()); }

    static float tatToSat(float tat, float mach)
    { return tat * compCoef(mach); }
    static float satToTat(float sat, float mach)
    { return sat / compCoef(mach); }
    static float compCoef(float mach)
    { return 1.0f + PHY_Kt * (PHY_GAMMA - 1.0f ) / 2.0f * qPow(mach, 2); }
    //{ return 1.0f + qPow(mach, 2) / 5.0f; }
    static float compHeat(Quantity tas) //Addition of heat from stagnation
    { return PHY_Kt / 2.0f * (PHY_GAMMA - 1.0f ) / PHY_GAMMA * PHY_MAIR / PHY_R * qPow(tas.raw(), 2); }

};


class Coordinate
{
public:
    Coordinate(float lat = 0, float lon = 0, float elevation = 0);
    float getLat();
    float getLon();
    float getElev();
    Coordinate& setLatLon(float lat, float lon);
    Coordinate& setLat(float lat);
    Coordinate& setLon(float lon);
    Coordinate& setElev(double elev);
private:
    QQuaternion latlon;
    double elev = 0;
};

#endif // PHYSICS_H
