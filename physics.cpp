#include "physics.h"

Unit::Unit(QString sym, double mult, double exp)
{
    symbol = sym;
    multiplier = mult;
    exponent = exp;
}

Physics::Physics()
{

}

Quantity::Quantity(Unit symbol, double val, bool raw) : unit(symbol)
{
    //unit = symbol;
    if (raw)
        value = val;
    else
        set(val);
}

QString Quantity::toStr()
{
    QString str = QString::number(val()) + unit.sym();
    return str;
}


QDataStream& operator<<(QDataStream &stream, const Quantity &q)
{
    stream << q.val() << q.unit.sym();

    return stream;
}

Length::Length(double val) : Quantity(Unit("m"), val)
{

}

Length::Length(Unit symbol, double val, bool raw) : Quantity(symbol, val, raw)
{

}

Mass::Mass() : Quantity(Unit("kg", 1000))
{

}

Time::Time() : Quantity(Unit("s"))
{

}

Coordinate::Coordinate(float lat, float lon, float elevation)
{
    elev = elevation;
    setLatLon(lat, lon);
}

float Coordinate::getLat()
{
    return latlon.toEulerAngles().x();
}

float Coordinate::getLon()
{
    return latlon.toEulerAngles().y();
}

float Coordinate::getElev()
{
    return elev;
}

Coordinate& Coordinate::setLatLon(float lat, float lon)
{
    latlon = QQuaternion::fromEulerAngles(lat, lon, 0);

    return *this;
}

Coordinate& Coordinate::setLat(float lat)
{
    return setLatLon(lat, getLon());
}

Coordinate& Coordinate::setLon(float lon)
{
    return setLatLon(getLat(), lon);
}

Coordinate& Coordinate::setElev(double elevation)
{
    elev = elevation;

    return *this;
}

float Aerodynamics::qToQc(float mach)
{
    //Change
    //https://www.sciencedirect.com/topics/engineering/indicated-air-speed
    //https://www.sciencedirect.com/topics/engineering/stagnation-pressure
    if (!mach)
        return 1.0f;
    return 2.0f * qPow(compCoef(mach), PHY_GAMMA / (PHY_GAMMA - 1.0f) - 1.0f) / PHY_GAMMA / qPow(mach, 2.0f);
}


float Aerodynamics::casToQc(Quantity cas)
{
    // (Eq 2.23) p. 2.15 FTM 108 gives impact pressure without pitot tube error
    //return PHY_P * qPow(1 + 0.2f * qPow(cas.raw(), 2) / (PHY_C * PHY_C), 3.5f) - PHY_P;
    return PHY_P * qPow(1.0f + (PHY_GAMMA - 1.0f) / 2.0f * qPow(cas.raw(), 2.0f) / (PHY_C * PHY_C), PHY_GAMMA / (PHY_GAMMA - 1.0f)) - PHY_P;
}

float Aerodynamics::qcToCasMS(float qc)
{
    // (Eq 2.19) p. 2.14 FTM 108 gives CAS from true impact pressure without pitot tube error
    qDebug() << "qcToCasMS, qc" << qc << "Gamma" << PHY_GAMMA << "Rho" << PHY_RHO << "Press" << PHY_P;
    qDebug() << qPow(qc / PHY_P, (PHY_GAMMA - 1.0f) / PHY_GAMMA) << (qPow(qc / PHY_P + 1.0f, (PHY_GAMMA - 1.0f) / PHY_GAMMA) - 1.0f) << 2.0f * PHY_GAMMA * PHY_P / (PHY_GAMMA - 1.0f) / PHY_RHO << 2.0f * PHY_GAMMA * PHY_P / (PHY_GAMMA - 1.0f) / PHY_RHO * (qPow(qc / PHY_P + 1.0f, (PHY_GAMMA - 1.0f) / PHY_GAMMA) - 1.0f);
    return qSqrt(2.0f * PHY_GAMMA * PHY_P / (PHY_GAMMA - 1.0f) / PHY_RHO * (qPow(qc / PHY_P + 1.0f, (PHY_GAMMA - 1.0f) / PHY_GAMMA) - 1.0f));
}

Quantity Aerodynamics::qcToCas(float qc)
{
    return Quantity(Unit("kt", 1.852f/3.6f), qcToCasMS(qc), true);
}

float Aerodynamics::qcToMach(float qc, Atmosphere state)
{
    // (Eq 2.28) p. 2.17 FTM 108 gives mach from true impact pressure without pitot tube error
    return qSqrt(2.0f / (PHY_GAMMA - 1.0f) * qPow(qc / state.pressure + 1.0f, (PHY_GAMMA - 1.0f) / PHY_GAMMA) - 2.0f / (PHY_GAMMA - 1.0f));
}

float Aerodynamics::machToQc(float mach, Atmosphere state)
{
    // (Eq 2.29) p. 2.18 FTM 108 gives true impact pressure without pitot tube error from Mach and ambient pressure
    qDebug() << "Pressure" << state.pressure << "Gamma" << PHY_GAMMA << "Mach" << mach;
    qDebug() << "Result" << state.pressure * (qPow(1.0f + (PHY_GAMMA - 1.0f) / 2.0f * qPow(mach, 2.0f), PHY_GAMMA / (PHY_GAMMA - 1.0f))) - state.pressure;
    qDebug() << qPow(mach, 2.0f) << qPow((PHY_GAMMA - 1.0f) / 2.0f * qPow(mach, 2.0f), PHY_GAMMA / (PHY_GAMMA - 1.0f));
    return state.pressure * (qPow(1.0f + (PHY_GAMMA - 1.0f) / 2.0f * qPow(mach, 2.0f), PHY_GAMMA / (PHY_GAMMA - 1.0f))) - state.pressure;
}

float Aerodynamics::tasToQ(Quantity tas, Atmosphere state) // state.rho
{
    return state.rho * qPow(tas.raw(), 2) / 2;
}

float Aerodynamics::tasToQc(Quantity tas, Atmosphere state)
{
    // (Eq 2.15 solved for Qc) p. 2.13 FTM 108 gives true impact pressure without pitot tube error
   return state.pressure * (qPow((PHY_GAMMA - 1.0f) / PHY_GAMMA / 2.0f / state.pressure * state.rho * qPow(tas.raw(), 2.0f) + 1.0f, PHY_GAMMA / (PHY_GAMMA - 1.0f)) - 1.0f);
}

float Aerodynamics::qcToTasMS(float qc, Atmosphere state)
{
    //Quantity v(Unit("kt", 1.852f/3.6f));
    // (Eq 2.16) p. 2.13 FTM 108 gives tas from true impact pressure without pitot tube error
    return qSqrt(2.0f * PHY_GAMMA / (PHY_GAMMA - 1.0f) * state.pressure / state.rho * (qPow(qc / state.pressure + 1.0f, (PHY_GAMMA - 1.0f) / PHY_GAMMA) - 1.0f));
}

float Aerodynamics::tasToMach(Quantity tas, Atmosphere state)
{
    return tas.raw() / state.c;
}

float Aerodynamics::machToTasMS(float mach, Atmosphere state)
{
    return mach * state.c;
}

Quantity Aerodynamics::machToCas(float mach, Atmosphere state)
{
    return qcToCas(machToQc(mach, state));
}

