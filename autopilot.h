#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "Control"

template<typename T = float>
class AutopilotAxis;
template<typename T = float>
class Autopilot;
//class PerfPlugin;
//template<typename T>
class Aircraft;

//#include "perfplugin.h"

template<typename T>
class Autopilot
{
public:
    Autopilot() {};
    ~Autopilot();

    bool set();
    void add(AutopilotAxis<T> *newAxis)
    { axis.append(newAxis); }
    bool isStable()
    { return stable; }

private:
    QList<AutopilotAxis<T>*> axis;
    bool stable = false;
};


template<typename T>
class AutopilotAxis
{
public:
    AutopilotAxis(T acc, T(*pv)(), Aircraft *ac, void (Aircraft::*mv)(T), Control<T> *_control = nullptr, Amean<T> *mean = nullptr);
    AutopilotAxis(T acc, T(*pv)(), Aircraft *ac, AutopilotAxis *chld, Control<T> *_control = nullptr, Amean<T> *mean = nullptr);
    virtual ~AutopilotAxis();

    virtual bool set();
    virtual bool set(T sp)
    { control->sp = sp; if (PVmean) PVmean->reset(); return set(); }
    virtual void setScale(T _max, T _min);
    virtual void setNegScale(T _min);
    virtual void setPosScale(T _max);
    bool isStable()
    { return stable; }

protected:
    //AutopilotAxis(T _sp = 0.0f, T _p = 1.0f, T _i = 1.0f, T _d = 1.0f, T _dt = 1.0f, T _max = -1.0f, T _min = 0.0f);

    void setMv(T pv);

    Control<T> *control = nullptr;
    T (*get_pv)() = nullptr;
    void (Aircraft::*set_mv)(T) = nullptr;
    Amean<T> *PVmean = nullptr;
    bool scaleAxis();

    Aircraft *m_pAC = nullptr;
    AutopilotAxis *child = nullptr;
    T accuracy = 0.0f;
    bool neg_scaled = false;
    bool pos_scaled = false;
    bool scaled = false;
    T neg_scale = 1.0f;
    T pos_scale = 1.0f;
    T curE = 0.0f;
    T sp = 0.0f;
    bool stable = false;
    int count = 0;
    //Filter<T> *filter;
};

template<typename T>
class AutothrottleAxis : public AutopilotAxis<T>
{
public:
    AutothrottleAxis(T acc, T(*pv)(), Aircraft *ac, void (Aircraft::*mv)(T), Control<T> *_control = nullptr, Amean<T> *mean = nullptr);
    virtual ~AutothrottleAxis();

    virtual bool set();
    virtual bool set(T sp)
    { this->control->sp = sp; if (this->PVmean) this->PVmean->reset(); return set(); }

private:
    T thrust = 0.0f;
};

#endif // AUTOPILOT_H
