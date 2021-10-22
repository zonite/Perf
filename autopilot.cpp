#include "autopilot.h"
#include "perfplugin.h"
#include "testpoint.h"

/*
template<typename T>
Autopilot<T>::Autopilot()
{

}
*/

template<typename T>
Autopilot<T>::~Autopilot()
{
    for (qsizetype i = 0; i < axis.size(); ++i) {
        delete axis.at(i);
    }
    axis.clear();
}

template<typename T>
bool Autopilot<T>::set()
{
    //return false; //Debug!!!
    bool result;
    QList<bool> results;

    result = true;

    for (qsizetype i = 0; i < axis.size(); ++i) {
        axis.at(i)->set();
        results.append(axis.at(i)->isStable());
        if (!axis.at(i)->isStable())
            result = false;
    }

    //qDebug() << "Autopilot Axis's status:" << results;

    return stable = result;
}

template<typename T>
AutopilotAxis<T>::AutopilotAxis(T accur, T(*pv)(), Aircraft *ac, void (Aircraft::*mv)(T), Control<T> *_control, Amean<T> *mean) :
    control(_control), get_pv(pv), set_mv(mv), PVmean(mean), m_pAC(ac), accuracy(accur)
{
    if (!control)
        control = new PID<T>;
}

template<typename T>
AutopilotAxis<T>::AutopilotAxis(T accur, T(*pv)(), Aircraft *ac, AutopilotAxis *chld, Control<T> *_control, Amean<T> *mean) :
    control(_control), get_pv(pv), PVmean(mean), m_pAC(ac), child(chld), accuracy(accur)
{
    if (!control)
        control = new PID<T>;
}

template<typename T>
AutopilotAxis<T>::~AutopilotAxis()
{
    if (control)
        delete control;
    if (PVmean)
        delete PVmean;
    if (child)
        delete child;
    control = nullptr;
    child = nullptr;
    PVmean = nullptr;
}

template<typename T>
bool AutopilotAxis<T>::scaleAxis()
{
    //PerfPlugin::pause();
    /*
    if (count < 1) {
        sp = 0.0f;
        setMv(0.0f);
    } else if (count < 2) {
        curE = get_pv();
        setMv(0.1f);
    } else if (count < 3) {
        setPosScale(0.1f * PerfPlugin::getEnvironment().q / (get_pv() - curE));
        curE = get_pv();
        setMv(0.0f);
    } else if (count < 4) {
        curE = get_pv();
        setMv(-0.1f);
    } else if (count < 5) {
        setNegScale(0.1f * PerfPlugin::getEnvironment().q / (get_pv() - curE));
        //setMv(pv += 0.1f);
        //PerfPlugin::pause();
        return true;
    }
    */
    if (count < 6) {
        sp = 0.0f;
        setMv(0.0f);
    } else if (count < 10) {
        curE = get_pv();
        setMv((count - 5) * 0.1f);
    } else if (count < 11) {
        setPosScale(0.1f * PerfPlugin::getEnvironment().q / (get_pv() - curE));
        curE = get_pv();
        setMv((14 - count) * 0.1f);
    } else if (count < 15) {
        curE = get_pv();
        setMv((14 - count) * 0.1f);
    } else if (count < 16) {
        setNegScale(-0.1f * PerfPlugin::getEnvironment().q / (get_pv() - curE));
        //setMv(pv += 0.1f);
        //PerfPlugin::pause();
        return true;
    } else
        return true;

    qDebug() << "Scale scale" << (float)curE << (float)get_pv();

    ++count;
    return false;
}

template<typename T>
bool AutopilotAxis<T>::set()
{
    T pv = 0.0f, error;

    //qDebug() << "AxisSet:" << scaled;

    if (!scaled && !scaleAxis())
        return false;

    sp = control->sp;
    //pv = (get_pv() < 0.0f ? get_pv() * neg_scale : get_pv() * pos_scale) / PerfPlugin::getEnvironment().q;
    pv = (get_pv() < 0.0f ? get_pv() * neg_scale : get_pv() * pos_scale) / PerfPlugin::getEnvironment().q;
    error = sp - pv;
    if (PVmean) {
        curE = PVmean->get(sp - get_pv());
        //curE = PVmean->get(error);
        stable = abs(curE) < accuracy && PVmean->valid();
    } else {
        curE = sp - get_pv();
        //curE = error;
        stable = abs(curE) < accuracy;
    }

    //qDebug() << "Axis: scaled pv, raw pv" << (float)pv << (float)get_pv() << "error" << (float)error << "neg scale" << (float)neg_scale << "pos scale" << (float)pos_scale;

    setMv(control->get(pv));

    //PerfPlugin::pause();
    return stable;

    /*
    if (child) {
        child->set(control->get(pv / scale));
    } else {
        set_mv(control->get(pv / scale));
    }
    */
    /*
    if (child) {
        child->set(filter.get(control.get(get_pv()/scale)));
    } else {
        set_mv(filter.get(control.get(get_pv()/scale)));
    }
    */
}

template<typename T>
void AutopilotAxis<T>::setMv(T pv)
{
    if (child) {
        child->set(pv);
    } else {
        //(m_pAC->*set_mv)(pv);
        std::invoke(set_mv, m_pAC, pv); //Same as above, but more readable.
    }
}

template<typename T>
void AutopilotAxis<T>::setScale(T _max, T _min)
{
    //scale = _max / (control->max - control->min);
    setPosScale(_max);
    setNegScale(_min);
    qDebug() << "Scale" << (float)neg_scale << (float)pos_scale;
    scaled = true;
}

template<typename T>
void AutopilotAxis<T>::setNegScale(T _min)
{
    //scale = _max / (control->max - control->min);
    neg_scale = _min;
    qDebug() << "Scale" << (float)neg_scale;
    neg_scaled = true;
    if (pos_scaled)
        scaled = true;
}

template<typename T>
void AutopilotAxis<T>::setPosScale(T _max)
{
    //scale = _max / (control->max - control->min);
    pos_scale = _max;
    qDebug() << "Scale" << (float)pos_scale;
    pos_scaled = true;
    if (neg_scaled)
        scaled = true;
}

template<typename T>
AutothrottleAxis<T>::AutothrottleAxis(T accur, T(*pv)(), Aircraft *ac, void (Aircraft::*mv)(T), Control<T> *_control, Amean<T> *mean) :
    AutopilotAxis<T>(accur, pv, ac, mv, _control, mean)
{
    //if (!control)
    //    control = new PID<T>;
}

template<typename T>
AutothrottleAxis<T>::~AutothrottleAxis()
{
    //AutopilotAxis<T>::~AutopilotAxis();
}

template<typename T>
bool AutothrottleAxis<T>::set()
{
    //Aircraft *ac = this->m_pAC;
    //TestPoint *tp = ac->curTP();
    //pointType t = tp->type;
    bool result1 = true;
    bool result2 = false;
    TestPoint *tp = nullptr;
    if (this->m_pAC)
        tp = this->m_pAC->curTP();

    //if (this->m_pAC && t == autoTHR)
    if (tp && tp->type == autoTHR) {
        if (tp->count < 2.0f) { //Scale for the current level
            T scale = PerfPlugin::getEnvironment().q / (this->m_pAC->getMaxThrust() - this->m_pAC->getMinThrust());
            this->neg_scale = scale;
            this->pos_scale = scale;
            this->neg_scaled = true;
            this->pos_scaled = true;
            this->scaled = true;
            this->control->reset(this->m_pAC->getThrottle());
        }

        result1 = AutopilotAxis<T>::set();
        //if (result)
        //    PerfPlugin::pause();
    }

    if (result1 || this->control->maxed()) {
        //T cur = PerfPlugin::getN1mean();
        //T cur = PerfPlugin::getN2mean(); //Fast
        //T cur = PerfPlugin::getEPRmean();
        T cur = PerfPlugin::getThrustTotal(); //Very accurate but slow
        //qDebug() << "Thrust stability" << (float)cur << (float)thrust << (cur == thrust);
        if (qAbs(cur - thrust) < 1.0f) //Thrust changed less that a Newton
            result2 = true;
        thrust = cur;
    }
    //qDebug() << "Throttle axis type" << this->m_pAC->curTP()->type << "N1mean" << PerfPlugin::getN1mean() << "N2mean" << PerfPlugin::getN2mean() << "EPRmean" << PerfPlugin::getEPRmean() << "Total thrust" << PerfPlugin::getThrustTotal() << "result" << result1 << result2;

    if (!result1 && result2) //result1 == false (power not set) && result2 == true (power is stable and maximum) -> not enough power to sustain level flight
        this->m_pAC->curTP()->state = low_thrust;

    return this->stable = result1 && result2;
    /*
    if (this->PVmean) {
        this->curE = this->PVmean->get(this->sp - get_pv());
        //curE = PVmean->get(error);
        stable = abs(this->curE) < this->accuracy && this->PVmean->valid();
    } else {
        this->curE = this->sp - get_pv();
        //curE = error;
        this->stable = abs(this->curE) < this->accuracy;
    }
    */
}

template class Autopilot<float>;
template class Autopilot<double>;
template class Autopilot<long double>;

template class AutopilotAxis<float>;
template class AutopilotAxis<double>;
template class AutopilotAxis<long double>;

template class AutothrottleAxis<float>;
template class AutothrottleAxis<double>;
template class AutothrottleAxis<long double>;
