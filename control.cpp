#include "control.h"

#include <QDebug>

//template class Filter<float>;
//template class Filter<double>;
//template class Filter<long double>;

template<typename T>
T Amean<T>::get(T _new) //calculate new value
{
    T add = 0;
    if (!valid())
        current *= samples.size()/(samples.size() + 1.0); //Let mean grow to the wanted size.

    samples.prepend(_new);
    if (samples.size() > size) {
        add = -samples.last();
        samples.resize(size); //pop last away
    }

    return current = (add + _new) / samples.size(); //add new to current value.
}

template class Amean<float>;
template class Amean<double>;
template class Amean<long double>;

template<typename T>
void EMA<T>::setWeight(T procent) //set k=num term according to m
{
    qDebug() << "EMA setWeight: procent =" << (float)procent << "--------------------------------------------";
    qDebug() << "m =" << (float)m;
    if (procent > 100.0)
        return;

    qDebug() << "m =" << (float)m;
    qDebug() << "ln(1.0 - procent/100.0) =" << qLn(1.0 - (float)procent/100.0);
    qDebug() << "qLn(1.0 - m) =" << qLn(1.0 - (float)m);
    qDebug() << "ln(1.0 - procent/100.0) / qLn(1.0 - m) =" << qLn(1.0 - (float)procent/100.0) / qLn(1.0 - (float)m);
    qDebug() << "ceil(res) =" << ceil(qLn(1.0 - (float)procent/100.0) / qLn(1.0 - (float)m));

    setTerms(qCeil(qLn(1.0 - procent/100.0) /
              qLn(1.0 - m)));
}

template<typename T>
void EMA<T>::setN(T _n) //sets m according to type.
{
    qDebug() << "EMA setN: _n =" << (float)_n << "------------------------------------------------------";
    if (type == traditional)
        m = 2.0 / ( _n + 1.0 );
    if (type == wilder)
        m = 1.0 / _n;
    qDebug() << "m =" << (float)m;
}

template<typename T>
void EMA<T>::setTerms(qsizetype numTerms) //set k
{
    qDebug() << "EMA setTerms: numTerms =" << numTerms << "------------------------------------------------";
    qDebug() << "m =" << (float)m;
    k = numTerms;
    if (k < 1)
        k = 1;

    qDebug() << "k =" << (float)k;
    qDebug() << "m =" << (float)m;

    //multip.reserve(k);
    multip.resize(k);
    multip.squeeze();

    for (qsizetype i = 0; i < k; ++i) {
        multip[i] = qPow(1 - m, i);
    }
}

template<typename T>
T EMA<T>::get(T _new) //calculate new value
{
    terms.prepend(_new);
    qsizetype size = terms.size();
    //qDebug() << "EMA get: _new = " << (float)_new <<  "------------------------------------------------------";
    //qDebug() << "size =" << size;
    //qDebug() << "k =" << k;

    if (size > k) {
        terms.resize(k);
        size = k;
    }

    //qDebug() << "currrent =" << (float)current;
    //qDebug() << "k =" << k;
    //qDebug() << "m =" << (float)m;
    //qDebug() << "terms.at(0) =" << (float)terms.at(0);

    if (k == 1) {
        if (size == 1)
            current = terms.at(0);
        else
            current = m * terms.at(0) + (1 - m) * current;
    } else {
        T sum = 0.0;
        T count = 0.0;

        for (qsizetype i = 0; i < size; ++i) {
            sum += multip.at(i) * terms.at(i);
            count += multip.at(i);
        }

        current = sum / count;
    }

    //qDebug() << "current =" << (float)current;

    return current;
}

template class EMA<float>;
template class EMA<double>;
template class EMA<long double>;

template<typename T>
LPFilter<T>::LPFilter(T SampleFrequency, T Cutoff, T out) :
    m_Val(out)
{
    setSampleFrequency(SampleFrequency);
    setCutoffFrequency(Cutoff);
}

template<typename T>
void LPFilter<T>::setSampleFrequency(T SampleFrequency)
{
    m_T = 1 / SampleFrequency;
}

template<typename T>
void LPFilter<T>::setCutoffFrequency(T Cutoff)
{
    m_Beta = qExp( -(2.0 * M_PI * Cutoff) * m_T); //calculate -rad/s * samplte time
}

template<typename T>
T LPFilter<T>::get(T val)
{
    return m_Val = val + m_Beta * (m_Val - val);
    // m_Val = m_Beta * m_Val + (1 - m_Beta) * val;
}

template<typename T>
void LPFilter<T>::set(T val)
{
    m_Val = val;
}

template class LPFilter<float>;
template class LPFilter<double>;
template class LPFilter<long double>;

template<typename T>
PID<T>::~PID()
{
    if (KpFilter)
        delete(KpFilter);
    if (KiFilter)
        delete(KiFilter);
    if (KdFilter)
        delete(KdFilter);

    KpFilter = nullptr;
    KiFilter = nullptr;
    KdFilter = nullptr;
}

template<typename T>
PID<T>::PID(T S, T P, T I, T D, T DT, T M, T N) : Control<T>(S, M, N), Kp(P), Ki(I), Kd(D), dt(DT)
{

}

template<typename T>
T PID<T>::get(T pv)
{
    T error;
    T p;
    T d;
    T mv;

    error = Control<T>::sp - pv;
    p = error;
    i += error * dt;
    d = (error - prev_error) / dt;
    prev_error = error;

    if (KpFilter)
        p = KpFilter->get(p);
    if (KiFilter)
        i = KiFilter->get(i);
    if (KdFilter)
        d = KdFilter->get(d);

    mv = Kp * p + Ki * i + Kd * d;
    //qDebug() << "PID Control, pv =" << (float)pv << "\tsp" << (float)Control<T>::sp << "mv =" << (float)mv << "p =" << (float)p << "i =" << (float)i << "d =" << (float)d;

    if (Control<T>::min > Control<T>::max)
        return mv;

    if (mv > Control<T>::max)
        return Control<T>::max;
    else if (mv < Control<T>::min)
        return Control<T>::min;
    return mv;

    //return Kp * p + Ki * i + Kd * d;
}

template<typename T>
PoM<T>::PoM(T S, T P, T I, T D, T DT, T M, T N) : PID<T>(S, P, I, D, DT, M, N)
{

}

template<typename T>
T PoM<T>::get(T pv)
{
    T error;
    //static T p;
    T d;
    T mv = 0.0f;
    //T last_mv = 0.0f;
    T PoM;

    if (first) {
        pvInit = pv;
        last_pv = pv;
    }
    first = false;

    PoM = pvInit - pv;
    error = Control<T>::sp - pv;
    //p = error;
    PID<T>::i += error * PID<T>::dt;
    //d = (error - PID<T>::prev_error) / PID<T>::dt; //derivative of error: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
    d = - (pv - last_pv) / PID<T>::dt; //derivative on Measurement: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
    PID<T>::prev_error = error;
    last_pv = pv;

    if (PID<T>::KpFilter)
        PoM = PID<T>::KpFilter->get(PoM);
    if (PID<T>::KiFilter)
        PID<T>::i = PID<T>::KiFilter->get(PID<T>::i);
    if (PID<T>::KdFilter)
        d = PID<T>::KdFilter->get(d);

    mv = PID<T>::Kp * PoM + PID<T>::Ki * PID<T>::i + PID<T>::Kd * d;
    //qDebug() << "PoM Control, pv =" << (float)pv << "\tsp" << (float)Control<T>::sp << "\tmv =" << (float)mv << "\tPoM =" << (float)PoM << "\tpvInit =" << (float)pvInit << "\ti =" << (float)PID<T>::i << "\td =" << (float)d;

    if (fabs(mv - this->last_mv) > 0.1f) {
        if (mv > this->last_mv)
            mv = this->last_mv + 0.1f;
        else
            mv = this->last_mv - 0.1f;
    }

    this->last_mv = mv;

    if (Control<T>::min > Control<T>::max)
        return mv;

    if (mv > Control<T>::max)
        return Control<T>::max;
    else if (mv < Control<T>::min)
        return Control<T>::min;
    return mv;

    //return Kp * p + Ki * i + Kd * d;
}

template<typename T>
MNY<T>::MNY(T S, T M, T N) : Control<T>(S, M, N)
{

}

template<typename T>
T MNY<T>::get(T pv)
{
    T cur_trend = pv - last_pv;

    if (pv < lo_pv || pv > hi_pv) {
        hi_valid = false;
        lo_valid = false;
        //count = 0;
    }

    if (last_trend > 0.0f && cur_trend < 0.0f) { //hi peak
        //if (error > 0.0f) { //postive error, mv needs to be higher!!!
        //}
        hi_pv = last_pv;
        hi_valid = true; //Hi oscillation has occured
    } else if (last_trend < 0.0f && cur_trend > 0.0f) { //lo peak
        //if (error < 0.0f) { //negative error, mv needs to be lower!!!
        //}
        lo_pv = last_pv;
        lo_valid = true; //Lo oscillation has occured
    }

    T mean = (lo_pv + hi_pv) / 2.0f;
    T error = Control<T>::sp - pv;
    //T error = Control<T>::sp - mean;

    if (qAbs(error) > 0.001f && count > 1) {
        if (last_error)
            coef = qAbs(last_error / (last_error - error));
        last_error = error;
        this->last_mv += last_error * coef;
        last_mean = mean;
        hi_valid = false;
        lo_valid = false;
        count = 0;
    } else if (hi_valid && lo_valid && count > 4) {
        error = Control<T>::sp - mean;
        if (last_error)
            coef = qAbs(last_error / (last_error - error));
        last_error = error * coef;
        this->last_mv += last_error;
        last_mean = mean;
        hi_valid = false;
        lo_valid = false;
        count = 0;
    }

    last_pv = pv;
    last_trend = cur_trend;
    ++count;

    if (this->last_mv > Control<T>::max || this->last_mv < Control<T>::min) {
        coef = 1.0f;
        this->last_mv = error;
        count = 0;
    }

    //qDebug() << "Control" << (void*) this << "pv" << (float)pv << (float)Control<T>::sp << "mv" << (float)this->last_mv  << "HiLo" << hi_valid << lo_valid << "hi" << (float)hi_pv << "lo" << (float)lo_pv << "count" << count << "trend" << (float)cur_trend << "Last_E" << (float)last_error << "Coef" << (float)coef << "Mean" << (float)last_mean;
    return this->last_mv;
}

template<typename T>
MNY2<T>::MNY2(T S, T M, T N) : Control<T>(S, M, N)
{
    max_step = (M - N) / 20.0f;
}

template<typename T>
T MNY2<T>::get(T _pv)
{
    T pv = _pv;
    T mv = this->last_mv;
    if (inFilter)
        pv = inFilter->get(_pv);

    T error = Control<T>::sp - pv;
    if (error < 0.0f && positive) { //Dampen oscillation
        stabilizer *= 2;
        positive = false;
    } else if (error > 0.0f && !positive) {
        stabilizer *= 2;
        positive = true;
    }

    step = error / devider / stabilizer;

    stabilizer /= pow(2.0f, 1.0f/10.0f);
    if (stabilizer < 1.0f)
        stabilizer = 1.0f;
    if (stabilizer > 128.0f)
        stabilizer = 128.0f;

    if (qAbs(step) > max_step)
        step = step / qAbs(step) * max_step;

    if (outFilter)
        step = outFilter->get(step);

    mv += step;
    if (mv > Control<T>::max)
        mv = Control<T>::max;
    else if (mv < Control<T>::min)
        mv = Control<T>::min;

    //qDebug() << "MNY2 controller, pv" << (float)pv << "_pv" << (float)_pv << "error" << (float)error << "step" << (float)step << "mv" << (float)mv;

    return this->last_mv = mv;
    //return mv;
}

template<typename T>
int MNY2<T>::maxed()
{
    int res = Control<T>::maxed();

    if (!res)
        return res;

    if (res > 0 && step > 0.0f)
        return res;
    if (res < 0 && step < 0.0f)
        return res;
    return 0;
}


template class Control<float>;
template class Control<double>;
template class Control<long double>;

template class PID<float>;
template class PID<double>;
template class PID<long double>;

template class PoM<float>;
template class PoM<double>;
template class PoM<long double>;

template class MNY<float>;
template class MNY<double>;
template class MNY<long double>;

template class MNY2<float>;
template class MNY2<double>;
template class MNY2<long double>;
