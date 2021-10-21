#ifndef CONTROL_H
#define CONTROL_H

#include <QtMath>
#include <QDebug>

template<typename T>
class Filter
{
public:
    Filter() {};
    virtual ~Filter() {};
    virtual void reset() = 0;
    virtual T get(T val) = 0;
    virtual T get() = 0;
    virtual bool valid() = 0;
};

//Average
template<typename T = float>
class Amean : public Filter<T>
{
public:
    Amean(qsizetype _size) : size(_size) { if (size < 1) size = 1; }
    virtual void reset()
    { samples.clear(); }

    virtual T get(T _new); //calculate new value
    virtual T get() //return current
    { return current; }
    virtual bool valid()
    { return samples.size() == size; }

private:
    QList<T> samples;
    qsizetype size;
    T current = 0;
};

//Exponential moving average from finite to infinity
template<typename T = float>
class EMA : public Filter<T>
{
public:
    enum EMAtype {
        traditional = 0,
        wilder
    };

    EMA(T _n, T _weight, EMAtype _type = traditional) : EMA(_n, _type)
    { setWeight(_weight); }
    EMA(T _n, qsizetype _terms, EMAtype _type = traditional) : EMA(_n, _type)
    { setTerms(_terms); }
    EMA(T _n, EMAtype _type = traditional)
    { reset(); setN(_n, _type); }
    virtual ~EMA() {};
    virtual void reset()
    { terms.clear(); multip.clear(); }

    void setWeight(T procent); //set k=num term according to m
    void setTerms(qsizetype numTerms); //set k
    void setN(T _n); //sets m according to type.
    void setN(T _n, EMAtype newtype) //sets m according to type.
    { type = newtype; setN(_n); }

    virtual T get(T _new); //calculate new value
    virtual T get() //return current
    { return current; }
    virtual bool valid()
    { return terms.size() == k; }

private:
    QList<T> terms;
    QList<T> multip;
    T current = 0;
    T previous = 0;
    qsizetype k = -1; // number of terms in finite series
    //T n; //
    T m = 0; // m=1/n (Wilder) or m=2/(n+1) (traditional)
    EMAtype type = traditional;
};


//First order Low Pass Filter
template<typename T = float>
class LPFilter : public Filter<T>
{
public:
    LPFilter(T SampleFrequency = 20.0f, T Cutoff = 10.0f, T out = 0);
    virtual ~LPFilter() {};

    void setSampleFrequency(T SampleFrequency);
    void setCutoffFrequency(T Cutoff);
    virtual T get(T val);
    virtual T get()
    { return m_Val; };
    void set(T val);

private:
    T m_Beta; // Beta = e^-wT
    T m_T; // T = time between samples
    T m_Val; // Output
};

/*
//Second order Low Pass Filter
template<typename T =float>
class LPFilter2 : public Filter<T>
{
public:
    LPFilter2(T SampleFrequency, T Cutoff, T out = 0);
    void setSampleFrequency(T SampleFrequency);
    void setCutoffFrequency(T Cutoff);
    T get(T val);
    T current() { return m_Val; };
    void set(T val);

private:
    T m_Val;
};
*/

#pragma push_macro("min")
#pragma push_macro("max")
#undef max
#undef min
template<typename T = float>
class Control
{
public:
    Control(T _sp, T M, T N) : max(M), min(N), sp(_sp) {};
    virtual ~Control() {};
    virtual T get(T pv) = 0;
    virtual T get(T pv, T _sp) = 0;
    virtual void reset() = 0;
    virtual void reset(T mv)
    { last_mv = mv; }

    virtual T cur()
    { return last_mv; }
    virtual int maxed()
    { if (max < min) return 0; else if (last_mv == max) return 1; else if (last_mv == min) return -1; return 0;  }

    T max = -1.0f;
    T min = 0.0f;
    T sp = 0.0f;
    T last_mv = 0.0f;
};
#pragma pop_macro("min")
#pragma pop_macro("max")

template<typename T = float>
class PID : public Control<T> //PID controller
{
public:
    PID(T S = 0.0f, T P = 1.0f, T I = 1.0f, T D = 1.0f, T DT = 1.0f, T M = 0.0f, T N = 0.0f);
    virtual ~PID();

    virtual T get(T pv);
    virtual T get(T pv, T _sp)
    { Control<T>::sp = _sp; return get(pv); }
    virtual void reset()
    { i = 0.0f; }
    virtual void reset(T _err)
    { prev_error = _err; reset(); }

    T Kp = 1.0f;
    T Ki = 1.0f;
    T Kd = 1.0f;
    T dt = 1.0f;
    Filter<T> *KpFilter = nullptr;
    Filter<T> *KiFilter = nullptr;
    Filter<T> *KdFilter = nullptr;

protected:
    T prev_error = 0.0f;
    T i = 0.0f;
};

template<typename T = float>
class PoM : public PID<T> //PoM controller
{
public:
    PoM(T S = 0.0f, T P = 1.0f, T I = 1.0f, T D = 1.0f, T DT = 1.0f, T M = 0.0f, T N = 0.0f);
    virtual ~PoM() {};

    virtual T get(T pv);
    virtual T get(T pv, T _sp)
    { Control<T>::sp = _sp; return get(pv); }
    virtual void reset()
    { first = true; }
    virtual void reset(T pv)
    { last_pv = pv; reset(); }

private:
    T pvInit = 0;
    bool first = true;
    //T last_mv = 0.0f;
    T last_pv = 0.0f;
};

template<typename T = float>
class MNY : public Control<T> //MNY controller
{
public:
    MNY(T S, T M, T N);
    virtual ~MNY() {};

    virtual T get(T pv);
    virtual T get(T pv, T _sp)
    { Control<T>::sp = _sp; return get(pv); }
    virtual void reset()
    { hi_valid = false; lo_valid = false; count = 0; }
    virtual void reset(T mv)
    { this->reset(mv); reset(); }


private:
    T last_pv = 0.0f;
    T hi_pv = 0.0f;
    T lo_pv = 0.0f;
    bool hi_valid = false;
    bool lo_valid = false;

    T coef = 1.0f;
    T last_error = 0.0f;
    T last_trend = 0.0f;
    T last_mean = 0.0f;
    //T last_mv = 0.0f;
    int count = 0;
};

template<typename T = float>
class MNY2 : public Control<T> //MNY2 controller
{
public:
    MNY2(T S, T M, T N);
    virtual ~MNY2() {};

    virtual T get(T pv);
    virtual T get(T pv, T _sp)
    { Control<T>::sp = _sp; return get(pv); }
    virtual void reset()
    { step = 0.0f; stabilizer = 1.0f; if (inFilter) inFilter->reset(); if (outFilter) outFilter->reset(); }
    virtual void reset(T _mv)
    { Control<T>::reset(_mv); positive = (_mv > ((this->max + this->min) / 2.0f)) ? false : true; reset(); }

    virtual int maxed();

    void setInFilter(Filter<T> *fil)
    { inFilter = fil; }
    void setOutFilter(Filter<T> *fil)
    { outFilter = fil; }
    void setDevider(T v)
    { devider = v; }

private:
    Filter<T> *inFilter = nullptr;
    Filter<T> *outFilter = nullptr;

    T max_step = 0.0f;
    T step = 0.0f;
    T devider = 5.0f;
    T stabilizer = 1.0f;
    bool positive = true;
    //T last_sp = 0.0f;
    //T mv = 0.0f;
};


#endif // CONTROL_H
