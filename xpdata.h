#ifndef XPDATA_H
#define XPDATA_H

#include <XPLMDataAccess.h>

#include <QByteArray>
#include <QVariant>

class XPdata
{
public:
    static XPdata* ref(QByteArray name);
    static QVariant get(QByteArray name);
    static void clearAll();
    static void inc();

    static QByteArray refNameWithoutModifiers(QByteArray &original);

    static QList<int> toListInt(QVariant v, qsizetype size);
    static QList<float> toListFloat(QVariant v, qsizetype size);
    static QVariant toVariant(QList<int> v);
    static QVariant toVariant(QList<float> v);

    QVariant get();
    int geti();
    float getf();
    double getd();
    QList<int> getiList();
    QList<float> getfList();
    QList<QVariant> getList();
    QByteArray getArray();
    static QHash<QByteArray, QVariant> *get(QHash<QByteArray, QVariant> *hash);

    //XPLMDataRef getRef()
    //{ return m_pRef; }

    int set(const int value);
    float set(const float value);
    double set(const double value);
    QList<int> set(const QList<int> value);
    QList<float> set(const QList<float> value);
    QVariant set(const QVariant value);
    static const QHash<QByteArray, QVariant> *set(const QHash<QByteArray, QVariant> *hash);

    int operator=(int value)
    { return set (value); }
    float operator=(float value)
    { return set (value); }
    double operator=(double value)
    { return set (value); }
private:
    XPdata(XPLMDataRef pRef);

    bool m_bRW = false;
    quint64 m_uCtr;
    QVariant m_data;
    int m_iSize = 1;
    XPLMDataTypeID m_iType = xplmType_Unknown;
    XPLMDataRef m_pRef = nullptr;

    static QHash<QByteArray, XPdata*> allRefs;
    static quint64 m_uLoopctr;
};

#endif // XPDATA_H
