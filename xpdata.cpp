#include "xpdata.h"

QHash<QByteArray, XPdata*> XPdata::allRefs = QHash<QByteArray, XPdata*>();
quint64 XPdata::m_uLoopctr = 0;

void XPdata::inc() {
    ++m_uLoopctr;
}

XPdata* XPdata::ref(QByteArray name)
{
    QByteArray realname = refNameWithoutModifiers(name);

    if (allRefs.contains(realname))
        return allRefs.value(realname);

    XPLMDataRef pRef = XPLMFindDataRef(realname.constData());

    if (pRef) {
        XPdata* pNewRef = new XPdata(pRef);
        allRefs.insert(realname, pNewRef);
        return pNewRef;
    }
    qDebug() << name;
    Q_ASSERT(pRef);
    return nullptr;
}

void XPdata::clearAll ()
{
    QHash<QByteArray, XPdata*>::iterator i = allRefs.begin();
    while(i != allRefs.end()) {
        delete i.value();
        i.value() = nullptr;
        i = allRefs.erase(i);
    }
}

//XPdata::XPdata(QByteArray name)
XPdata::XPdata(XPLMDataRef pRef)
{
    //QByteArray realname = refNameWithoutModifiers(name);

    m_pRef = pRef;
    m_iType = XPLMGetDataRefTypes(m_pRef);
    m_bRW = XPLMCanWriteDataRef(m_pRef);
}

QByteArray XPdata::refNameWithoutModifiers(QByteArray &original) {
    return original.contains(":") ? original.left(original.indexOf(":")) : original;
}

QVariant XPdata::get(QByteArray name)
{
    XPdata* data = ref(name);
    if (data)
       return data->get();
    return QVariant::fromValue(nullptr);
}

QList<int> XPdata::toListInt(QVariant v, qsizetype size)
{
    QList<int> buffer = QList<int>(size);
    QList<QVariant> data = v.toList();
    int newSize = size > data.size() ? data.size() : size;
    buffer.resize(newSize);

    //for (qsizetype i = 0; i < buffer.size(); ++i) {
    for (qsizetype i = 0; i < newSize; ++i) {
        buffer[i] = data.at(i).toInt();
    }

    return buffer;
}

QList<float> XPdata::toListFloat(QVariant v, qsizetype size)
{
    QList<float> buffer = QList<float>(size);
    QList<QVariant> data = v.toList();
    int newSize = size > data.size() ? data.size() : size;
    buffer.resize(newSize);

    //for (qsizetype i = 0; i < buffer.size(); ++i) {
    for (qsizetype i = 0; i < newSize; ++i) {
        buffer[i] = data.at(i).toFloat();
    }

    return buffer;
}

QVariant XPdata::toVariant(QList<int> v)
{
    QList<QVariant> newlist(0);
    newlist.reserve(v.size());
    for (qsizetype i = 0; i < v.size(); ++i) {
        newlist.append(v.at(i));
    }

    Q_ASSERT(v.size() == newlist.size());

    return newlist;
}

QVariant XPdata::toVariant(QList<float> v)
{
    QList<QVariant> newlist(0);
    newlist.reserve(v.size());
    for (qsizetype i = 0; i < v.size(); ++i) {
        newlist.append(v.at(i));
    }

    Q_ASSERT(v.size() == newlist.size());

    return newlist;
}


QVariant XPdata::get()
{
    if (!m_pRef)
        return QVariant::fromValue(nullptr);

    if (m_uCtr == m_uLoopctr)
        return m_data;

    m_uCtr = m_uLoopctr;

    if (m_iType & xplmType_Data) {
        m_iSize = XPLMGetDatab(m_pRef, nullptr, 0, 0);

        QByteArray buffer(nullptr, m_iSize);

        XPLMGetDatab(m_pRef, buffer.data(), 0, m_iSize);

        Q_ASSERT(m_iSize == buffer.size());
        m_data = buffer;
    } else if (m_iType & xplmType_FloatArray) {
        m_iSize = XPLMGetDatavf(m_pRef, nullptr, 0, 0);

        QList<float> buffer = QList<float>(m_iSize);

        m_iSize = XPLMGetDatavf(m_pRef, buffer.data(), 0, m_iSize);
        buffer.resize(m_iSize);

        m_data = toVariant(buffer);
    } else if (m_iType & xplmType_IntArray) {
        m_iSize = XPLMGetDatavi(m_pRef, nullptr, 0, 0);

        QList<int> buffer = QList<int>(m_iSize);

        m_iSize = XPLMGetDatavi(m_pRef, buffer.data(), 0, m_iSize);
        buffer.resize(m_iSize);

        m_data = toVariant(buffer);
    } else if (m_iType & xplmType_Double) {
        m_data = XPLMGetDatad(m_pRef);
    } else if (m_iType & xplmType_Float) {
        m_data = XPLMGetDataf(m_pRef);
    } else if (m_iType & xplmType_Int) {
        m_data = XPLMGetDatai(m_pRef);
    } else {
        QVariant::fromValue(nullptr);
    }

    return m_data;
}

int XPdata::geti()
{
    return get().toInt();
}

float XPdata::getf()
{
    return get().toFloat();

}

double XPdata::getd()
{
    return get().toDouble();

}

QList<QVariant> XPdata::getList()
{
    return get().toList();
}

QList<int> XPdata::getiList()
{
    get();
    return toListInt(m_data, m_iSize);
}

QList<float> XPdata::getfList()
{
    get();
    return toListFloat(m_data, m_iSize);
}

QByteArray XPdata::getArray()
{
    return get().toByteArray();
}

int XPdata::set(int value)
{
    return set(QVariant(value)).toInt();
}

float XPdata::set(float value)
{
    return set(QVariant(value)).toFloat();
}

double XPdata::set(double value)
{
    return set(QVariant(value)).toDouble();
}

QList<int> XPdata::set(QList<int> value)
{
    set(toVariant(value));

    return value;
}

QList<float> XPdata::set(QList<float> value)
{
    set(toVariant(value));

    return value;
}

/*
QByteArray XPdata::set(QByteArray value)
{
    set(value);

    return value;
}
*/

QVariant XPdata::set(QVariant value)
{
    if (!m_pRef)
        return QVariant::fromValue(nullptr);

    m_data = value;

    if (m_iType & xplmType_Data) {
        m_iSize = XPLMGetDatab(m_pRef, nullptr, 0, 0);
        //qDebug() << "XPdata type: byteArray =" << m_iType;

        QByteArray buffer = m_data.toByteArray();
        m_iSize = m_iSize > buffer.size() ? buffer.size() : m_iSize;

        XPLMSetDatab(m_pRef, buffer.data(), 0, m_iSize);
    } else if (m_iType & xplmType_FloatArray) {
        m_iSize = XPLMGetDatavf(m_pRef, nullptr, 0, 0);
        //qDebug() << "XPdata type: floatArray =" << m_iType;
        QList<float> list = toListFloat(m_data, m_iSize);
        m_iSize = list.size();

        XPLMSetDatavf(m_pRef, list.data(), 0, m_iSize);
        //XPLMSetDatavf(m_pRef, buffer.data(), 0, m_iSize);
    } else if (m_iType & xplmType_IntArray) {
        m_iSize = XPLMGetDatavi(m_pRef, nullptr, 0, 0);
        //qDebug() << "XPdata type: intArray =" << m_iType;
        QList<int> list = toListInt(m_data, m_iSize);
        m_iSize = list.size();

        XPLMSetDatavi(m_pRef, list.data(), 0, m_iSize);
        //XPLMSetDatavi(m_pRef, buffer.data(), 0, m_iSize);
    } else if (m_iType & xplmType_Double) {
        XPLMSetDatad(m_pRef, m_data.toDouble());
        //qDebug() << "XPdata type: double =" << m_iType;
    } else if (m_iType & xplmType_Float) {
        XPLMSetDataf(m_pRef, m_data.toFloat());
        //qDebug() << "XPdata type: float =" << m_iType;
    } else if (m_iType & xplmType_Int) {
        XPLMSetDatai(m_pRef, m_data.toInt());
        //qDebug() << "XPdata type: int =" << m_iType;
    } else {
        QVariant::fromValue(nullptr);
    }

    return m_data;
}

QHash<QByteArray, QVariant> *XPdata::get(QHash<QByteArray, QVariant> *hash)
{
    QHash<QByteArray, QVariant>::iterator i = hash->begin();
    while (i != hash->end()) {
        //(*hash)[i.key()] = XPdata::ref(i.key())->get();
        i.value() = XPdata::ref(i.key())->get();
        ++i;
    }

    return hash;
}

const QHash<QByteArray, QVariant> *XPdata::set(const QHash<QByteArray, QVariant> *hash)
{
    QHash<QByteArray, QVariant>::const_iterator i = hash->constBegin();
    while (i != hash->constEnd()) {
        //qDebug() << "key = " << i.key() << "value =" << i.value();
        XPdata::ref(i.key())->set(i.value());
        ++i;
    }

    return hash;
}
