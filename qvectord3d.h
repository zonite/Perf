/****************************************************************************
**
** Copyright (C) 2020 The Qt Company Ltd.
** Copyright (C) 2020 Klarälvdalens Datakonsult AB, a KDAB Group company, info@kdab.com, author Giuseppe D'Angelo <giuseppe.dangelo@kdab.com>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtGui module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 2.0 or (at your option) the GNU General
** Public license version 3 or any later version approved by the KDE Free
** Qt Foundation. The licenses are as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL2 and LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-2.0.html and
** https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QVECTORD3D_H
#define QVECTORD3D_H

#include <QtGui/qtguiglobal.h>
#include <QtCore/qpoint.h>
#include <QtCore/qrect.h>
#include <QtCore/qmath.h>
#include <QtGui/QVector3D>
#include <QDebug>

QT_BEGIN_NAMESPACE

//class QVector2D;
//class QVector3D;
class QVectorD3D;
//class QVector4D;
//class QMatrix4x4;
//class QVariant;

/***************************** QVectorD3D *****************************/

#ifndef QT_NO_VECTORD3D

class QVectorD3D
{
public:
    constexpr QVectorD3D() noexcept;
    explicit QVectorD3D(Qt::Initialization) noexcept {}
    constexpr QVectorD3D(double xpos, double ypos, double zpos) noexcept : v{xpos, ypos, zpos} {}

    constexpr explicit QVectorD3D(QPoint point) noexcept;
    constexpr explicit QVectorD3D(QPointF point) noexcept;
#ifndef QT_NO_VECTOR2D
    constexpr explicit QVectorD3D(QVector2D vector) noexcept;
    constexpr QVectorD3D(QVector2D vector, double zpos) noexcept;
#endif
    constexpr QVectorD3D(QVector3D vector) noexcept;
#ifndef QT_NO_VECTOR4D
    constexpr explicit QVectorD3D(QVector4D vector) noexcept;
#endif

    constexpr bool isNull() const noexcept;

    constexpr double x() const noexcept;
    constexpr double y() const noexcept;
    constexpr double z() const noexcept;

    constexpr void setX(double x) noexcept;
    constexpr void setY(double y) noexcept;
    constexpr void setZ(double z) noexcept;

    constexpr double &operator[](int i);
    constexpr double operator[](int i) const;

    [[nodiscard]] double length() const noexcept;
    [[nodiscard]] constexpr double lengthSquared() const noexcept;

    [[nodiscard]] QVectorD3D normalized() const noexcept;
    void normalize() noexcept;

    constexpr QVectorD3D &operator+=(QVectorD3D vector) noexcept;
    constexpr QVectorD3D &operator-=(QVectorD3D vector) noexcept;
    constexpr QVectorD3D &operator*=(double factor) noexcept;
    constexpr QVectorD3D &operator*=(QVectorD3D vector) noexcept;
    constexpr QVectorD3D &operator/=(double divisor);
    constexpr QVectorD3D &operator/=(QVectorD3D vector);

    [[nodiscard]] static constexpr double dotProduct(QVectorD3D v1, QVectorD3D v2) noexcept;
    [[nodiscard]] static constexpr QVectorD3D crossProduct(QVectorD3D v1, QVectorD3D v2) noexcept;

    [[nodiscard]] static QVectorD3D normal(QVectorD3D v1, QVectorD3D v2) noexcept;
    [[nodiscard]] static QVectorD3D normal(QVectorD3D v1, QVectorD3D v2, QVectorD3D v3) noexcept;

    Q_GUI_EXPORT QVectorD3D project(const QMatrix4x4 &modelView, const QMatrix4x4 &projection, const QRect &viewport) const;
    Q_GUI_EXPORT QVectorD3D unproject(const QMatrix4x4 &modelView, const QMatrix4x4 &projection, const QRect &viewport) const;

QT_WARNING_PUSH
QT_WARNING_DISABLE_FLOAT_COMPARE
    constexpr friend inline bool operator==(QVectorD3D v1, QVectorD3D v2) noexcept
    {
        return v1.v[0] == v2.v[0] && v1.v[1] == v2.v[1] && v1.v[2] == v2.v[2];
    }

    constexpr friend inline bool operator!=(QVectorD3D v1, QVectorD3D v2) noexcept
    {
        return v1.v[0] != v2.v[0] || v1.v[1] != v2.v[1] || v1.v[2] != v2.v[2];
    }
QT_WARNING_POP
    double distanceToPoint(QVectorD3D point) const noexcept;
    constexpr double distanceToPlane(QVectorD3D plane, QVectorD3D normal) const noexcept;
    double distanceToPlane(QVectorD3D plane1, QVectorD3D plane2, QVectorD3D plane3) const noexcept;
    double distanceToLine(QVectorD3D point, QVectorD3D direction) const noexcept;


    constexpr friend inline QVectorD3D operator+(QVectorD3D v1, QVectorD3D v2) noexcept
    {
        return QVectorD3D(v1.v[0] + v2.v[0], v1.v[1] + v2.v[1], v1.v[2] + v2.v[2]);
    }

    constexpr friend inline QVectorD3D operator-(QVectorD3D v1, QVectorD3D v2) noexcept
    {
        return QVectorD3D(v1.v[0] - v2.v[0], v1.v[1] - v2.v[1], v1.v[2] - v2.v[2]);
    }

    constexpr friend inline QVectorD3D operator*(double factor, QVectorD3D vector) noexcept
    {
        return QVectorD3D(vector.v[0] * factor, vector.v[1] * factor, vector.v[2] * factor);
    }

    constexpr friend inline QVectorD3D operator*(QVectorD3D vector, double factor) noexcept
    {
        return QVectorD3D(vector.v[0] * factor, vector.v[1] * factor, vector.v[2] * factor);
    }

    constexpr friend inline QVectorD3D operator*(QVectorD3D v1, QVectorD3D v2) noexcept
    {
        return QVectorD3D(v1.v[0] * v2.v[0], v1.v[1] * v2.v[1], v1.v[2] * v2.v[2]);
    }

    constexpr friend inline QVectorD3D operator-(QVectorD3D vector) noexcept
    {
        return QVectorD3D(-vector.v[0], -vector.v[1], -vector.v[2]);
    }

    constexpr friend inline QVectorD3D operator/(QVectorD3D vector, double divisor)
    {
        Q_ASSERT(divisor < 0 || divisor > 0);
        return QVectorD3D(vector.v[0] / divisor, vector.v[1] / divisor, vector.v[2] / divisor);
    }

    constexpr friend inline QVectorD3D operator/(QVectorD3D vector, QVectorD3D divisor)
    {
        Q_ASSERT(divisor.v[0] > 0 || divisor.v[0] < 0);
        Q_ASSERT(divisor.v[1] > 0 || divisor.v[1] < 0);
        Q_ASSERT(divisor.v[2] > 0 || divisor.v[2] < 0);
        return QVectorD3D(vector.v[0] / divisor.v[0], vector.v[1] / divisor.v[1],
                         vector.v[2] / divisor.v[2]);
    }

    friend Q_GUI_EXPORT bool qFuzzyCompare(QVectorD3D v1, QVectorD3D v2) noexcept;

#ifndef QT_NO_VECTOR2D
    constexpr QVector2D toVector2D() const noexcept;
#endif
    constexpr QVector3D toVector3D() const noexcept;
#ifndef QT_NO_VECTOR4D
    constexpr QVector4D toVector4D() const noexcept;
#endif

    constexpr QPoint toPoint() const noexcept;
    constexpr QPointF toPointF() const noexcept;

    Q_GUI_EXPORT operator QVariant() const;

private:
    double v[3];

    friend class QVector2D;
    friend class QVector4D;
#ifndef QT_NO_MATRIX4X4
    friend QVectorD3D operator*(const QVectorD3D& vector, const QMatrix4x4& matrix);
    friend QVectorD3D operator*(const QMatrix4x4& matrix, const QVectorD3D& vector);
#endif

    template <std::size_t I,
              typename V,
              std::enable_if_t<(I < 3), bool> = true,
              std::enable_if_t<std::is_same_v<std::decay_t<V>, QVectorD3D>, bool> = true>
    friend constexpr decltype(auto) get(V &&vec) noexcept
    {
        if constexpr (I == 0)
            return (std::forward<V>(vec).v[0]);
        else if constexpr (I == 1)
            return (std::forward<V>(vec).v[1]);
        else if constexpr (I == 2)
            return (std::forward<V>(vec).v[2]);
    }

};

Q_DECLARE_TYPEINFO(QVectorD3D, Q_PRIMITIVE_TYPE);

#endif // QT_NO_VECTORD3D


/***************************** QVectorD3D *****************************/

#ifndef QT_NO_VECTOR3D

constexpr inline QVectorD3D::QVectorD3D() noexcept : v{0.0, 0.0, 0.0} {}

constexpr inline QVectorD3D::QVectorD3D(QPoint point) noexcept : v{double(point.x()), double(point.y()), 0.0} {}

constexpr inline QVectorD3D::QVectorD3D(QPointF point) noexcept : v{double(point.x()), double(point.y()), 0.0} {}

#ifndef QT_NO_VECTOR2D
constexpr inline QVectorD3D::QVectorD3D(QVector2D vector) noexcept : v{vector[0], vector[1], 0.0} {}
constexpr inline QVectorD3D::QVectorD3D(QVector2D vector, double zpos) noexcept : v{vector[0], vector[1], zpos} {}
#endif

constexpr inline QVectorD3D::QVectorD3D(QVector3D vector) noexcept : v{vector[0], vector[1], vector[2]} {}

#ifndef QT_NO_VECTOR4D
constexpr inline QVectorD3D::QVectorD3D(QVector4D vector) noexcept : v{vector[0], vector[1], vector[2]} {}
#endif

constexpr inline bool QVectorD3D::isNull() const noexcept
{
    return qIsNull(v[0]) && qIsNull(v[1]) && qIsNull(v[2]);
}

constexpr inline double QVectorD3D::x() const noexcept { return v[0]; }
constexpr inline double QVectorD3D::y() const noexcept { return v[1]; }
constexpr inline double QVectorD3D::z() const noexcept { return v[2]; }

constexpr inline void QVectorD3D::setX(double aX) noexcept { v[0] = aX; }
constexpr inline void QVectorD3D::setY(double aY) noexcept { v[1] = aY; }
constexpr inline void QVectorD3D::setZ(double aZ) noexcept { v[2] = aZ; }

constexpr inline double &QVectorD3D::operator[](int i)
{
    Q_ASSERT(uint(i) < 3u);
    return v[i];
}

constexpr inline double QVectorD3D::operator[](int i) const
{
    Q_ASSERT(uint(i) < 3u);
    return v[i];
}

inline double QVectorD3D::length() const noexcept
{
    return qHypot(v[0], v[1], v[2]);
}

inline QVectorD3D QVectorD3D::normalized() const noexcept
{
    const double len = length();
    return qFuzzyIsNull(len - 1.0f) ? *this : qFuzzyIsNull(len) ? QVectorD3D()
        : QVectorD3D(v[0] / len, v[1] / len, v[2] / len);
}

inline void QVectorD3D::normalize() noexcept
{
    const double len = length();
    if (qFuzzyIsNull(len - 1.0f) || qFuzzyIsNull(len))
        return;

    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}

constexpr inline double QVectorD3D::lengthSquared() const noexcept
{
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

constexpr inline QVectorD3D &QVectorD3D::operator+=(QVectorD3D vector) noexcept
{
    v[0] += vector.v[0];
    v[1] += vector.v[1];
    v[2] += vector.v[2];
    return *this;
}

constexpr inline QVectorD3D &QVectorD3D::operator-=(QVectorD3D vector) noexcept
{
    v[0] -= vector.v[0];
    v[1] -= vector.v[1];
    v[2] -= vector.v[2];
    return *this;
}

constexpr inline QVectorD3D &QVectorD3D::operator*=(double factor) noexcept
{
    v[0] *= factor;
    v[1] *= factor;
    v[2] *= factor;
    return *this;
}

constexpr inline QVectorD3D &QVectorD3D::operator*=(QVectorD3D vector) noexcept
{
    v[0] *= vector.v[0];
    v[1] *= vector.v[1];
    v[2] *= vector.v[2];
    return *this;
}

constexpr inline QVectorD3D &QVectorD3D::operator/=(double divisor)
{
    Q_ASSERT(divisor < 0 || divisor > 0);
    v[0] /= divisor;
    v[1] /= divisor;
    v[2] /= divisor;
    return *this;
}

constexpr inline QVectorD3D &QVectorD3D::operator/=(QVectorD3D vector)
{
    Q_ASSERT(vector.v[0] > 0 || vector.v[0] < 0);
    Q_ASSERT(vector.v[1] > 0 || vector.v[1] < 0);
    Q_ASSERT(vector.v[2] > 0 || vector.v[2] < 0);
    v[0] /= vector.v[0];
    v[1] /= vector.v[1];
    v[2] /= vector.v[2];
    return *this;
}

constexpr inline double QVectorD3D::dotProduct(QVectorD3D v1, QVectorD3D v2) noexcept
{
    return v1.v[0] * v2.v[0] + v1.v[1] * v2.v[1] + v1.v[2] * v2.v[2];
}

constexpr inline QVectorD3D QVectorD3D::crossProduct(QVectorD3D v1, QVectorD3D v2) noexcept
{
    return QVectorD3D(v1.v[1] * v2.v[2] - v1.v[2] * v2.v[1],
                     v1.v[2] * v2.v[0] - v1.v[0] * v2.v[2],
                     v1.v[0] * v2.v[1] - v1.v[1] * v2.v[0]);
}

inline QVectorD3D QVectorD3D::normal(QVectorD3D v1, QVectorD3D v2) noexcept
{
    return crossProduct(v1, v2).normalized();
}

inline QVectorD3D QVectorD3D::normal(QVectorD3D v1, QVectorD3D v2, QVectorD3D v3) noexcept
{
    return crossProduct((v2 - v1), (v3 - v1)).normalized();
}

inline double QVectorD3D::distanceToPoint(QVectorD3D point) const noexcept
{
    return (*this - point).length();
}

constexpr inline double QVectorD3D::distanceToPlane(QVectorD3D plane, QVectorD3D normal) const noexcept
{
    return dotProduct(*this - plane, normal);
}

inline double QVectorD3D::distanceToPlane(QVectorD3D plane1, QVectorD3D plane2, QVectorD3D plane3) const noexcept
{
    QVectorD3D n = normal(plane2 - plane1, plane3 - plane1);
    return dotProduct(*this - plane1, n);
}

inline double QVectorD3D::distanceToLine(QVectorD3D point, QVectorD3D direction) const noexcept
{
    if (direction.isNull())
        return (*this - point).length();
    QVectorD3D p = point + dotProduct(*this - point, direction) * direction;
    return (*this - p).length();
}

#ifndef QT_NO_VECTOR2D
constexpr inline QVector2D QVectorD3D::toVector2D() const noexcept
{
    return QVector2D(v[0], v[1]);
}
#endif
constexpr inline QVector3D QVectorD3D::toVector3D() const noexcept
{
    return QVector3D(v[0], v[1], v[2]);
}
#ifndef QT_NO_VECTOR4D
constexpr inline QVector4D QVectorD3D::toVector4D() const noexcept
{
    return QVector4D(v[0], v[1], v[2], 0.0f);
}
#endif

constexpr inline QPoint QVectorD3D::toPoint() const noexcept
{
    return QPoint(qRound(v[0]), qRound(v[1]));
}

constexpr inline QPointF QVectorD3D::toPointF() const noexcept
{
    return QPointF(qreal(v[0]), qreal(v[1]));
}

#ifndef QT_NO_DEBUG_STREAM
Q_GUI_EXPORT QDebug operator<<(QDebug dbg, QVectorD3D vector);
#endif

#ifndef QT_NO_DATASTREAM
Q_GUI_EXPORT QDataStream &operator<<(QDataStream &, QVectorD3D );
Q_GUI_EXPORT QDataStream &operator>>(QDataStream &, QVectorD3D &);
#endif

#endif // QT_NO_VECTORD3D

QT_END_NAMESPACE

/***************************** Tuple protocol *****************************/

namespace std {
#ifndef QT_NO_VECTORD3D
    template <>
    class tuple_size<QT_PREPEND_NAMESPACE(QVectorD3D)> : public integral_constant<size_t, 3> {};
    template <>
    class tuple_element<0, QT_PREPEND_NAMESPACE(QVectorD3D)> { public: using type = double; };
    template <>
    class tuple_element<1, QT_PREPEND_NAMESPACE(QVectorD3D)> { public: using type = double; };
    template <>
    class tuple_element<2, QT_PREPEND_NAMESPACE(QVectorD3D)> { public: using type = double; };
#endif // QT_NO_VECTORD3D
}

#endif // QVECTORD3D_H
