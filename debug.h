#ifndef DEBUG_H
#define DEBUG_H

#include <QDebug>
#include <QString>
#include <XPLMUtilities.h>

/**
 * @brief Output for debugging
 *
 * Use this macro to output debugging information for debug builds.
 * Release builds will discard this macro and not output anything.
 * The class and function which called the macro is automatically included
 * in the output, and the macro can be called without any arguments.
 *
 * @see qDebug()
 */
#define DEBUG qDebug() << "Performance-Plugin:" << Q_FUNC_INFO

/**
 * @brief Output for information
 *
 * Use this macro to output information for both debug and release builds
 * to the console. The macro can be called without any arguments.
 *
 * @see qCritical()
 */
#define INFO qCritical() << "Perf:" << Q_FUNC_INFO

#define CONSOLE(x) do { XPLMDebugString(QString(x).append("\n").toUtf8().data()); } while (0)

#endif // DEBUG_H
