#include "debug.h"

/*
void TextEditOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    //QByteArray localMsg = msg.toLocal8Bit();
    QString file = context.file ? context.file : "";
    QString function = context.function ? context.function : "";
    switch (type) {
    case QtDebugMsg:
        QString("Debug: %1 (%2:%3, %4)\n").arg(msg).arg(file).arg(context.line).arg(function);
        //fprintf(stderr, "Debug: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtInfoMsg:
        fprintf(stderr, "Info: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtWarningMsg:
        fprintf(stderr, "Warning: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtCriticalMsg:
        fprintf(stderr, "Critical: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    case QtFatalMsg:
        fprintf(stderr, "Fatal: %s (%s:%u, %s)\n", localMsg.constData(), file, context.line, function);
        break;
    }
}
*/
