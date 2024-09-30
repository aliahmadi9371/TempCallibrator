#include "mainwindow.h"
#include <QApplication>

#include <QDateTime>
#include <QSettings>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QHash>

#define LOGSIZE 1024 * 10000
#ifdef QT_DEBUG
   #define LOG_TO_FILE false
#else
  #define LOG_TO_FILE true
#endif

void myMessageOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
   QHash<QtMsgType,QString> msgLevelHash;
   msgLevelHash[QtDebugMsg] = "Debug";
   msgLevelHash[QtWarningMsg] = "Warning";
   msgLevelHash[QtCriticalMsg] = "Critical";
   msgLevelHash[QtFatalMsg] = "Fatal";
   QByteArray localMsg = msg.toLocal8Bit();
   QDateTime time = QDateTime::currentDateTime();
   QString formattedTime = time.toString("yyyy-MM-dd hh:mm:ss:zzz");
   QString fileName =  QString("log/Log_%1_.txt")
           .arg(QDate::currentDate().toString("yyyy_MM_dd"));
   QByteArray formattedTimeMsg = formattedTime.toLocal8Bit();
   QString logLevelName = msgLevelHash[type];
   QByteArray logLevelMsg = logLevelName.toLocal8Bit();
   QDir dir;
   if (!dir.exists("log"))
       dir.mkpath("log");
   QFile outFileCheck(fileName);
   QString NewFileName = fileName;
   int size = outFileCheck.size();
   int counter = 0 ;
   while ( size > LOGSIZE ){
       counter++;
       NewFileName = fileName+QString::number(counter);
       QFile outFileCheck(NewFileName);
       size = outFileCheck.size();
   }
   if (LOG_TO_FILE) {
       QString txt = QString("%1 %2: %3 (%4,%5)").arg(formattedTime, logLevelName, msg,  context.file, QString::number(context.line));
       QFile outFile(NewFileName);
       outFile.open(QIODevice::WriteOnly | QIODevice::Append);
       QTextStream ts(&outFile);
       ts << txt << endl;
       outFile.close();
   } else {
       fprintf(stderr, "%s %s: %s (%s:%u, %s)\n", formattedTimeMsg.constData(), logLevelMsg.constData(), localMsg.constData(), context.file, context.line, context.function);
       fflush(stderr);
   }
   if (type == QtFatalMsg)
       abort();
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QSettings setting("config.ini",QSettings::IniFormat);
    bool bDebug = setting.value("app/debug",false).toBool();
        qDebug() << "MAIN: if debug to file -->" << bDebug;

        if(bDebug && LOG_TO_FILE)
            qInstallMessageHandler(myMessageOutput);

    MainWindow w;
    w.show();

    return a.exec();
}
