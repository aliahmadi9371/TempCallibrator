#-------------------------------------------------
#
# Project created by QtCreator 2020-05-06T15:21:13
#
#-------------------------------------------------

QT       += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tempCalibrator
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
    rtdhandler.cpp

HEADERS += \
        mainwindow.h \
    rtdhandler.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/usr/local/lib/release/ -lwiringPi
else:win32:CONFIG(debug, debug|release): LIBS += -L$$/usr/local/lib/debug/ -lwiringPi
else:unix: LIBS += -L$$/usr/local/lib/ -lwiringPi

win32:CONFIG(release, debug|release): LIBS += -L$$/usr/local/lib/release/ -lwiringPiDev
else:win32:CONFIG(debug, debug|release): LIBS += -L$$/usr/local/lib/debug/ -lwiringPiDev
else:unix: LIBS += -L$$/usr/local/lib/ -lwiringPiDev

INCLUDEPATH += $$/usr/local/include
DEPENDPATH += $$/usr/local/include

INCLUDEPATH += $$/usr/local/arm-linux-gnueabihf
DEPENDPATH += $$/usr/local/arm-linux-gnueabihf
