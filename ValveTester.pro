QT       += core gui printsupport serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    command.cpp \
    devicemodel.cpp \
    ledindicator.cpp \
    main.cpp \
    preferencesdialog.cpp \
    template.cpp \
    valveanalyser.cpp

HEADERS += \
    command.h \
    devicemodel.h \
    ledindicator.h \
    preferencesdialog.h \
    template.h \
    valveanalyser.h

FORMS += \
    preferencesdialog.ui \
    valveanalyser.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../CMake/ceres-solver/lib/ -lceres
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../CMake/ceres-solver/lib/ -lceres-debug
else:unix: LIBS += -L$$PWD/../../CMake/ceres-solver/lib/ -lceres

INCLUDEPATH += $$PWD/../../CMake/ceres-solver/include
DEPENDPATH += $$PWD/../../CMake/ceres-solver/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../CMake/ceres-solver/lib/libceres.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../CMake/ceres-solver/lib/libceresd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../CMake/ceres-solver/lib/ceres.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../CMake/ceres-solver/lib/ceres-debug.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../CMake/ceres-solver/lib/libceres.a

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../CMake/glog/lib/ -lglog
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../CMake/glog/lib/ -lglogd
else:unix: LIBS += -L$$PWD/../../CMake/glog/lib/ -lglog

INCLUDEPATH += $$PWD/../../CMake/glog/include
DEPENDPATH += $$PWD/../../CMake/glog/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../CMake/gflags/lib/ -lgflags_static
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../CMake/gflags/lib/ -lgflags_static_debug
else:unix: LIBS += -L$$PWD/../../CMake/gflags/lib/ -lgflags_static

INCLUDEPATH += $$PWD/../../CMake/gflags/include
DEPENDPATH += $$PWD/../../CMake/gflags/include

INCLUDEPATH += $$PWD/../../CMake/eigen3/include/eigen3
DEPENDPATH += $$PWD/../../CMake/eigen3/include/eigen3
