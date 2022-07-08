QT       += core gui printsupport serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    analyser/analyser.cpp \
    analyser/client.cpp \
    analyser/sampleset.cpp \
    ledindicator.cpp \
    main.cpp \
    preferencesdialog.cpp \
    analyser/sample.cpp \
    template.cpp \
    valveanalyser.cpp \
    valvemodel/circuit.cpp \
    valvemodel/device.cpp \
    valvemodel/improvedkorentriode.cpp \
    valvemodel/korentriode.cpp \
    valvemodel/model.cpp \
    valvemodel/modelfactory.cpp \
    valvemodel/parameter.cpp \
    valvemodel/plot.cpp \
    valvemodel/simpletriode.cpp \
    valvemodel/template.cpp \
    valvemodel/triodeaccathodefollower.cpp \
    valvemodel/triodecommoncathode.cpp \
    valvemodel/triodedccathodefollower.cpp \
    valvemodel/uibridge.cpp

HEADERS += \
    analyser/analyser.h \
    analyser/client.h \
    analyser/constants.h \
    analyser/sampleset.h \
    ledindicator.h \
    preferencesdialog.h \
    analyser/sample.h \
    template.h \
    valveanalyser.h \
    valvemodel/circuit.h \
    valvemodel/device.h \
    valvemodel/improvedkorentriode.h \
    valvemodel/korentriode.h \
    valvemodel/model.h \
    valvemodel/modelfactory.h \
    valvemodel/parameter.h \
    valvemodel/plot.h \
    valvemodel/simpletriode.h \
    valvemodel/template.h \
    valvemodel/triodeaccathodefollower.h \
    valvemodel/triodecommoncathode.h \
    valvemodel/triodedccathodefollower.h \
    valvemodel/uibridge.h

FORMS += \
    preferencesdialog.ui \
    valveanalyser.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/ceres-solver/lib/ -lceres
else:win32:CONFIG(debug, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/ceres-solver/lib/ -lceres-debug
else:unix: LIBS += -L$$(CMAKE_PREFIX_PATH)/ceres-solver/lib/ -lceres

INCLUDEPATH += $$(CMAKE_PREFIX_PATH)/ceres-solver/include
DEPENDPATH += $$(CMAKE_PREFIX_PATH)/ceres-solver/include

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(CMAKE_PREFIX_PATH)/ceres-solver/lib/libceres.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(CMAKE_PREFIX_PATH)/ceres-solver/lib/libceresd.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$(CMAKE_PREFIX_PATH)/ceres-solver/lib/ceres.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$(CMAKE_PREFIX_PATH)/ceres-solver/lib/ceres-debug.lib
else:unix: PRE_TARGETDEPS += $$(CMAKE_PREFIX_PATH)/ceres-solver/lib/libceres.a

win32:CONFIG(release, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/glog/lib/ -lglog
else:win32:CONFIG(debug, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/glog/lib/ -lglogd
else:unix: LIBS += -L$$(CMAKE_PREFIX_PATH)/glog/lib/ -lglog

INCLUDEPATH += $$(CMAKE_PREFIX_PATH)/glog/include
DEPENDPATH += $$(CMAKE_PREFIX_PATH)/glog/include

win32:CONFIG(release, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/gflags/lib/ -lgflags_static
else:win32:CONFIG(debug, debug|release): LIBS += -L$$(CMAKE_PREFIX_PATH)/gflags/lib/ -lgflags_static_debug
else:unix: LIBS += -L$$(CMAKE_PREFIX_PATH)/gflags/lib/ -lgflags_static

INCLUDEPATH += $$(CMAKE_PREFIX_PATH)/gflags/include
DEPENDPATH += $$(CMAKE_PREFIX_PATH)/gflags/include

INCLUDEPATH += $$(CMAKE_PREFIX_PATH)/eigen3/include/eigen3
DEPENDPATH += $$(CMAKE_PREFIX_PATH)/eigen3/include/eigen3
