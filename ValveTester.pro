QT       += core gui printsupport serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    analyser/analyser.cpp \
    analyser/client.cpp \
    ledindicator.cpp \
    main.cpp \
    preferencesdialog.cpp \
    template.cpp \
    valveanalyser.cpp \
    valvemodel/model/device.cpp \
    valvemodel/model/model.cpp \
    valvemodel/model/modelfactory.cpp \
    valvemodel/model/improvedkorentriode.cpp \
    valvemodel/model/korentriode.cpp \
    valvemodel/model/sample.cpp \
    valvemodel/model/sampleset.cpp \
    valvemodel/model/simpletriode.cpp \
    valvemodel/circuit/circuit.cpp \
    valvemodel/circuit/triodeaccathodefollower.cpp \
    valvemodel/circuit/triodecommoncathode.cpp \
    valvemodel/circuit/triodedccathodefollower.cpp \
    valvemodel/ui/uibridge.cpp \
    valvemodel/ui/plot.cpp \
    valvemodel/ui/parameter.cpp

HEADERS += \
    analyser/analyser.h \
    analyser/client.h \
    ledindicator.h \
    preferencesdialog.h \
    template.h \
    valveanalyser.h \
    valvemodel/constants.h \
    valvemodel/model/device.h \
    valvemodel/model/improvedkorentriode.h \
    valvemodel/model/korentriode.h \
    valvemodel/model/model.h \
    valvemodel/model/modelfactory.h \
    valvemodel/model/sample.h \
    valvemodel/model/sampleset.h \
    valvemodel/model/simpletriode.h \
    valvemodel/circuit/circuit.h \
    valvemodel/circuit/triodeaccathodefollower.h \
    valvemodel/circuit/triodecommoncathode.h \
    valvemodel/circuit/triodedccathodefollower.h \
    valvemodel/ui/uibridge.h \
    valvemodel/ui/plot.h \
    valvemodel/ui/parameter.h

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
