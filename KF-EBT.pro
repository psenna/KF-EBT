QT += core
QT -= gui

QMAKE_CXXFLAGS -= -O
QMAKE_CXXFLAGS -= -O1
QMAKE_CXXFLAGS -= -O2

QMAKE_CXXFLAGS += -std=c++11 -pedantic -O3 -Wno-long-long -fno-omit-frame-pointer -Wall
QMAKE_CXXFLAGS += -march=corei7 -mtune=corei7

TARGET = KF-EBT
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

INCLUDEPATH += /usr/local/include/
INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/local/include/opencv/

# Opencv
LIBS += -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_features2d -lopencv_ml -lopencv_video -lopencv_calib3d -lopencv_videoio -lopencv_imgcodecs -ltrax

SOURCES += main.cpp \
    trackers/ASMS/colotracker.cpp \
    trackers/ASMS/histogram.cpp \
    trackers/ASMS/region.cpp \
    trackers/kcf/piotr_fhog/gradientMex.cpp \
    trackers/kcf/adjust.cpp \
    trackers/kcf/kcf.cpp \
    kfebt.cpp \
    trackers/tasms.cpp \
    trackers/tkcf.cpp \
    trackers/CBT/consensus/fastcluster/fastcluster.cpp \
    trackers/CBT/consensus/common.cpp \
    trackers/CBT/consensus/Consensus.cpp \
    trackers/tcbt.cpp \
    trackers/CBT/cbt.cpp \
    trackers/CBT/consensus/coloravaliation.cpp \
    trackers/MOSSE/mosse.cpp \
    trackers/NCC/ncc.cpp \
    trackers/tncc.cpp \
    trackers/tmosse.cpp \
    trackers/tvdp.cpp

HEADERS += \
    trackers/ASMS/colotracker.h \
    trackers/ASMS/histogram.h \
    trackers/ASMS/region.h \
    trackers/kcf/piotr_fhog/fhog.hpp \
    trackers/kcf/piotr_fhog/gradientMex.h \
    trackers/kcf/piotr_fhog/sse.hpp \
    trackers/kcf/piotr_fhog/wrappers.hpp \
    trackers/kcf/adjust.h \
    trackers/kcf/complexmat.hpp \
    trackers/kcf/kcf.h \
    trackers/btracker.h \
    kfebt.h \
    vot.h \
    trackers/tasms.h \
    trackers/tkcf.h \
    trackers/CBT/consensus/fastcluster/fastcluster.h \
    trackers/CBT/consensus/common.h \
    trackers/CBT/consensus/Consensus.h \
    trackers/tcbt.h \
    trackers/CBT/cbt.h \
    trackers/CBT/consensus/coloravaliation.h \
    trackers/MOSSE/mosse.h \
    trackers/NCC/ncc.h \
    trackers/tncc.h \
    trackers/tmosse.h \
    trackers/tvdp.h

DISTFILES += \
    trackers/ASMS/CMakeLists.txt \
    trackers/kcf/piotr_fhog/CMakeLists.txt \
    trackers/kcf/CMakeLists.txt \
    trackers/kcf/README.md \
    README.md
