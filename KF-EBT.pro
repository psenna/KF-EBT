QT += core
QT -= gui

CONFIG += c++11

TARGET = KF-EBT
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    ASMS/colotracker.cpp \
    ASMS/histogram.cpp \
    ASMS/region.cpp \
    kcf/piotr_fhog/gradientMex.cpp \
    kcf/adjust.cpp \
    kcf/kcf.cpp

HEADERS += \
    ASMS/colotracker.h \
    ASMS/histogram.h \
    ASMS/region.h \
    kcf/piotr_fhog/fhog.hpp \
    kcf/piotr_fhog/gradientMex.h \
    kcf/piotr_fhog/sse.hpp \
    kcf/piotr_fhog/wrappers.hpp \
    kcf/adjust.h \
    kcf/complexmat.hpp \
    kcf/kcf.h

DISTFILES += \
    ASMS/CMakeLists.txt \
    kcf/piotr_fhog/CMakeLists.txt \
    kcf/CMakeLists.txt \
    kcf/README.md \
    README.md
