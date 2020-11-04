QT += core gui widgets

CONFIG += c++11

INCLUDEPATH += \
    /opt/ros/melodic/include

FORMS += \
    src/gui/form_main.ui

HEADERS += \
    src/gui/form_main.h

SOURCES += \
    src/gui/form_main.cpp \
    src/main.cpp


DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml
