QT += core gui widgets charts

CONFIG += c++11

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../message_introspection/include \
    src

FORMS += \
    src/gui/form_main.ui

HEADERS += \
    src/data_set.h \
    src/gui/form_main.h

SOURCES += \
    src/data_set.cpp \
    src/gui/form_main.cpp \
    src/main.cpp


DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml
