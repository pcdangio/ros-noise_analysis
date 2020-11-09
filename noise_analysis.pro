QT += core gui widgets charts

CONFIG += c++11

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../message_introspection/include \
    src

FORMS += \
    src/gui/form_array.ui \
    src/gui/form_main.ui

HEADERS += \
    src/data/candidate_field.h \
    src/data/candidate_topic.h \
    src/data/dataset.h \
    src/data/data_interface.h \
    src/graph/chart.h \
    src/gui/form_array.h \
    src/gui/form_main.h

SOURCES += \
    src/data/candidate_field.cpp \
    src/data/dataset.cpp \
    src/data/data_interface.cpp \
    src/graph/chart.cpp \
    src/gui/form_array.cpp \
    src/gui/form_main.cpp \
    src/main.cpp


DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml
