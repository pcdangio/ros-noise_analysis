QT += core gui widgets charts

CONFIG += c++11

INCLUDEPATH += \
    /opt/ros/melodic/include \
    ../message_introspection/include \
    src

FORMS += \
    src/gui/form_field.ui \
    src/gui/form_main.ui

HEADERS += \
    src/data_set.h \
    src/gui/form_field.h \
    src/gui/form_main.h \
    src/models/definition_tree_item.h \
    src/models/definition_tree_model.h

SOURCES += \
    src/data_set.cpp \
    src/gui/form_field.cpp \
    src/gui/form_main.cpp \
    src/main.cpp \
    src/models/definition_tree_item.cpp \
    src/models/definition_tree_model.cpp


DISTFILES += \
    CMakeLists.txt \
    LICENSE \
    README.md \
    package.xml
