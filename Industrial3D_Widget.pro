#-------------------------------------------------
#
# Project created by QtCreator 2015-11-26T00:19:48
#
#-------------------------------------------------

#CONFIG += c++11
# With C++11 support
greaterThan(QT_MAJOR_VERSION, 4){
CONFIG += c++11
} else {
QMAKE_CXXFLAGS += -std=c++0x
}

QT       += core gui
QT       += opengl
QT       += widgets



greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Industrial3D_Widget
TEMPLATE = app


SOURCES += main.cpp\
    PointCloud.cpp \
    Point3D.cpp \
    GLcamera.cpp \
    GLwidget.cpp \
    mainwindow_.cpp

HEADERS  += \
    K3DTree.h \
    Point3D.h \
    PointCloud.h \
    GLcamera.h \
    GLwidget.h \
    mainwindow_.h

FORMS    += \
    mainwindow_.ui


