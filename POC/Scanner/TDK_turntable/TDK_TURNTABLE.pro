QT += core
QT -= gui
QT += serialport

CONFIG += c++11

TARGET = TDK_TURNTABLE
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    tdk_turntable.cpp

HEADERS += \
    tdk_turntable.h
