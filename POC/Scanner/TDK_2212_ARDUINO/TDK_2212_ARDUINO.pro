QT += core
QT -= gui
QT += serialport

CONFIG += c++11

TARGET = TDK_2212_ARDUINO
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    tdk_serialportreader.cpp

HEADERS += \
    tdk_serialportreader.h
