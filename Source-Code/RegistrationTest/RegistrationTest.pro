QT -= gui

CONFIG += c++11 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp \
    registration.cpp

HEADERS += \
    registration.h

# PCL
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\include\pcl-1.8"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\3rdParty\VTK\include\vtk-7.0"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\3rdParty\Boost\include\boost-1_61"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\3rdParty\Qhull\include"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\3rdParty\FLANN\include"
INCLUDEPATH += "C:\Program Files\PCL 1.8.0_1\3rdParty\Eigen\eigen3"
INCLUDEPATH += "C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc"
INCLUDEPATH += "C:\Program Files\OpenNI2\Include"
INCLUDEPATH += "C:\Program Files (x86)\Intel\RSSDK\include"
INCLUDEPATH += "C:\Program Files (x86)\Intel\RSSDK\src\libpxc"

LIBS += opengl32.lib advapi32.lib Ws2_32.lib user32.lib shell32.lib gdi32.lib kernel32.lib
LIBS += "-LC:\Program Files\PCL 1.8.0_1\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0_1\3rdParty\VTK\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0_1\3rdParty\Qhull\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0_1\3rdParty\FLANN\lib"
LIBS += "-LC:\Program Files\PCL 1.8.0_1\3rdParty\Boost\lib"


