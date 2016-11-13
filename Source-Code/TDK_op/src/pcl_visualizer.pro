#-------------------------------------------------
#
# Project created by QtCreator 2014-05-01T14:24:33
#
#-------------------------------------------------

QT += core gui opengl
QT += opengl

LIBS += opengl32.lib

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_visualizer
TEMPLATE = app

# PCL
INCLUDEPATH += "C:/Program Files/PCL 1.6.0/include/pcl-1.6"
INCLUDEPATH += "C:/Program Files/PCL 1.6.0/3rdParty/Eigen/include"
INCLUDEPATH += "C:/Program Files/PCL 1.6.0/3rdParty/Boost/include"
INCLUDEPATH += "C:/Program Files/PCL 1.6.0/3rdParty/Flann/include"
INCLUDEPATH += "C:/Program Files/PCL 1.6.0/3rdParty/VTK/include/vtk-5.8"
#INCLUDEPATH += "C:/Program Files/PCL 1.6.0/3rdParty/Qhull/include"
INCLUDEPATH += "C:/Program Files/OpenNI/Include"

LIBS += opengl32.lib advapi32.lib

LIBS += "-LC:/Program Files/OpenNI/Lib64/openNI64.lib"

LIBS += "-LC:/Program Files/PCL 1.6.0/3rdParty/Qhull/lib/qhullstatic.lib"

LIBS += "-LC:/Program Files/PCL 1.6.0/lib"
LIBS += -lpcl_apps_release
LIBS += -lpcl_common_release
LIBS += -lpcl_filters_release
LIBS += -lpcl_io_release
LIBS += -lpcl_keypoints_release
LIBS += -lpcl_kdtree_release
LIBS += -lpcl_search_release
LIBS += -lpcl_registration_release
LIBS += -lpcl_features_release
LIBS += -lpcl_io_release
LIBS += -lpcl_io_ply_release
LIBS += -lpcl_visualization_release
LIBS += -lpcl_sample_consensus_release
LIBS += -lpcl_surface_release
LIBS += -lpcl_tracking_release
LIBS += -lpcl_segmentation_release


LIBS += "-LC:/Program Files/PCL 1.6.0/3rdParty/VTK/lib/vtk-5.8"
LIBS += -lMapReduceMPI
LIBS += -lmpistubs
LIBS += -lQVTK#-gd
LIBS += -lvtkalglib#-gd
LIBS += -lvtkCharts#-gd
LIBS += -lvtkCommon#-gd
LIBS += -lvtkDICOMParser#-gd
LIBS += -lvtkexoIIc#-gd
LIBS += -lvtkexpat#-gd
LIBS += -lvtkFiltering#-gd
LIBS += -lvtkfreetype#-gd
LIBS += -lvtkftgl#-gd
LIBS += -lvtkGenericFiltering#-gd
LIBS += -lvtkGeovis#-gd
LIBS += -lvtkGraphics#-gd
LIBS += -lvtkhdf5#-gd
LIBS += -lvtkHybrid#-gd
LIBS += -lvtkImaging#-gd
LIBS += -lvtkInfovis#-gd
LIBS += -lvtkIO#-gd
LIBS += -lvtkjpeg#-gd
LIBS += -lvtklibxml2#-gd
LIBS += -lvtkmetaio#-gd
LIBS += -lvtkNetCDF#-gd
LIBS += -lvtkNetCDF_cxx#-gd
LIBS += -lvtkpng#-gd
LIBS += -lvtkproj4#-gd
LIBS += -lvtkRendering#-gd
LIBS += -lvtksqlite#-gd
LIBS += -lvtksys#-gd
LIBS += -lvtktiff#-gd
LIBS += -lvtkverdict#-gd
LIBS += -lvtkViews#-gd
LIBS += -lvtkVolumeRendering#-gd
LIBS += -lvtkWidgets#-gd
LIBS += -lvtkzlib#-gd

LIBS += "-LC:/Program File/PCL 1.6.0/3rdParty/FLANN/lib"
# LIBS += -lflann_cpp_s

LIBS += "-LC:/Program Files/PCL 1.6.0/3rdParty/Boost/lib"
LIBS += -llibboost_date_time-vc100-mt-1_49
LIBS += -llibboost_thread-vc100-mt-1_49
LIBS += -llibboost_filesystem-vc100-mt-1_49
LIBS += -llibboost_system-vc100-mt-1_49
LIBS += -llibboost_iostreams-vc100-mt-1_49


SOURCES += main.cpp \
        pclviewer.cpp \
    TDK_PointOperations.cpp

HEADERS  += pclviewer.h \
    TDK_PointOperations.h

FORMS    += pclviewer.ui
